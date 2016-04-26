// example_action_server: a simple action server
// Wyatt Newman

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include<pcl_utils/pcl_utils.h>
#include <pcl/filters/passthrough.h>
#include<object_finder/objectFinderAction.h>
#include <tf/tf.h>

#define CAN_HEIGHT .13
#define MIN_CLOUD_SIZE 100
#define RED 96
#define GREEN 33
#define BLUE 49
#define COLOR_ERR 40
#define MIN_X .3
#define MAX_X .7

class ObjectFinder {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<object_finder::objectFinderAction> object_finder_as_;
    
    // here are some message types to communicate with our client(s)
    object_finder::objectFinderGoal goal_; // goal message, received from client
    object_finder::objectFinderResult result_; // put results here, to be sent back to the client when done w/ goal
    object_finder::objectFinderFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    PclUtils pclUtils_;
    tf::TransformListener tf_listener;
    tf::StampedTransform tf_sensor_frame_to_torso_frame; //use this to transform sensor frame to torso frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr can_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_transformed_cloud;
    ros::Publisher pubCloud;
    Eigen::Vector3f centroid;
    float surface_height;

    //specialized function to find an upright Coke can on known height of horizontal surface;
    // returns true/false for found/not-found, and if found, fills in the object pose
    bool find_upright_coke_can(geometry_msgs::PoseStamped &object_pose);
    tf::StampedTransform wait_for_transform();
    void transform_kinect_cloud();
    void filter_kinect_cloud();
    void publish_can_cloud();
    bool can_exists();

public:
    ObjectFinder(); //define the body of the constructor outside of class definition

    ~ObjectFinder(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr& goal);
};



ObjectFinder::ObjectFinder() :
        object_finder_as_(nh_, "objectFinderActionServer", boost::bind(&ObjectFinder::executeCB, this, _1), false), pclUtils_(&nh_),
        can_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
        temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
        kinect_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>) {
    ROS_INFO("in constructor of ObjectFinder...");
    // do any other desired initializations here...specific to your implementation

    object_finder_as_.start(); //start the server running
    ROS_INFO("Started action server");
    pubCloud = nh_.advertise<sensor_msgs::PointCloud2> ("can", 1, true);
    ROS_INFO("Created 'can' publisher");
}

tf::StampedTransform ObjectFinder::wait_for_transform() {
    //let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr) {
        tferr = false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame.
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("base_link", "camera_rgb_optical_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //tf-listener found a complete chain from sensor to world; ready to roll
    return tf_sensor_frame_to_torso_frame;
}
void ObjectFinder::transform_kinect_cloud() {
    //convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_sensor_wrt_torso;
    A_sensor_wrt_torso = pclUtils_.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    //transform the kinect data to the torso frame;
    // we don't need to have it returned; pcl_utils can own it as a member var
    ROS_INFO("Transforming kinect cloud");
    pclUtils_.transform_kinect_clr_cloud(A_sensor_wrt_torso);
}

void ObjectFinder::filter_kinect_cloud() {
    // First clear all our clouds
    can_cloud->clear();
    temp_cloud->clear();

    // Save transformed kinect data into PointCloud object that we can manipulate
    ROS_INFO("Getting transformed kinect cloud");
    pclUtils_.get_kinect_transformed_points(kinect_transformed_cloud);

    // Filter the kinect cloud to just contain points that could feasibly be a part of the can based on height
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object

    pass.setInputCloud(kinect_transformed_cloud); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    pass.setFilterLimits(surface_height, surface_height + CAN_HEIGHT); //here is the range of z values
    //std::vector<int> indices;
    ROS_INFO("Filtering cloud by z height");
    pass.filter(*can_cloud); // Store the filtered cloud in the can_cloud container
    //ROS_INFO_STREAM( indices.size() << " indices passed by z filter.");
    ROS_INFO_STREAM("Z filtered cloud has " << can_cloud->size() << " points");
    // Now we will filter by x and store it in the temp_cloud container
    ROS_INFO("Filtering cloud by x distance");
    pass.setInputCloud(can_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(MIN_X, MAX_X);
    pass.filter(*temp_cloud);
    can_cloud->clear(); // Clear the can_cloud so that it can store newly filtered points later
    ROS_INFO_STREAM("X filtered cloud has " << temp_cloud->size() << " points");
    // Set properties of can_cloud
    //can_cloud->points.resize(indices.size());
    //can_cloud->header.frame_id = "base_link";

    // Now add points that passed the height filter into the can_cloud
    // But only add them if they are approximately red in color
    ROS_INFO("Filtering cloud by color");
    Eigen::Vector3i color;
    for (unsigned i = 0; i < temp_cloud->size(); i++) {
        color = temp_cloud->points[i].getRGBVector3i();
        if (abs(color(0) - RED) < COLOR_ERR && abs(color(1) - GREEN) < COLOR_ERR && abs(color(2) - BLUE) < COLOR_ERR) {
            can_cloud->points.push_back(temp_cloud->points[i]);
        }
    }
    ROS_INFO_STREAM("Final can cloud has " << can_cloud->size() << " points");
}

void ObjectFinder::publish_can_cloud() {

    sensor_msgs::PointCloud2 ros_can_cloud;
    pcl::toROSMsg(*can_cloud, ros_can_cloud);

    while (ros::ok) {
        pubCloud.publish(ros_can_cloud); // will not need to keep republishing if display setting is persistent
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
}

bool ObjectFinder::can_exists() {
    if (can_cloud->points.size() > MIN_CLOUD_SIZE) {
        ROS_INFO("Can is present on table");
        return true;
    }
    else {
        ROS_INFO("No can detected");
        return false;
    }
}

//specialized function: DUMMY...JUST RETURN A HARD-CODED POSE; FIX THIS
bool ObjectFinder::find_upright_coke_can(geometry_msgs::PoseStamped &object_pose) {
    bool found_object=false;

    wait_for_transform();
    transform_kinect_cloud();
    filter_kinect_cloud();
    if (can_exists()) {
        centroid = pclUtils_.compute_centroid(can_cloud);

        object_pose.header.frame_id = "base_link";
        object_pose.pose.position.x = centroid(0);
        object_pose.pose.position.y = centroid(1);
        object_pose.pose.position.z = surface_height;
        object_pose.pose.orientation.x = 0;
        object_pose.pose.orientation.y = 0;
        object_pose.pose.orientation.z = 0;
        object_pose.pose.orientation.w = 1;
        found_object = true;
        publish_can_cloud(); // TODO remove this once filters are tuned since it will hang indefinitely
        // TODO might need to set orientation to orientation we want gripper to be at, or just manually set gripper orientation in object_grabber
    }

    return found_object;
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void ObjectFinder::executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr& goal) {
    int object_id = goal->object_id;
    geometry_msgs::PoseStamped object_pose;
    bool known_surface_ht = goal->known_surface_ht;
    if (known_surface_ht) {
        surface_height=goal->surface_ht;
    }
    bool found_object = false;
    switch(object_id) {
        case object_finder::objectFinderGoal::COKE_CAN_UPRIGHT: 
              //specialized function to find an upright Coke can on a horizontal surface of known height:
               found_object = find_upright_coke_can(object_pose); //special case for Coke can;
               if (found_object) {
                   ROS_INFO("found upright Coke can!");
                   result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                   result_.object_pose = object_pose;
                   object_finder_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not find requested object");
                   object_finder_as_.setAborted(result_);
               }
               break;
        default:
             ROS_WARN("this object ID is not implemented");
             result_.found_object_code = object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED; 
             object_finder_as_.setAborted(result_);
            }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_action_server_node"); // name this node 

    ROS_INFO("instantiating the demo action server: ");

    ObjectFinder object_finder_as; // create an instance of the class "ObjectFinder"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }

    return 0;
}

