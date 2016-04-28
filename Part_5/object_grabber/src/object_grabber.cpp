//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses
#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_planner/baxter_cart_moveAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include<geometry_msgs/PoseStamped.h>
#include <object_grabber/object_grabberAction.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>

#define X_OFFSET 0
#define Y_OFFSET 0.05
#define Z_OFFSET 0

class ObjectGrabber {
private:
    ros::NodeHandle nh_;

    tf::TransformListener tf_listener;
    //messages to send/receive cartesian goals / results:
    object_grabber::object_grabberGoal grab_goal_;
    object_grabber::object_grabberResult grab_result_; 
    object_grabber::object_grabberFeedback grab_fdbk_;    
    geometry_msgs::PoseStamped object_pose_stamped_;
    int object_code_;
    std_msgs::Bool gripper_open,gripper_close;
    
    double gripper_theta_;
    double z_depart_,L_approach_;
    double gripper_table_z_;
    Eigen::Vector3d gripper_b_des_;
    Eigen::Vector3d gripper_n_des_;
    Eigen::Vector3d gripper_t_des_;
    Eigen::Vector3d grasp_origin_,approach_origin_,depart_origin_;
    Eigen::Matrix3d R_gripper_vert_cyl_grasp_;
    Eigen::Affine3d a_gripper_start_,a_gripper_end_;
    Eigen::Affine3d a_gripper_approach_,a_gripper_depart_, a_gripper_grasp_;
    
    ros::Publisher gripper_publisher;
    
    actionlib::SimpleActionServer<object_grabber::object_grabberAction> object_grabber_as_;
    //action callback fnc
    void executeCB(const actionlib::SimpleActionServer<object_grabber::object_grabberAction>::GoalConstPtr& goal);   
    void vertical_cylinder_power_grasp(geometry_msgs::PoseStamped object_pose);    
    tf::StampedTransform wait_for_transform(std_msgs::Header header);

public:
    ObjectGrabber(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition

    ~ObjectGrabber(void) {
    }
    //define some member methods here

};


//name this server; 
ObjectGrabber::ObjectGrabber(ros::NodeHandle* nodehandle): nh_(*nodehandle),
   object_grabber_as_(nh_, "objectGrabberActionServer", boost::bind(&ObjectGrabber::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of ObjectGrabber");
    // do any other desired initializations here, as needed
    gripper_table_z_ = -0.145; //gripper origin height above torso for grasp of cyl on table
    L_approach_ = 0.25; //distance to slide towards cylinder
    z_depart_ = 0.2; //height to lift cylinder
    //define a gripper orientation for power-grasp approach of upright cylinder
    gripper_n_des_ << 0, 0, 1; //gripper x-axis points straight up;
    gripper_theta_ = M_PI/3.0; //approach yaw angle--try this, reaching out and to the left
    gripper_b_des_ << cos(gripper_theta_), sin(gripper_theta_), 0;
    gripper_t_des_ = gripper_b_des_.cross(gripper_n_des_);
    R_gripper_vert_cyl_grasp_.col(0) = gripper_n_des_;
    R_gripper_vert_cyl_grasp_.col(1) = gripper_t_des_;
    R_gripper_vert_cyl_grasp_.col(2) = gripper_b_des_;
    a_gripper_start_.linear() = R_gripper_vert_cyl_grasp_;
    a_gripper_end_.linear() = R_gripper_vert_cyl_grasp_;   
    //define approach, grasp and depart poses: 
    a_gripper_approach_.linear() = R_gripper_vert_cyl_grasp_;
    a_gripper_depart_.linear() = R_gripper_vert_cyl_grasp_;
    a_gripper_grasp_.linear() = R_gripper_vert_cyl_grasp_;
    
    gripper_open.data= false;
    gripper_close.data=true;
    gripper_publisher = nh_.advertise<std_msgs::Bool>("gripper_open_close",1,true);

    object_grabber_as_.start(); //start the server running
}

tf::StampedTransform ObjectGrabber::wait_for_transform(std_msgs::Header header) {
    //let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    tf::StampedTransform tf_target_frame_to_source_frame;
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr) {
        tferr = false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame.
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("torso", header.frame_id, ros::Time(0), tf_target_frame_to_source_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //tf-listener found a complete chain from sensor to world; ready to roll
    return tf_target_frame_to_source_frame;
}

void ObjectGrabber::vertical_cylinder_power_grasp(geometry_msgs::PoseStamped object_pose) {
    ROS_INFO("Executing vertical_cylinder_power_grasp");

    ROS_INFO("Waiting for transform between object_pose frame_id and torso");
    tf::StampedTransform tf_header_to_torso = wait_for_transform(object_pose.header);

    geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose, object_transformed_pose;
    //make sure the object pose is in the torso frame; transform if necessary;
    if (object_pose.header.frame_id == "torso") {
        ROS_INFO("Given object pose already in the torso frame");
        object_transformed_pose = object_pose;
    }
    else {
        ROS_INFO("Given object pose not in the torso frame, transforming now");
        tf_listener.transformPose("torso", object_pose, object_transformed_pose);
    }

    //skip this for now
    int rtn_val;
       //send a command to plan a joint-space move to pre-defined pose:
    rtn_val = g_arm_motion_commander_ptr->plan_move_to_pre_pose();

    //send command to execute planned motion
    ROS_INFO("Going to pre-pose");
    rtn_val=g_arm_motion_commander_ptr->rt_arm_execute_planned_path();

    //inquire re/ right-arm joint angles:
    rtn_val=g_arm_motion_commander_ptr->rt_arm_request_q_data();

    Eigen::Affine3d object_affine;
    object_affine = g_arm_motion_commander_ptr->transformPoseToEigenAffine3d(object_transformed_pose.pose);
    Eigen::Vector3d object_origin;
    object_origin = object_affine.translation();
    grasp_origin_ = object_origin; //grasp origin is same as object origin...
    grasp_origin_(0) += X_OFFSET;
    grasp_origin_(1) += Y_OFFSET;
    grasp_origin_(2) += Z_OFFSET;
    if (grasp_origin_(2) < gripper_table_z_) {
        ROS_WARN("Grasp origin z too low, resetting");
        grasp_origin_(2) = gripper_table_z_;
    }
    //grasp_origin_(2) = gripper_table_z_;//except elevate the gripper for table clearance
    a_gripper_grasp_.translation() = grasp_origin_;

    //to slide sideways to approach, compute a pre-grasp approach pose;
    // corresponds to backing up along gripper-z axis by distance L_approach:
    approach_origin_ = grasp_origin_ - gripper_b_des_*L_approach_;
    a_gripper_approach_.translation() = approach_origin_;

    // after have cylinder grasped, move purely upwards by z_depart:
    depart_origin_ = grasp_origin_ + gripper_n_des_*z_depart_;
    a_gripper_depart_.translation() = depart_origin_;

    //open the gripper:
    gripper_publisher.publish(gripper_open);

    //start w/ a jnt-space move from current pose to approach pose:
    int planner_rtn_code;
    des_gripper_approach_pose.header.frame_id = "torso";
    des_gripper_approach_pose.pose = g_arm_motion_commander_ptr->transformEigenAffine3dToPose(a_gripper_approach_);
    des_gripper_approach_pose.pose.orientation = object_transformed_pose.pose.orientation;
    planner_rtn_code = g_arm_motion_commander_ptr->rt_arm_plan_path_current_to_goal_pose(des_gripper_approach_pose);

    //try to move here:
    g_arm_motion_commander_ptr->rt_arm_execute_planned_path();

    //slide to can:
    des_gripper_grasp_pose.header.frame_id = "torso";
    des_gripper_grasp_pose.pose = g_arm_motion_commander_ptr->transformEigenAffine3dToPose(a_gripper_grasp_);
    des_gripper_grasp_pose.pose.orientation = object_transformed_pose.pose.orientation;
    planner_rtn_code = g_arm_motion_commander_ptr->rt_arm_plan_path_current_to_goal_pose(des_gripper_grasp_pose);
    g_arm_motion_commander_ptr->rt_arm_execute_planned_path();

    //close the gripper:
    gripper_publisher.publish(gripper_close);
    //wait for gripper to close:
    ros::Duration(2.0).sleep();

    //depart vertically:
    des_gripper_depart_pose.header.frame_id = "torso";
    des_gripper_depart_pose.pose = g_arm_motion_commander_ptr->transformEigenAffine3dToPose(a_gripper_depart_);
    des_gripper_depart_pose.pose.orientation = object_transformed_pose.pose.orientation;
    planner_rtn_code = g_arm_motion_commander_ptr->rt_arm_plan_path_current_to_goal_pose(des_gripper_depart_pose);
    g_arm_motion_commander_ptr->rt_arm_execute_planned_path();
} 


//callback: at present, hard-coded for Coke-can object;
//extend this to add more grasp strategies for more objects
// also, this code does NO error checking (e.g., unreachable); needs to be fixed!
void ObjectGrabber::executeCB(const actionlib::SimpleActionServer<object_grabber::object_grabberAction>::GoalConstPtr& goal) {
 
   int object_code = goal->object_code;
   geometry_msgs::PoseStamped object_pose = goal->object_frame;
   switch(object_code) {
   case object_grabber::object_grabberGoal::COKE_CAN: 
     vertical_cylinder_power_grasp(object_pose);
     grab_result_.return_code = object_grabber::object_grabberResult::OBJECT_ACQUIRED; 
     object_grabber_as_.setSucceeded(grab_result_);
     break;
   default:
             ROS_WARN("this object ID is not implemented");
             grab_result_.return_code = object_grabber::object_grabberResult::FAILED_OBJECT_UNKNOWN; 
             object_grabber_as_.setAborted(grab_result_);
            }
     
   
    //grab_result_.return_code = object_grabber::object_grabberResult::OBJECT_ACQUIRED; 
  
    //object_grabber_as_.setAborted(grab_result_); 
    //object_grabber_as_.setSucceeded(grab_result_); 
}
