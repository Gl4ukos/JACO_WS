#include "ros/ros.h"
#include "sensor_msgs/JointState.h"



/* this republishes joint_states information to "/joint_states" bc this is the topic that moveit reads*/
void callback(const sensor_msgs::JointState::ConstPtr& msg, ros::Publisher* pub){
    pub->publish(*msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle nh;

    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",10);
    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/jaco_driver/out/joint_state", 10 ,boost::bind(callback, _1, &joint_state_pub));

    ros::spin();
}