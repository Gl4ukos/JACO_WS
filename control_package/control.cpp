
#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_comm.h"
#include "kinova_driver/kinova_arm.h"
#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"
#include "kinova_driver/kinova_joint_trajectory_controller.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt16.h>
#include <iostream>
#include <string>
#include <fstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#define ANGULAR_HANDLING 0 /* SET TO 1 IF U WISH TO USE ANGULAR CONTROL IN PROBLEMATIC POSES*/


ros::Publisher servo_pub;
std_msgs::UInt16 servo_value;


ros::Publisher status_pub;
std_msgs::String arm_movement_status;

geometry_msgs::Pose board_pos_l2 [8][8];
geometry_msgs::Pose board_pos_l1 [8][8];


float tile_len = 0.045; // originally 0.055
// basic pose msgs
geometry_msgs::Pose home_pose_msg;
geometry_msgs::Pose start_pose_msg;
geometry_msgs::Pose start_l2_pose_msg;
geometry_msgs::Pose test_pose_msg;
geometry_msgs::Pose out_pose_msg;

kinova::KinovaAngles curr_angles;
kinova::KinovaAngles home_angles; 
kinova::KinovaAngles safe_angles; 
kinova::KinovaAngles c1_angles, c1_low_angles;
kinova::KinovaAngles d1_angles, d1_low_angles;
kinova::KinovaAngles e1_angles, e1_low_angles;
kinova::KinovaAngles f1_angles, f1_low_angles;


int jaco_position_x;
int jaco_position_y;
int jaco_position_layer;

int order_x;
int order_y;

/* Loads coordinates stored in  files into the essential poses: 
    -> home
    -> start: The a1 tile's coordinates
    -> start_l2: Same as "start" but a little higher
    -> test_pose: for testing if the arm can reach the current pose
    -> c1_angles etc: Angular poses for when the arm cant approach cartesian coords given due to sungularities occurring
 */
void init_board(){

    float a,b,c,d,e,f,g,   a2,b2,c2,d2,e2,f2,g2;
    // getting home position coords
    std::ifstream file("/home/orng/jaco_gamiesai/src/jaco_control/src/position_coords/home");
    file>>a>>b>>c>>d>>e>>f>>g;
    home_pose_msg.position.x = a;
    home_pose_msg.position.y = b;
    home_pose_msg.position.z = c;
    home_pose_msg.orientation.x = d;
    home_pose_msg.orientation.y = e;
    home_pose_msg.orientation.z = f;
    home_pose_msg.orientation.w = g;
    file.close();
    
    // loading start position coords
    file.open("/home/orng/jaco_gamiesai/src/jaco_control/src/position_coords/start");
    file>>a>>b>>c>>d>>e>>f>>g;
    start_pose_msg.position.x = a;
    start_pose_msg.position.y = b;
    start_pose_msg.position.z = c;
    start_pose_msg.orientation.x = d;
    start_pose_msg.orientation.y = e;
    start_pose_msg.orientation.z = f;
    start_pose_msg.orientation.w = g;
    file.close();

    file.open("/home/orng/jaco_gamiesai/src/jaco_control/src/position_coords/start_l2");
    file>>a>>b>>c>>d>>e>>f>>g;
    start_l2_pose_msg.position.x = a;
    start_l2_pose_msg.position.y = b;
    start_l2_pose_msg.position.z = c;
    start_l2_pose_msg.orientation.x = d;
    start_l2_pose_msg.orientation.y = e;
    start_l2_pose_msg.orientation.z = f;
    start_l2_pose_msg.orientation.w = g;
    file.close();

    file.open("/home/orng/jaco_gamiesai/src/jaco_control/src/position_coords/test_pose");
    file>>a>>b>>c>>d>>e>>f>>g;
    test_pose_msg.position.x = a;
    test_pose_msg.position.y = b;
    test_pose_msg.position.z = c;
    test_pose_msg.orientation.x = d;
    test_pose_msg.orientation.y = e;
    test_pose_msg.orientation.z = f;
    test_pose_msg.orientation.w = g;
    file.close();

    for(int i=0; i<8; i++){
        for (int j=0; j<8; j++){
            board_pos_l2[i][j].position.x = start_l2_pose_msg.position.x -(tile_len*j);
            board_pos_l2[i][j].position.y = start_l2_pose_msg.position.y - (tile_len*i);
            board_pos_l2[i][j].position.z = start_l2_pose_msg.position.z;
            board_pos_l2[i][j].orientation.x = start_l2_pose_msg.orientation.x;
            board_pos_l2[i][j].orientation.y = start_l2_pose_msg.orientation.y;
            board_pos_l2[i][j].orientation.z = start_l2_pose_msg.orientation.z;
            board_pos_l2[i][j].orientation.w = start_l2_pose_msg.orientation.w;

            board_pos_l1[i][j].position.x = start_pose_msg.position.x -(tile_len*j);
            board_pos_l1[i][j].position.y = start_pose_msg.position.y - (tile_len*i);
            board_pos_l1[i][j].position.z = start_pose_msg.position.z;
            board_pos_l1[i][j].orientation.x = start_pose_msg.orientation.x;
            board_pos_l1[i][j].orientation.y = start_pose_msg.orientation.y;
            board_pos_l1[i][j].orientation.z = start_pose_msg.orientation.z;
            board_pos_l1[i][j].orientation.w = start_pose_msg.orientation.w;

        }
    }

    out_pose_msg.position.x = start_l2_pose_msg.position.x + (tile_len);
    out_pose_msg.position.y = start_l2_pose_msg.position.y + (tile_len);
    out_pose_msg.position.z = start_l2_pose_msg.position.z;
    out_pose_msg.orientation.x = start_l2_pose_msg.orientation.x;
    out_pose_msg.orientation.y = start_l2_pose_msg.orientation.y;
    out_pose_msg.orientation.z = start_l2_pose_msg.orientation.z;
    out_pose_msg.orientation.w = start_l2_pose_msg.orientation.w;


    //adding angle information to problematic poses that when given coordinates cause singularities
    file.open("/home/orng/jaco_gamiesai/src/jaco_control/src/position_coords/problematic_position_angles/c1");
    file>>a>>b>>c>>d>>e>>f>>g>>a2>>b2>>c2>>d2>>e2>>f2>>g2;
    c1_angles.Actuator1 = a;
    c1_angles.Actuator2 = b;
    c1_angles.Actuator3 = c;
    c1_angles.Actuator4 = d;
    c1_angles.Actuator5 = e;
    c1_angles.Actuator6 = f;
    c1_angles.Actuator7 = g;

    c1_low_angles.Actuator1 = a2;
    c1_low_angles.Actuator2 = b2;
    c1_low_angles.Actuator3 = c2;
    c1_low_angles.Actuator4 = d2;
    c1_low_angles.Actuator5 = e2;
    c1_low_angles.Actuator6 = f2;
    c1_low_angles.Actuator7 = g2;

    file.close();

    file.open("/home/orng/jaco_gamiesai/src/jaco_control/src/position_coords/problematic_position_angles/d1");
    file>>a>>b>>c>>d>>e>>f>>g>>a2>>b2>>c2>>d2>>e2>>f2>>g2;
    d1_angles.Actuator1 = a;
    d1_angles.Actuator2 = b;
    d1_angles.Actuator3 = c;
    d1_angles.Actuator4 = d;
    d1_angles.Actuator5 = e;
    d1_angles.Actuator6 = f;
    d1_angles.Actuator7 = g;

    d1_low_angles.Actuator1 = a2;
    d1_low_angles.Actuator2 = b2;
    d1_low_angles.Actuator3 = c2;
    d1_low_angles.Actuator4 = d2;
    d1_low_angles.Actuator5 = e2;
    d1_low_angles.Actuator6 = f2;
    d1_low_angles.Actuator7 = g2;
    file.close();

    file.open("/home/orng/jaco_gamiesai/src/jaco_control/src/position_coords/problematic_position_angles/e1");
    file>>a>>b>>c>>d>>e>>f>>g>>a2>>b2>>c2>>d2>>e2>>f2>>g2;
    e1_angles.Actuator1 = a;
    e1_angles.Actuator2 = b;
    e1_angles.Actuator3 = c;
    e1_angles.Actuator4 = d;
    e1_angles.Actuator5 = e;
    e1_angles.Actuator6 = f;
    e1_angles.Actuator7 = g;

    e1_low_angles.Actuator1 = a2;
    e1_low_angles.Actuator2 = b2;
    e1_low_angles.Actuator3 = c2;
    e1_low_angles.Actuator4 = d2;
    e1_low_angles.Actuator5 = e2;
    e1_low_angles.Actuator6 = f2;
    e1_low_angles.Actuator7 = g2;
    file.close();

    file.open("/home/orng/jaco_gamiesai/src/jaco_control/src/position_coords/problematic_position_angles/f1");
    file>>a>>b>>c>>d>>e>>f>>g>>a2>>b2>>c2>>d2>>e2>>f2>>g2;
    f1_angles.Actuator1 = a;
    f1_angles.Actuator2 = b;
    f1_angles.Actuator3 = c;
    f1_angles.Actuator4 = d;
    f1_angles.Actuator5 = e;
    f1_angles.Actuator6 = f;
    f1_angles.Actuator7 = g;

    f1_low_angles.Actuator1 = a2;
    f1_low_angles.Actuator2 = b2;
    f1_low_angles.Actuator3 = c2;
    f1_low_angles.Actuator4 = d2;
    f1_low_angles.Actuator5 = e2;
    f1_low_angles.Actuator6 = f2;
    f1_low_angles.Actuator7 = g2;

    file.close();


    file.open("/home/orng/jaco_gamiesai/src/jaco_control/src/position_coords/problematic_position_angles/home");
    file>>a>>b>>c>>d>>e>>f>>g;
    home_angles.Actuator1 = a;
    home_angles.Actuator2 = b;
    home_angles.Actuator3 = c;
    home_angles.Actuator4 = d;
    home_angles.Actuator5 = e;
    home_angles.Actuator6 = f;
    home_angles.Actuator7 = g;
    file.close();

    file.open("/home/orng/jaco_gamiesai/src/jaco_control/src/position_coords/problematic_position_angles/safe");
    file>>a>>b>>c>>d>>e>>f>>g;
    safe_angles.Actuator1 = a;
    safe_angles.Actuator2 = b;
    safe_angles.Actuator3 = c;
    safe_angles.Actuator4 = d;
    safe_angles.Actuator5 = e;
    safe_angles.Actuator6 = f;
    safe_angles.Actuator7 = g;
    file.close();




}


void open_gripper(){
    servo_value.data = 1;
    servo_pub.publish(servo_value);
    ROS_INFO("OPENING GRIPPER");
}
void close_gripper(){
    servo_value.data = 0;
    servo_pub.publish(servo_value);
    ROS_INFO("CLOSING GRIPPER");
}
void set_status_busy(){
    arm_movement_status.data = "busy";
    status_pub.publish(arm_movement_status);
    ros::spinOnce();
}
void set_status_ready(){
    arm_movement_status.data = "ready";
    status_pub.publish(arm_movement_status);
    ros::spinOnce();
}

/* returns whether the arm has arrived, when in cartesian control*/
bool arrived(kinova::KinovaPose goal, kinova::KinovaPose curr, double thresh){
    double x_diff = abs(curr.X - goal.X); 
    double y_diff = abs(curr.Y - goal.Y);
    double z_diff = abs(curr.Z - goal.Z);

    bool result = (x_diff<thresh) && (y_diff<thresh) && (z_diff<thresh);
    return (result);
}
/* returns whether the arm has arrived, when in angular control */
bool arrived_angled(kinova::KinovaAngles goal, kinova::KinovaAngles curr, double thresh){
    double diff1 = abs(curr.Actuator1 - goal.Actuator1);
    double diff2 = abs(curr.Actuator2 - goal.Actuator2);
    double diff3 = abs(curr.Actuator3 - goal.Actuator3);
    double diff4 = abs(curr.Actuator4 - goal.Actuator4);
    double diff5 = abs(curr.Actuator5 - goal.Actuator5);
    double diff6 = abs(curr.Actuator6 - goal.Actuator6);
    double diff7 = abs(curr.Actuator7 - goal.Actuator7);

    bool result = (diff1<thresh) && (diff2<thresh) && (diff3<thresh) && (diff4<thresh) && (diff5<thresh) && (diff6<thresh) && (diff7<thresh);
    return result;
}

std::string move;
int active;
/* Handles the input from "move_generator.py" script*/
int_least64_t position_control(kinova::KinovaComm control)
{

    kinova::KinovaPose home_pose(home_pose_msg);
    kinova::KinovaPose test_pose(test_pose_msg);
    kinova::KinovaPose temp_pose;

    control.eraseAllTrajectories();

    open_gripper();

    ros::spinOnce();
    sleep(1);
    close_gripper();
    ros::spinOnce();
    sleep(1);

    while (ros::ok()){
            int count=0;
        try{
            ros::spinOnce();
            if(active){
                open_gripper();
                ros::spinOnce(); 
                set_status_busy();
                if(move==""){
                }else if(move=="print"){
                    kinova::KinovaPose pose;
                    control.getCartesianPosition(pose);
                    control.printPosition(pose);
                }else if(move=="help"){
                    ROS_INFO("AVAILABLE COMMANDS: \n\t print, home, grab, let, test, <chess move such as:> \"a1c1\"\n");
                }else if(move == "angles"){
                    control.getJointAngles(curr_angles);
                    control.printAngles(curr_angles);
                }else if(move == "home"){
                    control.setCartesianPosition(home_pose, 0, true);
                    ROS_INFO("HOMING");
                }else if(move == "grab"){
                    close_gripper();
                    ros::spinOnce();
                }else if(move == "let"){
                    open_gripper();
                    ros::spinOnce();
                }else if (move == "test"){
                    try{
                        control.eraseAllTrajectories();
                        for(int i=0; i<8; i++){
                            for (int j=0; j<8; j++){
                                
                                    ros::spinOnce();
                                    kinova::KinovaPose l2_pose(board_pos_l2[i][j]);
                                    control.setCartesianPosition(l2_pose, 0, true);
                                    ROS_INFO("MOVING TO %d-%d", i,j);
                                    control.getCartesianPosition(temp_pose);
                                    while(!arrived(l2_pose, temp_pose, 0.01)){
                                        sleep(1);
                                        if(count>10){
                                            throw -1;
                                            break;
                                        }else{
                                            count+=1;
                                            control.getCartesianPosition(temp_pose);
                                        }
                                    }

                                    
                                    sleep(0.2);
                                    ros::spinOnce();
                                    kinova::KinovaPose l1_pose(board_pos_l1[i][j]);
                                    control.setCartesianPosition(l1_pose, 0, true);
                                    control.getCartesianPosition(temp_pose);
                                    while(!arrived(l1_pose, temp_pose, 0.01)){
                                        sleep(1);
                                        if(count>10){
                                            throw -1;
                                            break;
                                        }else{
                                            count+=1;
                                            control.getCartesianPosition(temp_pose);
                                        }
                                    }
                                    
                                    
                                    sleep(0.2);
                                    ros::spinOnce();
                                    control.setCartesianPosition(l2_pose, 0, true);
                                    control.getCartesianPosition(temp_pose);
                                    while(!arrived(l2_pose, temp_pose, 0.01)){
                                        sleep(1);
                                        if(count>10){
                                            throw -1;
                                            break;
                                        }else{
                                            count+=1;
                                            control.getCartesianPosition(temp_pose);
                                        }
                                    }
                                    
                                    ROS_INFO("ARRIVED AT %d-%d", i,j);
                                    count=0;
                                    sleep(1);
                            }
                        }
                    }catch(...){
                        ROS_INFO("MOVE TIMEOUT!");
                        sleep(1);
                        while(ros::ok()){
                            ROS_INFO("EXPECTING MANUAL SHUTDOWN...");
                            sleep(5);
                        }
                    }
                    ROS_INFO("TEST COMPLETE. NO FREEZES!");
                    sleep(3);
                }else{
                    //decoding move
                    char coord1_x = move[0];
                    char coord1_y = move[1];
                    char coord2_x = move[2];
                    char coord2_y = move[3];

                    int x1 = coord1_x - 'a';
                    int y1 = coord1_y - '0' -1;
                    int x2 = coord2_x -'a';
                    int y2 = coord2_y -'0'-1;

                    count=0;

                    if(!(x1<0 || x1>=8 || y1<0 || y1>=8 || x2<0 || x2>8 || y2<0 || y2>8)){
                        std::cout<<"\n****************\nMOVING TO "<<x1<<","<<y1<<"\n";

                        if(ANGULAR_HANDLING ==1 &&  y1==0 && x1>=2 && x1<=5){
                            ROS_INFO("PROBLEMATIC POSE");
                            ROS_INFO("HOMING");

                            control.eraseAllTrajectories();
                            control.setCartesianPosition(home_pose, 0, true);
                            count=0;
                            control.getCartesianPosition(temp_pose);
                            ROS_INFO("EXPECTING ARM TO HOME");
                            while(!arrived(home_pose, temp_pose, 0.01)){
                                sleep(1);
                                if(count>10){
                                    return 1;
                                    break;
                                }else{
                                    count+=1;
                                    control.getCartesianPosition(temp_pose);
                                }
                            }
                            ROS_INFO("ARM HAS BEEN HOMED");
                            sleep(1);

                            ROS_INFO("SWITCHING TO ANGULAR CONTROL");
                            ros::spinOnce();
                            control.eraseAllTrajectories();
                            control.setAngularControl();
                            ros::spinOnce();

                            kinova::KinovaAngles temp_angles;

                            switch(x1){
                                case 2:
                                    temp_angles = c1_angles;
                                    break;
                                case 3:
                                    temp_angles = d1_angles;
                                    break;
                                case 4:
                                    temp_angles = e1_angles;
                                    break;
                                case 5:
                                    temp_angles = f1_angles;
                                    break;
                            }

                            control.setJointAngles(temp_angles,20,20,0,true);
                            ros::spinOnce();
                            control.getJointAngles(curr_angles);
                            while(!arrived_angled(curr_angles, temp_angles, 5)){
                                sleep(1);
                                if(count>10){
                                    ros::spinOnce();
                                    throw -1;
                                    break;
                                }else{
                                    ros::spinOnce();
                                    count+=1;
                                    control.getJointAngles(curr_angles);
                                }
                            }
                            
                            sleep(1);
                            ros::spinOnce();

                            if(x1!=2){
                                sleep(2);
                                control.setJointAngles(safe_angles, 20,20, 0,true);
                                control.getJointAngles(curr_angles);
                                while(!arrived_angled(curr_angles, safe_angles, 5)){
                                    sleep(1);
                                    if(count>10){
                                        ros::spinOnce();
                                        throw -1;
                                        break;
                                    }else{
                                        ros::spinOnce();
                                        count+=1;
                                        control.getJointAngles(curr_angles);
                                    }
                                }
                            }

                            ROS_INFO("RESTORING CARTESIAN CONTROL");
                            control.eraseAllTrajectories();
                            control.setCartesianControl();

                            sleep(2);
                            control.setCartesianPosition(home_pose,0 ,true);

                            count=0;
                            control.getCartesianPosition(temp_pose);
                            ROS_INFO("EXPECTING ARM TO HOME");
                            while(!arrived(home_pose, temp_pose, 0.01)){
                                sleep(1);
                                if(count>10){
                                    ros::spinOnce();
                                    throw -11;
                                    break;
                                }else{
                                    ros::spinOnce();
                                    count+=1;
                                    control.getCartesianPosition(temp_pose);
                                }
                            }
                            ROS_INFO("ARM HAS BEEN HOMED");

                        }else{

                            kinova::KinovaPose l2_pose(board_pos_l2[y1][x1]);
                            kinova::KinovaPose l1_pose(board_pos_l1[y1][x1]);
                        

                            control.setCartesianPosition(l2_pose,0,true);
                            control.getCartesianPosition(temp_pose);
                            ROS_INFO("WAITING FOR ARM TO ARRIVE AT %d,%d",x1,y1);
                            while(!arrived(l2_pose, temp_pose, 0.01)){
                                sleep(1);
                                if(count>10){
                                    ros::spinOnce();
                                    throw -1;
                                    break;
                                }else{
                                    ros::spinOnce();
                                    count+=1;
                                    control.getCartesianPosition(temp_pose);
                                }
                            }

                            ros::spinOnce();

                            sleep(0.5);
                            count=0;
                            control.setCartesianPosition(l1_pose,0,true);
                            control.getCartesianPosition(temp_pose);
                            ROS_INFO("WAITING FOR ARM TO LOWER ITSELF");
                            while(!arrived(l1_pose, temp_pose, 0.01)){
                                sleep(1);
                                if(count>10){
                                    ros::spinOnce();
                                    throw -1;
                                    break;
                                }else{
                                    ros::spinOnce();
                                    count+=1;
                                    control.getCartesianPosition(temp_pose);
                                }
                            }
                            close_gripper();
                            sleep(2);
                            ros::spinOnce();
                            
                            count=0;
                            control.setCartesianPosition(l2_pose,0 ,true);
                            control.getCartesianPosition(temp_pose);
                            ROS_INFO("WAITING FOR ARM TO RETURN TO %d,%d",x2,y2);
                            while(!arrived(l2_pose, temp_pose, 0.01)){
                                sleep(1);
                                if(count>10){
                                    ros::spinOnce();
                                    throw -1;
                                    break;
                                }else{
                                    ros::spinOnce();
                                    count+=1;
                                    control.getCartesianPosition(temp_pose);
                                }
                            }
                            sleep(0.5);
                            
                            kinova::KinovaPose l2_pose2;
                            kinova::KinovaPose l1_pose2;
                            if(x2==8 && y2==8){
                                ROS_INFO("A PIECE HAS BEEN CAPTURED");
                                kinova::KinovaPose temp_l2_pose2(out_pose_msg);
                                kinova::KinovaPose temp_l1_pose2(out_pose_msg);

                                l2_pose2 = temp_l2_pose2;
                                l1_pose2 = temp_l1_pose2;
                            }else{
                                kinova::KinovaPose temp_l2_pose2(board_pos_l2[y2][x2]);
                                kinova::KinovaPose temp_l1_pose2(board_pos_l1[y2][x2]);
                            
                                l2_pose2 = temp_l2_pose2;
                                l1_pose2 = temp_l1_pose2;
                            }

                            control.setCartesianPosition(l2_pose2,0,true);
                            control.getCartesianPosition(temp_pose);
                            ROS_INFO("WAITING FOR ARM TO ARRIVE AT %d,%d",x2,y2);
                            while(!arrived(l2_pose2, temp_pose, 0.01)){
                                sleep(1);
                                if(count>10){
                                    ros::spinOnce();
                                    throw -1;
                                    break;
                                }else{
                                    ros::spinOnce();
                                    count+=1;
                                    control.getCartesianPosition(temp_pose);
                                }
                            }
                            count=0;
                            control.setCartesianPosition(l1_pose2,0,true);
                            control.getCartesianPosition(temp_pose);
                            ROS_INFO("WAITING FOR ARM TO LOWER ITSELF");
                            while(!arrived(l1_pose2, temp_pose, 0.01)){
                                sleep(1);
                                if(count>10){
                                    ros::spinOnce();
                                    throw -1;
                                    break;
                                }else{
                                    ros::spinOnce();
                                    count+=1;
                                    control.getCartesianPosition(temp_pose);
                                }
                            }
                            open_gripper();
                            sleep(2);
                            ros::spinOnce();

                            count=0;
                            control.setCartesianPosition(l2_pose2,0 ,true);
                            control.getCartesianPosition(temp_pose);
                            ROS_INFO("WAITING FOR ARM TO RETURN TO %d,%d",x2,y2);
                            while(!arrived(l2_pose2, temp_pose, 0.01)){
                                sleep(1);
                                if(count>10){
                                    ros::spinOnce();
                                    throw -1;
                                    break;
                                }else{
                                    ros::spinOnce();
                                    count+=1;
                                    control.getCartesianPosition(temp_pose);
                                }
                            }
                            ROS_INFO("MOVE COMPLETED");
                            ros::spinOnce();
                        }
                    }else{
                        ROS_INFO("COORDS GIVEN DO NOT BELONG IN THE BOARD (%d,%d -> %d,%d)",x1,y1,x2,y2);
                    }

                }
                active=0;
                set_status_ready();
            }    

        }catch(...){
            ROS_INFO("MOVE TIMEOUT! ABORTING...");
            count=0;
            control.eraseAllTrajectories();
            control.stopAPI();
            control.startAPI();
            control.setCartesianControl();
            control.setCartesianPosition(home_pose,0 ,true);

            count=0;
            ros::spinOnce();
            control.getCartesianPosition(temp_pose);
            ROS_INFO("EXPECTING ARM TO HOME");
            while(!arrived(home_pose, temp_pose, 0.01)){
                sleep(1);
                if(count>10){
                    ros::spinOnce();
                    return 1;
                    break;
                }else{
                    ros::spinOnce();
                    count+=1;
                    control.getCartesianPosition(temp_pose);
                }
            }
            ROS_INFO("ARM HAS BEEN HOMED");
            active=0;
            set_status_ready();
        }

    }
    control.eraseAllTrajectories();
    return 0;
}


void moveCallback(const std_msgs::String::ConstPtr& msg)
{
        /* discards repetitive commands*/
    if(move != msg->data){
        move = msg->data;
        active =1;
    }

}


int main(int argc, char **argv)
{
    // while(1){
    ros::init(argc, argv, "kinova_arm_driver");
    ros::NodeHandle nh("~");
    boost::recursive_mutex api_mutex;

    std::string kinova_robotType = argv[argc-1];
    std::string kinova_robotName = argv[argc-1];
    ROS_INFO("INIT DRIVER");


    kinova::KinovaComm control(nh, api_mutex, true, kinova_robotType);
    kinova::KinovaArm kinova_arm(control, nh, kinova_robotType, kinova_robotName);
    kinova::JointTrajectoryController joint_trajectory_controller(control, nh);

    ros::Subscriber sub = nh.subscribe("chess_moves", 1000, moveCallback);
    status_pub = nh.advertise<std_msgs::String>("arm_movement_status",1000); 
    servo_pub = nh.advertise<std_msgs::UInt16>("servo",1);

    servo_value.data = 0;
    servo_pub.publish(servo_value);
    ros::spinOnce();

    init_board();
    move="";
    active =0;
    control.eraseAllTrajectories();
        
    
    if(!position_control(control)){
            return 0;
    }else{
        ROS_INFO("WOAH!");
        sleep(2);
        ROS_INFO("RESTARTING...");
        sleep(1);
    }
    
    return 0;
}
