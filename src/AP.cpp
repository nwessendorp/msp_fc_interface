#include "AP.hpp"


/* main program constructor 
void AP::do_nothing() {
    ros::Rate rate_1(50);
    while(1) {
        rate_1.sleep();
    }
}

AP::AP() {
    AP_thread = std::thread(&AP::do_nothing, this);
}



<!-- radar driver -->
<node name="radar_ros_driver" pkg="radar_ros_driver" type="radar_ros_driver" output="screen" />
<!-- radar server -->
<node name="radar_server" pkg="radar_server" type="radar_server"></node>
*/

void handle_command(radar_avoid_msgs::Command command_msg) {
    bool test_ros = command_msg.test_ros;
    if (test_ros) {
        ROS_INFO("true");
    } else {
        ROS_INFO("false");
    }
}

Controller *controller;
msp_node *msp;
int main(int argc, char** argv) {
    ros::init(argc, argv, "msp_fc_interface");
    ros::NodeHandle n;
    ros::Rate rate(50);//50 Hz
    
    msp = new msp_node();// MSP comminication handled in this thread
    controller = new Controller();// controller in seperate thread
    
    // All ROS communication handled here:
    //ros::Subscriber sub_arm   = n.subscribe("/uav/control/arm", 1, &MspInterface::set_armed, &iface);
    //ros::Subscriber sub_rates = n.subscribe("/uav/control/rate_thrust", 1, &MspInterface::set_rates, &iface);
    ros::Subscriber sub_radar_command  = n.subscribe("radar_commands", 1, handle_command);

    int i = 0;
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}