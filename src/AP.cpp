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

void dvs_cmd_callback(const std_msgs::Int32::ConstPtr& msg) {
    controller->avoid = msg->data;
    ROS_INFO("Subscribed DVS command: [%d]", msg->data);
}

void radar_cmd_callback(const radar_avoid_msgs::Command::ConstPtr& command_msg) {
    controller->avoid = command_msg->avoid_state;
    controller->v_xa_des = (double)command_msg->dv[0];
    controller->v_ya_des = (double)command_msg->dv[0];
    //ROS_INFO("Subscribed Radar command: [%d]", command_msg->avoid_state);
}

void publish_state_to_radar(ros::Publisher pub_st) {
    msp_fc_interface::RcData rc_msg;
    rc_msg.state[0] = controller->vel_x_est_velFrame;
    rc_msg.state[1] = controller->vel_y_est_velFrame;;
    pub_st.publish(rc_msg);
}

Controller *controller;
msp_node *msp;
NatNet *gps;
int main(int argc, char** argv) {
    ros::init(argc, argv, "msp_fc_interface");
    ros::NodeHandle n;
    ros::Rate rate(50);//50 Hz
    ros::Publisher pub_st;

    controller = new Controller();// controller in seperate thread
    //gps = new NatNet(); Optitrack thread
    #ifdef USE_NATNET
    gps = new NatNet();// Optitrack thread
    #endif
    msp = new msp_node();// MSP comminication handled in this thread
    
    // All ROS communication handled here:
    pub_st = n.advertise<msp_fc_interface::RcData>("/MAV_state", 1, true);
    ros::Subscriber sub_dvs_cmd  = n.subscribe("/roll_command", 1, dvs_cmd_callback);
    ros::Subscriber sub_radar_cmd  = n.subscribe("radar_commands", 1, radar_cmd_callback);

    int i = 0;
    while (ros::ok()) {
        publish_state_to_radar(pub_st);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
