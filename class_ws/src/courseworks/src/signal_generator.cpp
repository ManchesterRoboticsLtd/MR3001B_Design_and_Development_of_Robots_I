#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "signal_generator");
    ros::NodeHandle nh;
    // Publishers for signal and time
    ros::Publisher pub_signal = nh.advertise<std_msgs::Float32>("/signal", 100);
    ros::Publisher pub_time = nh.advertise<std_msgs::Float32>("/time", 100);

    ros::Rate loop_rate(10);

    float time = 0.0;

    while(ros::ok()){
        std_msgs::Float32 msg_signal;
        std_msgs::Float32 msg_time;
        msg_signal.data = sin(time);
        msg_time.data = time;
        pub_signal.publish(msg_signal);
        pub_time.publish(msg_time);

        ROS_INFO("Signal: %f, Time: %f", msg_signal.data, msg_time.data);

        time += 0.1;
        loop_rate.sleep();
        ros::spinOnce();
    }
}