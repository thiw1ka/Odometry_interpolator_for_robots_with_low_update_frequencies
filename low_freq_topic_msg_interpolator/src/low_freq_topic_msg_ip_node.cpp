#include "low_freq_topic_msg_interpolator/low_freq_topic_msg_ip.hpp"

int main (int argc, char* argv[]) {

    ros::init(argc, argv, "Odom_msg_interpolator");
    ros::NodeHandle nh;
    std::cout<<"low_freq_topic_msg_node_started.."<<std::endl;
    ros::AsyncSpinner spinner(0); // Use all threads
    spinner.start();
    std::unique_ptr <LowFreqTopicMsgIntepolator> odom_interpolator(new LowFreqTopicMsgIntepolator(nh));
    ros::waitForShutdown();

    return 0;
};