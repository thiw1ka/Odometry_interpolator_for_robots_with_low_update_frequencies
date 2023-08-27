#include <condition_variable>	
#include <deque>
#include <mutex>
#include <iostream>
#include <ros/ros.h>
#include <shared_mutex>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <thread>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include "yolov5_ros/Detection2DArray.h"

// https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html

#ifndef LOW_FREQ_TOPIC_MSG_IP_HPP
#define LOW_FREQ_TOPIC_MSG_IP_HPP

struct MsgContainer {

    MsgContainer() {};

    MsgContainer(std::deque <nav_msgs::Odometry>* o,
                std::deque <sensor_msgs::PointCloud2>* l,
                std::deque <yolov5_ros::Detection2DArray>* c
                ) {
            
            odom_message_container_ptr_.reset(o);
            lidar_message_container_ptr_.reset(l);
            camera_message_container_ptr_.reset(c);
    };

    std::unique_ptr <std::deque <nav_msgs::Odometry> >odom_message_container_ptr_;

    std::unique_ptr <std::deque <sensor_msgs::PointCloud2> > lidar_message_container_ptr_;

    std::unique_ptr <std::deque <yolov5_ros::Detection2DArray> > camera_message_container_ptr_;

    ~ MsgContainer () {
        std::printf("deleting the msg container \n");
    };
};

struct LowFreqTopicMsgIntepolator {

    LowFreqTopicMsgIntepolator (ros::NodeHandle n);

    ros::NodeHandle n_;
    
    ros::Subscriber subOdom_, subObstacle_, subCamera_;

    ros::Publisher pubOdom_, pubObstacle_, pubCamera_;

    void cbOdom (const nav_msgs::Odometry::ConstPtr& msg);

    void CentroidLabelObstacleCB (const sensor_msgs::PointCloud2::ConstPtr& msg);

    void CbForCameraMeasurements (const yolov5_ros::Detection2DArray::ConstPtr& msg);

    template <typename t>
    void callback_function  (const t& msg){

    };

    std::deque <nav_msgs::Odometry> odom_message_container_;

    std::deque <sensor_msgs::PointCloud2> lidar_message_container_;

    std::deque <yolov5_ros::Detection2DArray> camera_message_container_;

    geometry_msgs::Pose calculatePartialUpdates ( const nav_msgs::Odometry& a,
                                                const nav_msgs::Odometry& b, 
                                                const double& proposed_sample_size ); 

    std::deque <nav_msgs::Odometry>* odom_message_container_ptr_;

    std::deque <sensor_msgs::PointCloud2>* lidar_message_container_ptr_;

    std::deque <yolov5_ros::Detection2DArray>* camera_message_container_ptr_;

    std::recursive_mutex odomLock, lidarlock, camLock, mainContainerLock;

    void getCopyOfMsgs (void);

    // std::thread 
    static bool isThreadStillPublishing;

    std::condition_variable cvIsPublishing;

    std::deque <MsgContainer*> msg_tuples_to_publish;

    void publisherLoop (const ros::TimerEvent& e);

    void pushToMainContainer(MsgContainer* mcptr);

    ros::Timer timer_for_publisher;

};


#endif