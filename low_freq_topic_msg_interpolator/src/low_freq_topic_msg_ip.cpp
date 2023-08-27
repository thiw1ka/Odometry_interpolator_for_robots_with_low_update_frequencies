#include "low_freq_topic_msg_interpolator/low_freq_topic_msg_ip.hpp"

LowFreqTopicMsgIntepolator::LowFreqTopicMsgIntepolator (ros::NodeHandle n) : n_(n) {
    std::string odomTopicName = "/mavros/local_position/odom";
    std::string obstacleTopicName = "/centroid_label_obstacle";
    std::string cameraTopicName = "/yolov5/bounding_box";

    odom_message_container_.clear();
    lidar_message_container_.clear();
    camera_message_container_.clear();

    odom_message_container_ptr_ = new std::deque <nav_msgs::Odometry>;
    lidar_message_container_ptr_ = new std::deque <sensor_msgs::PointCloud2> ;
    camera_message_container_ptr_ = new std::deque <yolov5_ros::Detection2DArray>;

    subOdom_ = n_.subscribe (odomTopicName, 10, &LowFreqTopicMsgIntepolator::cbOdom, this);
    subObstacle_ = n_.subscribe (obstacleTopicName, 10, &LowFreqTopicMsgIntepolator::CentroidLabelObstacleCB, this);
    subCamera_ = n_.subscribe (cameraTopicName, 10, &LowFreqTopicMsgIntepolator::CbForCameraMeasurements, this);

    pubOdom_ = n_.advertise <nav_msgs::Odometry> (odomTopicName + "_republished", 10, false);
    pubObstacle_ = n_.advertise <sensor_msgs::PointCloud2> (obstacleTopicName + "_republished", 10, false);
    pubCamera_ = n_.advertise <yolov5_ros::Detection2DArray> (cameraTopicName+ "_republished", 10, false);

    timer_for_publisher = n_.createTimer(ros::Duration(0.2),&LowFreqTopicMsgIntepolator::publisherLoop,this,true);
}

bool isThreadStillPublishing = false;

void LowFreqTopicMsgIntepolator::cbOdom (const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard <std::recursive_mutex> lock(odomLock);
    odom_message_container_ptr_-> push_back(*msg);
    if(odom_message_container_ptr_->size() >= 2) getCopyOfMsgs ();

}

void LowFreqTopicMsgIntepolator::CentroidLabelObstacleCB (const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::lock_guard <std::recursive_mutex> lock (lidarlock);
    lidar_message_container_ptr_ -> push_back(*msg);
}

void LowFreqTopicMsgIntepolator::CbForCameraMeasurements (const yolov5_ros::Detection2DArray::ConstPtr& msg) {
    std::lock_guard <std::recursive_mutex> lock(camLock);
    camera_message_container_ptr_ -> push_back(*msg);
}

geometry_msgs::Pose LowFreqTopicMsgIntepolator::calculatePartialUpdates(const nav_msgs::Odometry& a, const nav_msgs::Odometry& b,const double& proposed_sample_size) {
    tf2::Quaternion start_orientation, end_orientation;
    tf2::Vector3 start_point, end_point;
    tf2::fromMsg (a.pose.pose.orientation, start_orientation);
    tf2::fromMsg (b.pose.pose.orientation, end_orientation);
    tf2::fromMsg (a.pose.pose.position, start_point);
    tf2::fromMsg (b.pose.pose.position, end_point);
    auto incremental_orientation = start_orientation.slerp(end_orientation, proposed_sample_size);
    auto incremental_vector = tf2::lerp(start_point, end_point, proposed_sample_size);
    geometry_msgs::Pose incremental_pose;
    incremental_pose.position = tf2::toMsg (incremental_vector, incremental_pose.position);
    incremental_pose.orientation = tf2::toMsg (incremental_orientation);
    return incremental_pose;
}

void LowFreqTopicMsgIntepolator::getCopyOfMsgs (void) {
    MsgContainer* copy_of_all_msgs;
    { // scope lock to work only till this bracket ends
        std::scoped_lock lk(lidarlock, camLock);
        auto lastodom = odom_message_container_ptr_ ->back();
        copy_of_all_msgs = new MsgContainer (odom_message_container_ptr_, lidar_message_container_ptr_, camera_message_container_ptr_);
        //assigning new containers
        odom_message_container_ptr_ = new std::deque <nav_msgs::Odometry>;
        odom_message_container_ptr_ ->push_back(lastodom);
        lidar_message_container_ptr_ = new std::deque <sensor_msgs::PointCloud2>;
        camera_message_container_ptr_ = new std::deque <yolov5_ros::Detection2DArray>;
    }
    std::thread t(&LowFreqTopicMsgIntepolator::pushToMainContainer, this, copy_of_all_msgs);
    t.detach(); // not waiting for thread to return so this will not get blocked.
}

void LowFreqTopicMsgIntepolator::pushToMainContainer(MsgContainer* mcptr) {
    //will try to insert msg into main loop. 
    //running inside a thread will reduce the amount of time that CB were being locked
    auto copyOfMsg = mcptr;
    std::lock_guard <std::recursive_mutex> lk(mainContainerLock);
    msg_tuples_to_publish.push_back(copyOfMsg);
    std::printf("A message pushed to the main thread \n");
}

void LowFreqTopicMsgIntepolator::publisherLoop (const ros::TimerEvent& e) {
    ros::Rate r(20);
    enum STATES {
        INIT = 0,
        PUBLISH = 1
    };
    auto CURRENT_STATE = STATES::INIT;
    MsgContainer* currentMsg; 
    try {
        while (ros::ok()) {
            if (CURRENT_STATE == STATES::INIT) {
                std::printf("checking for a new message to publish \n");
                std::lock_guard <std::recursive_mutex> lk (mainContainerLock);
                // mainContainerLock.lock();
                if (!msg_tuples_to_publish.empty()) {
                    std::printf("got a message \n");
                    currentMsg = msg_tuples_to_publish.front(); //get the copy of the first msg
                    msg_tuples_to_publish.pop_front();
                    std::cout << "size :"<< msg_tuples_to_publish.size() << std::endl;
                    CURRENT_STATE = STATES::PUBLISH;
                }
                // mainContainerLock.unlock();
            }
            else if (CURRENT_STATE == STATES::PUBLISH) {
                
                auto lidar_msg_count = currentMsg->lidar_message_container_ptr_->size();
                auto camera_msg_count = currentMsg->camera_message_container_ptr_->size(); 

                auto sample_size =  lidar_msg_count > camera_msg_count ? lidar_msg_count : camera_msg_count;
                std::printf ("lidar sz : %d, camera sz : %d, sample_size: %d \n",lidar_msg_count, camera_msg_count, sample_size);
                auto sample_sz = 1.0 / sample_size;
                auto sampleCounter = sample_sz;
                std::printf ("sample counter : %f, sample size : %f \n",sampleCounter, sample_sz );
                auto incre_pose = calculatePartialUpdates( currentMsg->odom_message_container_ptr_->at(0), 
                                                                                currentMsg->odom_message_container_ptr_->at(1),
                                                                                sample_sz);
                tf2::Quaternion increment_quat;
                tf2::Quaternion previous_q;
                tf2::fromMsg(incre_pose.orientation, increment_quat);
                ros::Rate rr(30);
                auto msg_to_pub = currentMsg->odom_message_container_ptr_->at(0);
                for (auto i = 0; i < sample_size; i++) {
                    std::printf ("sample counter : %f \n",sampleCounter );
                    msg_to_pub.pose.pose = calculatePartialUpdates( currentMsg->odom_message_container_ptr_->at(0), 
                                                                                currentMsg->odom_message_container_ptr_->at(1),
                                                                                sampleCounter);
                    pubOdom_.publish(msg_to_pub);
                    if (!currentMsg->lidar_message_container_ptr_->empty()) {
                        pubObstacle_.publish(currentMsg->lidar_message_container_ptr_->front());
                        currentMsg->lidar_message_container_ptr_ -> pop_front();
                    }
                    if (!currentMsg->camera_message_container_ptr_->empty()) {
                        pubCamera_.publish (currentMsg->camera_message_container_ptr_->front());     
                        currentMsg->camera_message_container_ptr_-> pop_front(); 
                    }
                    sampleCounter += sample_sz;
                    rr.sleep();
                }
                delete currentMsg;
                CURRENT_STATE = STATES::INIT;
            }
            r.sleep();
        }
    }
    catch (std::exception& e) {
        std::cout << "[lFTMI] Error inside republisher : " << e.what() << std::endl;
    }
}
