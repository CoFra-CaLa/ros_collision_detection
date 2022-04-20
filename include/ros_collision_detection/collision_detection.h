/**
 * @file collision_detection.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _COLLISION_DETECTION_H_
#define _COLLISION_DETECTION_H_


#include <ros/ros.h>
#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>  
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "ros_collision_detection/PerceivedObjects.h"
#include "ros_collision_detection/SubjectVehicleMotion.h"


typedef message_filters::sync_policies::ApproximateTime<ros_collision_detection::PerceivedObjects, ros_collision_detection::SubjectVehicleMotion> ApproximateSyncPolicy;


class CollisionDetection
{
private:
    ros::NodeHandle *node_handle;
    message_filters::Subscriber<ros_collision_detection::PerceivedObjects> fused_objects_subscriber;
    message_filters::Subscriber<ros_collision_detection::SubjectVehicleMotion> ego_position_subscriber;
    //message_filters::TimeSynchronizer<ros_collision_detection::PerceivedObjects, ros_collision_detection::SubjectVehicleMotion> time_snychronizer;
    message_filters::Synchronizer<ApproximateSyncPolicy> approximate_synchronizer;
    ros::Publisher collision_warning_publisher;

public:
    CollisionDetection(ros::NodeHandle *nh);
    ~CollisionDetection();
    void callbackFusedObjects(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects);
    void callbackEgoPosition(const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion);
    void callback(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects_msg, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg);
    void init();

};

#endif  // _COLLISION_DETECTION_H_