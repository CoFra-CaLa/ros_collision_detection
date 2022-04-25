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
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ros_collision_detection/PerceivedObjects.h"
#include "ros_collision_detection/SubjectVehicleMotion.h"
#include "ros_collision_detection/CollisionCheckResult.h"

#include "ros_collision_detection/ttc_calculator.h"

#include "ros_collision_detection/circle_algorithm.h"


typedef message_filters::sync_policies::ApproximateTime<ros_collision_detection::PerceivedObjects, ros_collision_detection::SubjectVehicleMotion> ApproximateSyncPolicy;


class CollisionDetection
{
private:
    ros::NodeHandle *node_handle;
    ros::Publisher collision_check_result_publisher;
    message_filters::Subscriber<ros_collision_detection::PerceivedObjects> fused_objects_subscriber;
    message_filters::Subscriber<ros_collision_detection::SubjectVehicleMotion> ego_position_subscriber;
    message_filters::Synchronizer<ApproximateSyncPolicy> approximate_synchronizer;
    ros::Publisher collision_warning_publisher;
    TTCCalculator ttc_calculator;

public:
    CollisionDetection(ros::NodeHandle *nh);
    ~CollisionDetection();
    void callback(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects_msg, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg);
    void init();

};

#endif  // _COLLISION_DETECTION_H_