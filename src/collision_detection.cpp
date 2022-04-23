/**
 * @file collision_detection.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros_collision_detection/PerceivedObjects.h"
#include "ros_collision_detection/SubjectVehicleMotion.h"
#include <ros_collision_detection/collision_detection.h>

CollisionDetection::CollisionDetection(ros::NodeHandle *nh)
:approximate_synchronizer(ApproximateSyncPolicy(10), fused_objects_subscriber, ego_position_subscriber)
{
    node_handle = nh;
    ttc_calculator.setTTCAlgorithm(new CircleAlgorithm()); // TODO: make choose of concrete algorithm configurable
    CollisionDetection::init();
}

void CollisionDetection::init()
{
    fused_objects_subscriber.subscribe(*node_handle, "/fused_objects", 100);
    ego_position_subscriber.subscribe(*node_handle, "/ego_position", 100);
    approximate_synchronizer.registerCallback(boost::bind(&CollisionDetection::callback, this, _1, _2));
}


CollisionDetection::~CollisionDetection()
{
    
}

void CollisionDetection::callback(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects_msg, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg)
{
    ttc_calculator.calculateAllTTCs(perceived_objects_msg, subject_vehicle_motion_msg);
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "collision_detecion");
    ros::NodeHandle nh;
    CollisionDetection collision_detection(&nh);

    ros::spin();
}