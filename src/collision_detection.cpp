/**
 * @file collision_detection.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Implementation of the main function of the node and implementation of methods of Collision Detection class.
 * @version 0.1
 * @date 2022-04-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include <ros_collision_detection/collision_detection.h>


CollisionDetection::CollisionDetection(ros::NodeHandle *nh)
:approximate_synchronizer(ApproximateSyncPolicy(10), fused_objects_subscriber, ego_position_subscriber),
warning_generator(collision_warning_publisher)
{
    node_handle = nh;

    // initialize ttc_calculator and warning_generator
    //ttc_calculator.setTTCAlgorithm(new CircleAlgorithm());                            // TODO: make choose of concrete algorithm configurable
    int n_circles = 2;
    ttc_calculator.setTTCAlgorithm(new NCircleAlgorithm(n_circles));                    // TODO: make choose of concrete algorithm configurable
    warning_generator.setWarningGeneratorAlgorithm(new TTCOnlyWarningAlgorithm());      // TODO: make choose of concrete algorithm configurable

    // register callback from ttc_calculator to warning_generator
    ttc_calculator.addWarningSignalCallback(boost::bind(&WarningGenerator::createWarning, &warning_generator, _1, _2, _3)); 

    CollisionDetection::init();
}

void CollisionDetection::init()
{
    collision_warning_publisher = node_handle->advertise<ros_collision_detection::CollisionCheckResult>("/collision_warning", 10);
    fused_objects_subscriber.subscribe(*node_handle, "/fused_objects", 100);
    ego_position_subscriber.subscribe(*node_handle, "/ego_position", 100);
    approximate_synchronizer.registerCallback(boost::bind(&CollisionDetection::callback, this, _1, _2));
    
    // log successful init
    ROS_INFO("collision_detection node successfully initialized.");
}

void CollisionDetection::callback(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects_msg, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg)
{
    // log seq number of the two messages
    ROS_DEBUG("TTCCalculator::calculateAllTTCs: Subject vehicle msg: seq = %d | perceived object msg: seq = %d.", subject_vehicle_motion_msg->header.seq, perceived_objects_msg->header.seq);
    
    ttc_calculator.calculateAllTTCs(perceived_objects_msg, subject_vehicle_motion_msg);
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "collision_detecion");
    ros::NodeHandle nh;
    CollisionDetection collision_detection(&nh);

    ros::spin();
}
