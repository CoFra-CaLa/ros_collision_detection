/**
 * @file warning_generator.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Implementation of the methods of Warning Generator class.
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros_collision_detection/warning_generator.h"


WarningGenerator::WarningGenerator(ros::Publisher &publisher)
:warning_generator_algorithm(nullptr),
collision_warning_publisher(publisher)
{
    ROS_DEBUG("WarningGenerator::WarningGenerator constructor.");
}

void WarningGenerator::setWarningGeneratorAlgorithm(boost::shared_ptr<WarningGeneratorAlgorithm> &algorithm)
{
    warning_generator_algorithm.swap(algorithm);
    ROS_DEBUG("WarningGenerator::setWarningGeneratorAlgorithm: set new algorithm.");
}

void WarningGenerator::setCollisionWarningPublisher(ros::Publisher &publisher)
{
    collision_warning_publisher = publisher;
    ROS_DEBUG("TTCCalculator::setCollisionWarningPublisher: set new publisher.");
}

void WarningGenerator::createWarning(const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg, const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg, double ttc)
{
    ros_collision_detection::CollisionCheckResult collision_check_msg; //!< the message to be published
    collision_check_msg.header.stamp = ros::Time::now();
    collision_check_msg.perceived_object = *perceived_object_motion_msg;
    collision_check_msg.ttc = (float) ttc;  // convert double to float for publishing
    collision_check_msg.result_type = warning_generator_algorithm->generateWarning(subject_vehicle_motion_msg, perceived_object_motion_msg, ttc);

    ROS_INFO("WarningGenerator: perceived object with ID %d has TTC %f with result type %d.", perceived_object_motion_msg->object_movement.id, collision_check_msg.ttc, collision_check_msg.result_type);
    collision_warning_publisher.publish(collision_check_msg);
}