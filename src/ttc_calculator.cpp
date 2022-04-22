/**
 * @file ttc_calculator.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros_collision_detection/ttc_calculator.h"


TTCCalculator::TTCCalculator()
{
    ROS_INFO("In TTC Calculator constructor.");
    ttc_algorithm = new CircleAlgorithm(); //TODO
}

TTCCalculator::~TTCCalculator()
{

}

void TTCCalculator::calculateAllTTCs(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects_msg, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg)
{
    ROS_INFO("In TTCCalculator::calculateAllTTCs:");
    ROS_INFO("Seq: %d: x_pos: %f", perceived_objects_msg->header.seq, perceived_objects_msg->perceived_objects[0].object_movement.position.x);
    ROS_INFO("Seq: %d", subject_vehicle_motion_msg->header.seq);
    // TODO: calculate TTC with Algorithm for all contained PerceivedObjects
}

