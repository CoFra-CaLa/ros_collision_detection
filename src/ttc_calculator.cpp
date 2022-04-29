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


//TODO: where to create the concrete TTC Algorithm instance?
TTCCalculator::TTCCalculator()
:ttc_algorithm(nullptr)
{
    ROS_INFO("In TTC Calculator constructor.");
}

TTCCalculator::~TTCCalculator()
{

}

void TTCCalculator::setTTCAlgorithm(TTCAlgorithm *algorithm)
{
    ROS_INFO("TTCCalculator::setTTCAlgorithm: reset algorithm.");
    ttc_algorithm.reset(algorithm);
}

object_motion_t TTCCalculator::createObjectMotionFromSubjectVehicleMotion(const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg)
{
    object_motion_t result;
    
    result.center_pos_x = subject_vehicle_motion_msg->vehicle_movement.position.x;
    result.center_pos_y = subject_vehicle_motion_msg->vehicle_movement.position.y;
    result.length_x = 5; // TODO: get length_xi, length_yi
    result.length_y = 2; // TODO: get length_xi, length_yi
    result.heading = subject_vehicle_motion_msg->vehicle_movement.heading;
    result.speed = subject_vehicle_motion_msg->vehicle_movement.speed;
    result.acceleration = subject_vehicle_motion_msg->vehicle_movement.acceleration;

    return result;
}

object_motion_t TTCCalculator::createObjectMotionFromPerceivedObjectMotion(const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg)
{
    object_motion_t result;

    // PerceivedObjectMotion's position.x and position.y refer to the bumper's center coordinate
    // transfrom bumper's center coordinate (bumper_pos_x, bumper_pos_y) to perceived object's center coordinate (center_pos_x, center_pos_y)
    float bumper_pos_x = perceived_object_motion_msg->object_movement.position.x;
    float bumper_pos_y = perceived_object_motion_msg->object_movement.position.y;
    float heading = perceived_object_motion_msg->object_movement.heading;
    float x_length = perceived_object_motion_msg->x_length;
    result.center_pos_x = bumper_pos_x - 0.5 * sin(heading * M_PI / 180.0) * x_length; // TODO: check for float/double converting issues
    result.center_pos_y = bumper_pos_y - 0.5 * cos(heading * M_PI / 180.0) * x_length; // TODO: check for float/double converting issues
    result.length_x = x_length;
    result.length_y = perceived_object_motion_msg->y_length;
    result.heading = heading;
    result.speed = perceived_object_motion_msg->object_movement.speed;
    result.acceleration = perceived_object_motion_msg->object_movement.acceleration;

    return result;
}

void TTCCalculator::handleTTCResult(boost::optional<double> &ttc_optional, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg, const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg)
{
    if(ttc_optional)
    {
        // valid TTC result --> pass TTC, subject vehicle and perceived object to the Warning Generator
        ROS_INFO("TTCCalculator::handleTTCResult: computed valid TTC %f for perceived object %d", *ttc_optional, perceived_object_motion_msg->object_movement.id);
    }
    else
    {
        // no valid TTC result --> only log
        ROS_INFO("TTCCalculator::handleTTCResult: no valid TTC could be computed for perceived object %d", perceived_object_motion_msg->object_movement.id);
    }
}

void TTCCalculator::calculateAllTTCs(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects_msg, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg)
{
    ROS_INFO("In TTCCalculator::calculateAllTTCs:");
    ROS_INFO("Seq: %d: x_pos: %f", perceived_objects_msg->header.seq, perceived_objects_msg->perceived_objects[0].object_movement.position.x);
    ROS_INFO("Seq: %d", subject_vehicle_motion_msg->header.seq);

    // TODO: calculate TTC with Algorithm for all contained PerceivedObjects
    // store the subject vehicle's motion data (vehicle i) for use with all perceived objects
    object_motion_t subject_vehicle_motion = createObjectMotionFromSubjectVehicleMotion(subject_vehicle_motion_msg);

    int perceived_objects_count = perceived_objects_msg->perceived_objects.size();
    for(int i = 0; i < perceived_objects_count; i++)
    {
        ros_collision_detection::PerceivedObjectMotion perceived_object = perceived_objects_msg->perceived_objects[i];
        boost::shared_ptr<ros_collision_detection::PerceivedObjectMotion> perceived_object_msg = boost::make_shared<ros_collision_detection::PerceivedObjectMotion>(perceived_object);
        object_motion_t perceived_object_motion = createObjectMotionFromPerceivedObjectMotion(perceived_object_msg);
        uint32_t perceived_object_id = perceived_object.object_movement.id;
        std::string perceived_object_type = perceived_object.object_type;

        boost::optional<double> ttc_optional;   //!< contains either valid TTC or not
        ttc_optional = ttc_algorithm->calculateTTC(subject_vehicle_motion, perceived_object_motion);
        // TODO: trigger the TTC warning output to publisher
        handleTTCResult(ttc_optional, subject_vehicle_motion_msg, perceived_object_msg);
    }
}

