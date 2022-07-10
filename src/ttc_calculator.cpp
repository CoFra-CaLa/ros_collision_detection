/**
 * @file ttc_calculator.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Implementation of the methods of TTC Calculator class.
 * @version 0.1
 * @date 2022-04-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros_collision_detection/ttc_calculator.h"

#define DEFAULT_LENGTH_SUBJECT_VEHICLE 5  //!< default passenger car length, see https://sumo.dlr.de/docs/Vehicle_Type_Parameter_Defaults.html
#define DEFAULT_WIDTH_SUBJECT_VEHICLE 1.8 //!< default passenger car width, see https://sumo.dlr.de/docs/Vehicle_Type_Parameter_Defaults.html


TTCCalculator::TTCCalculator()
:ttc_algorithm(nullptr)
{
    setSubjectVehicleDimensions(DEFAULT_LENGTH_SUBJECT_VEHICLE, DEFAULT_WIDTH_SUBJECT_VEHICLE);
    ROS_DEBUG("TTCCalculator::TTCCalculator constructed with default subject vehicle dimensions: length = %f, width = %f", length_subject_vehicle, width_subject_vehicle);
}

void TTCCalculator::setTTCAlgorithm(boost::shared_ptr<TTCAlgorithm> &algorithm)
{
    ttc_algorithm.swap(algorithm);
    ROS_DEBUG("TTCCalculator::setTTCAlgorithm: set new algorithm.");
}

void TTCCalculator::addWarningSignalCallback(const warning_signal_t::slot_type& signal_subscriber)
{
    warning_signal.connect(signal_subscriber);
}

void TTCCalculator::setSubjectVehicleDimensions(float length, float width)
{
    if(length > 0 && width > 0)
    {
        length_subject_vehicle = length;
        width_subject_vehicle = width;
        ROS_DEBUG("TTCCalculator::setSubjectVehicleDimensions: subject vehicle dimensions set to: length = %f, width = %f", length_subject_vehicle, width_subject_vehicle);
    }
    else
    {
        ROS_WARN("TTCCalculator::setSubjectVehicleDimensions: length = %f or width = %f not allowed. Subject vehicle dimensions not changed.", length, width);
    }
}

void TTCCalculator::sendWarningSignalCallback(const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg, const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg, double ttc)
{
    if(warning_signal.num_slots() > 0)
    {
        warning_signal(subject_vehicle_motion_msg, perceived_object_motion_msg, ttc);
    }
    else
    {
        ROS_WARN("TTCCalculator::sendWarningSignalCallback: no slot connected to warning_signal.");
    }
}

object_motion_t TTCCalculator::createObjectMotionFromSubjectVehicleMotion(const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg)
{
    object_motion_t result;
    
    result.center_pos_x = subject_vehicle_motion_msg->vehicle_movement.position.x;
    result.center_pos_y = subject_vehicle_motion_msg->vehicle_movement.position.y;
    result.length = length_subject_vehicle;
    result.width = width_subject_vehicle;
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
    result.center_pos_x = bumper_pos_x - 0.5 * sin(heading * M_PI / 180.0) * x_length;
    result.center_pos_y = bumper_pos_y - 0.5 * cos(heading * M_PI / 180.0) * x_length;
    result.length = x_length;
    result.width = perceived_object_motion_msg->y_length;
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
        ROS_INFO("TTCCalculator: computed valid TTC %f for perceived object with ID %d.", *ttc_optional, perceived_object_motion_msg->object_movement.id);
        sendWarningSignalCallback(subject_vehicle_motion_msg, perceived_object_motion_msg, *ttc_optional);
    }
    else
    {
        // no valid TTC result --> only log
        ROS_INFO("TTCCalculator: no valid TTC could be computed for perceived object with ID %d.", perceived_object_motion_msg->object_movement.id);
    }
}

void TTCCalculator::calculateAllTTCs(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects_msg, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg)
{
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
        
        // trigger the TTC warning output to publisher
        handleTTCResult(ttc_optional, subject_vehicle_motion_msg, perceived_object_msg);
    }
}

