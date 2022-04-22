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
    ttc_algorithm = new CircleAlgorithm(); //TODO: where to create the concrete TTC Algorithm instance?
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
    // store the subject vehicle's motion data (vehicle i) for use with all perceived objects
    float x_i = subject_vehicle_motion_msg->vehicle_movement.position.x;
    float y_i = subject_vehicle_motion_msg->vehicle_movement.position.y;
    float length_xi = 5; // TODO: get length_xi, length_yi
    float length_yi = 5; // TODO: get length_xi, length_yi
    float heading_i = subject_vehicle_motion_msg->vehicle_movement.heading;
    float speed_i = subject_vehicle_motion_msg->vehicle_movement.speed;
    float acceleration_i = subject_vehicle_motion_msg->vehicle_movement.acceleration; 

    int perceived_objects_count = perceived_objects_msg->perceived_objects.size();
    for(int i = 0; i < perceived_objects_count; i++)
    {
        ros_collision_detection::PerceivedObjectMotion perceived_object = perceived_objects_msg->perceived_objects[i];
        uint32_t object_id = perceived_object.object_movement.id;
        std::string object_type = perceived_object.object_type;

        double ttc;
        bool is_ttc_valid;
        is_ttc_valid = ttc_algorithm->calculateTTC(
            x_i, 
            y_i, 
            length_xi, 
            length_yi, 
            heading_i, 
            speed_i, 
            acceleration_i, 
            perceived_object.object_movement.position.x, 
            perceived_object.object_movement.position.y, 
            perceived_object.x_length, 
            perceived_object.y_length, 
            perceived_object.object_movement.heading, 
            perceived_object.object_movement.speed, 
            perceived_object.object_movement.acceleration, 
            &ttc
        );

        if(is_ttc_valid)
        {
            // TODO: trigger the TTC warning output to publisher
            ROS_INFO("TTCCalculator::calculateAllTTCs: TTC %f for perceived object %d.", ttc, object_id);
        }
        else
        {
            // TODO: error: TTC computation failed
            ROS_ERROR("TTCCalculator::calculateAllTTCs: calculateTTC failed.");
        }
    }
}

