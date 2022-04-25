/**
 * @file circle_algorithm.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros_collision_detection/circle_algorithm.h"

CircleAlgorithm::CircleAlgorithm()
{
    ROS_INFO("In CircleAlgorithm constructor.");
}

float CircleAlgorithm::calculateTTC(const object_motion_t &subject_object_motion, const object_motion_t &perceived_object_motion)
{
    // TODO: implement according to formula
    ROS_INFO("In CircleAlgorithm::calculateTTC");
    float ttc_output = 5.4321;
    return ttc_output;
}