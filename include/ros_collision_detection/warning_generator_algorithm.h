/**
 * @file warning_generator_algorithm.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-24
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _WARNING_GENERATOR_ALGORITHM_H_
#define _WARNING_GENERATOR_ALGORITHM_H_


#include "ros_collision_detection/PerceivedObjects.h"
#include "ros_collision_detection/SubjectVehicleMotion.h"

#include "ros_collision_detection/enum_result_type.h"


class WarningGeneratorAlgorithm
{
    public:
        virtual ResultType generateWarning(
            const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg, 
            const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg,
            double ttc
        ) = 0;

};

#endif // _WARNING_GENERATOR_ALGORITHM_H_