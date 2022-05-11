/**
 * @file warning_generator_algorithm.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for Warning Generator Algorithm interface.
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


/**
 * @brief Interface that defines the warning generation method that all concrete Warning Generator Algorithm classes must implement.
 * 
 */
class WarningGeneratorAlgorithm
{
    public:
        /**
         * @brief Generate a ResultType warning from the Subject Vehicle Motion, the Perceived Object Motion and the Time-To-Collision between the two motions.
         * 
         * @param subject_vehicle_motion_msg The Subject Vehicle Motion message used for TTC calculation.
         * @param perceived_object_motion_msg The Perceived Object Motion message used for TTC calculation.
         * @param ttc The Time-To-Collision computed between the Subject Vehicle and the Perceived Object.
         * @return Warning Result Type that defines the collision warning level.
         */
        virtual ResultType generateWarning(
            const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg, 
            const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg,
            double ttc
        ) = 0;

};

#endif // _WARNING_GENERATOR_ALGORITHM_H_