/**
 * @file ttc_only_warning_algorithm.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for the TTC Only Warning Algorithm class.
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _TTC_ONLY_WARNING_ALGORITHM_H_
#define _TTC_ONLY_WARNING_ALGORITHM_H_


#include <ros/ros.h>

#include "ros_collision_detection/warning_generator_algorithm.h"


/**
 * @brief Class that generates collision warnings based on TTC thresholds only.
 * 
 */
class TTCOnlyWarningAlgorithm: public WarningGeneratorAlgorithm
{
public:
    /**
     * @brief Construct a new TTCOnlyWarningAlgorithm object.
     * 
     */
    TTCOnlyWarningAlgorithm();

    /**
     * @brief Generate a ResultType warning from the Subject Vehicle Motion, the Perceived Object Motion and the TTC only based on TTC thresholds.
     * 
     * @param subject_vehicle_motion_msg The Subject Vehicle Motion message used for TTC calculation.
     * @param perceived_object_motion_msg The Perceived Object Motion message used for TTC calculation.
     * @param ttc The Time-To-Collision computed between the Subject Vehicle and the Perceived Object.
     * @return Warning Result Type that defines the collision warning level based on TTC thresholds.
     */
    ResultType generateWarning(
            const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg, 
            const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg,
            double ttc
     ) override;
};

#endif  // _TTC_ONLY_WARNING_ALGORITHM_H_