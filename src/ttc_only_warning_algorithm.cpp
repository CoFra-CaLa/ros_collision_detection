/**
 * @file ttc_only_warning_algorithm.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Implementation of the methods of TTC Only Warning Algorithm class.
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros_collision_detection/ttc_only_warning_algorithm.h"


TTCOnlyWarningAlgorithm::TTCOnlyWarningAlgorithm()
{
    ROS_INFO("TTCOnlyWarningAlgorithm::TTCOnlyWarningAlgorithm constructed.");
}

ResultType TTCOnlyWarningAlgorithm::generateWarning(const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg, const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg, double ttc)
{
    // TTC thresholds like Honda Collision Mitgation Braking System (CMBS)
    // see https://doi.org/10.1016/j.sbspro.2011.08.075
    if(ttc > 10)
    {
        return RESULT_IGNORE;
    }
    else if(ttc <= 10 && ttc > 3)
    {
        return RESULT_CLOSE_MONITORING;
    }
    else if(ttc <= 3 && ttc > 2)
    {
        return RESULT_WARNING;
    }
    else
    {
        // ttc <= 2
        return RESULT_ALERT;
    }
}