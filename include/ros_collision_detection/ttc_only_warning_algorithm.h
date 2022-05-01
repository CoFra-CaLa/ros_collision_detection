/**
 * @file ttc_only_warning_algorithm.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _TTC_ONLY_WARNING_ALGORITHM_H_
#define _TTC_ONLY_WARNING_ALGORITHM_H_


#include "ros_collision_detection/warning_generator_algorithm.h"

class TTCOnlyWarningAlgorithm: public WarningGeneratorAlgorithm
{
private:

public:
    ResultType generateWarning(
            const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg, 
            const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg,
            double ttc
     ) override;
};

#endif  // _TTC_ONLY_WARNING_ALGORITHM_H_