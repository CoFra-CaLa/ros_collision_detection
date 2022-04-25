/**
 * @file circle_algorithm.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _CIRCLE_ALGORITHM_H_
#define _CIRCLE_ALGORITHM_H_


#include <ros/ros.h>

#include <gsl/gsl_poly.h>

#include "ros_collision_detection/ttc_algorithm.h"


class CircleAlgorithm : public TTCAlgorithm
{
    public:
        CircleAlgorithm();
        float calculateTTC(
            const object_motion_t &subject_object_motion,
            const object_motion_t &perceived_object_motion
        ) override;
};

#endif // _CIRCLE_ALGORITHM_H_