/**
 * @file ttc_calculator.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _TTC_CALCULATOR_H_
#define _TTC_CALCULATOR_H_


#include <ros/ros.h>

#include "ros_collision_detection/PerceivedObjects.h"
#include "ros_collision_detection/SubjectVehicleMotion.h"

#include "ros_collision_detection/ttc_algorithm.h"


class TTCCalculator
{
private:
    boost::shared_ptr<TTCAlgorithm> ttc_algorithm;
    object_motion_t createObjectMotionFromSubjectVehicleMotion(const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg);
    object_motion_t createObjectMotionFromPerceivedObjectMotion(const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg);

public:
    TTCCalculator();
    ~TTCCalculator();
    void setTTCAlgorithm(TTCAlgorithm *algorithm);
    void calculateAllTTCs(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects_msg, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg);

};

#endif // _TTC_CALCULATOR_H_
