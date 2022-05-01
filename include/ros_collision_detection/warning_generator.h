/**
 * @file warning_generator.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-24
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _WARNING_GENERATOR_H_
#define _WARNING_GENERATOR_H_


#include <ros/ros.h>

#include "ros_collision_detection/CollisionCheckResult.h"

#include "ros_collision_detection/warning_generator_algorithm.h"


class WarningGenerator
{
private:
    boost::shared_ptr<WarningGeneratorAlgorithm> warning_generator_algorithm;
    ros::Publisher &collision_warning_publisher;

public:
    WarningGenerator(ros::Publisher &publisher);
    void setWarningGeneratorAlgorithm(WarningGeneratorAlgorithm *algorithm);
    void setCollisionWarningPublisher(ros::Publisher &publisher);
    void createWarning(
        const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg, 
        const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg,
        double ttc
    );
};

#endif // _WARNING_GENERATOR_H_
