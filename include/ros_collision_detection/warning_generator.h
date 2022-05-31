/**
 * @file warning_generator.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for Warning Generator class.
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


/**
 * @brief Class that receives calculated TTCs, generates warnings from them and publishes the warnings.
 * 
 */
class WarningGenerator
{
private:
    /**
     * @brief Shared pointer to an concrete instance of the WarningGeneratorAlgorithm interface.
     * 
     * A shared pointer to the warning generator algorithm. This concrete algorithm must implement the WarningGeneratorAlgorithm interface.
     * 
     */
    boost::shared_ptr<WarningGeneratorAlgorithm> warning_generator_algorithm;

    /**
     * @brief ROS publisher for collision warnings to topic "/collision_warning".
     * 
     */
    ros::Publisher &collision_warning_publisher;

public:
    /**
     * @brief Construct a new Warning Generator object.
     * 
     * Construct a new Warning Generator object with the passed ROS publisher as collision warning publisher
     * and an empty Warning Generator Algorithm member variable.
     * The concrete Warning Generator Algorithm has to be set with the respective Setter method before
     * the warning geneneration begins.
     * 
     * @param publisher ROS publisher for collision warnings.
     */
    WarningGenerator(ros::Publisher &publisher);

    /**
     * @brief Set the Warning Generator Algorithm member variable.
     * 
     * @param algorithm Pointer to an concrete instance of a class that implements the interface WarningGeneratorAlgorithm.
     */
    void setWarningGeneratorAlgorithm(boost::shared_ptr<WarningGeneratorAlgorithm> &algorithm);

    /**
     * @brief Set the Collision Warning Publisher member variable.
     * 
     * @param publisher ROS publisher for collision warnings.
     */
    void setCollisionWarningPublisher(ros::Publisher &publisher);

    /**
     * @brief Generate a collision warning and publish it.
     * 
     * First generate a collision warning from the Subject Vehicle Motion, the Perceived Object Motion and the Time-To-Collision.
     * Then publish the collision warning using the publisher.
     * 
     * @param subject_vehicle_motion_msg The Subject Vehicle Motion message used for TTC calculation.
     * @param perceived_object_motion_msg The Perceived Object Motion message used for TTC calculation.
     * @param ttc The Time-To-Collision computed between the Subject Vehicle and the Perceived Object.
     */
    void createWarning(
        const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg, 
        const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg,
        double ttc
    );
};

#endif // _WARNING_GENERATOR_H_
