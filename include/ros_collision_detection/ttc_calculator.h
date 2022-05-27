/**
 * @file ttc_calculator.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for TTC Calculator class.
 * @version 0.1
 * @date 2022-04-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _TTC_CALCULATOR_H_
#define _TTC_CALCULATOR_H_


#include <ros/ros.h>

#include <boost/signals2.hpp>

#include "ros_collision_detection/PerceivedObjects.h"
#include "ros_collision_detection/SubjectVehicleMotion.h"

#include "ros_collision_detection/ttc_algorithm.h"


/**
 * @brief Typedef for a boost signal that contains a callback function with three parameters.
 * 
 * This signal is used to trigger the warning generation by the Warning Generator.
 * The callback function must fulfill the interface defined by the function type signature:
 * The callback function has a Subject Vehicle Motion pointer, a Perceived Object Motion pointer and a double value as parameters. 
 * The callback function returns void.
 * 
 */
typedef boost::signals2::signal<void (ros_collision_detection::SubjectVehicleMotionConstPtr, ros_collision_detection::PerceivedObjectMotionConstPtr, double)> warning_signal_t;

/**
 * @brief Class that manages the Time-To-Collision calculation and passes valid TTCs to the Warning Generator.
 * 
 */
class TTCCalculator
{
private:
    /**
     * @brief Shared pointer to an concrete instance of the TTCAlgorithm interface.
     * 
     * A shared pointer to the TTC calculation algorithm. This concrete algorithm must implement the TTCAlgorithm interface.
     * 
     */
    boost::shared_ptr<TTCAlgorithm> ttc_algorithm;

    /**
     * @brief Signal for a callback to the Warning Generator.
     * 
     */
    warning_signal_t warning_signal;

    /**
     * @brief The length of the vehicle that is considered the subject vehicle for this TTC Calculator instance.
     * 
     */
    float length_subject_vehicle;

    /**
     * @brief The length of the vehicle that is considered the subject vehicle for this TTC Calculator instance.
     * 
     */
    float width_subject_vehicle;

    /**
     * @brief Call the callback function that is registered at the warning_signal.
     * 
     * @param subject_vehicle_motion_msg The Subject Vehicle Motion message used for TTC calculation.
     * @param perceived_object_motion_msg The Perceived Object Motion message used for TTC calculation.
     * @param ttc The Time-To-Collision computed between the Subject Vehicle and the Perceived Object.
     */
    void sendWarningSignalCallback(const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg, const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg, double ttc);

    /**
     * @brief Create a Object Motion from Subject Vehicle Motion object.
     * 
     * @param subject_vehicle_motion_msg The Subject Vehicle Motion message to be used for Object Motion.
     * @return Object Motion struct representing the Subject Vehicle Motion.
     */
    object_motion_t createObjectMotionFromSubjectVehicleMotion(const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg);

    /**
     * @brief Create a Object Motion from Perceived Object Motion object.
     * 
     * @param perceived_object_motion_msg The Perceived Object Motion message to be used for Object Motion.
     * @return Object Motion struct representing the Perceived Object Motion. 
     */
    object_motion_t createObjectMotionFromPerceivedObjectMotion(const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg);

    /**
     * @brief Deal with the TTC optional from the TTC calculation.
     * 
     * @param ttc_optional Optional that either contains a valid Time-To-Collision or has no valid content.
     * @param subject_vehicle_motion_msg The Subject Vehicle Motion message used for TTC calculation.
     * @param perceived_object_motion_msg The Perceived Object Motion message used for TTC calculation.
     */
    void handleTTCResult(boost::optional<double> &ttc_optional, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg, const ros_collision_detection::PerceivedObjectMotionConstPtr& perceived_object_motion_msg);

public:
    /**
     * @brief Construct a new TTC Calculator object.
     * 
     * Construct a new TTC Calculator object with an empty TTC Algorithm member variable.
     * The concrete TTC Algorithm has to be set with the respective Setter method before
     * the TTC calculation begins.
     * 
     */
    TTCCalculator();

    /**
     * @brief Set the TTC Algorithm member variable.
     * 
     * @param algorithm Pointer to an concrete instance of a class that implements the interface TTCAlgorithm.
     */
    void setTTCAlgorithm(boost::shared_ptr<TTCAlgorithm> &algorithm);

    /**
     * @brief Add a callback function to the warning_signal that fulfills the type signature defined by warning_signal_t.
     * 
     * @param slot The callback function to be added to the signal.
     */
    void addWarningSignalCallback(const warning_signal_t::slot_type& slot);

    /**
     * @brief Set the length and the width of the vehicle that is considered the subject vehicle for this TTC Calculator instance.
     * 
     * @param length The length of the subject vehicle.
     * @param width The width of the subject vehicle.
     */
    void setSubjectVehicleDimensions(float length, float width);

    /**
     * @brief Calculate all TTCs between the Subject Vehicle Motion and all Perceived Object Motions.
     * 
     * For all Perceived Object Motions that the Perceived Objects message contains, calculate the Time-To-Collision between
     * the Perceived Object and the Subject Vehicle Motion.
     * 
     * @param perceived_objects_msg The Perceived Objects message that contains several Perceived Object Motions.
     * @param subject_vehicle_motion_msg The Subject Vehicle Motion message.
     */
    void calculateAllTTCs(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects_msg, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg);

};

#endif // _TTC_CALCULATOR_H_
