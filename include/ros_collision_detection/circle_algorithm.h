/**
 * @file circle_algorithm.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for the Circle Algorithm class.
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _CIRCLE_ALGORITHM_H_
#define _CIRCLE_ALGORITHM_H_


#include <ros/ros.h>

#include "ros_collision_detection/circle_equation_solver.h"

#include "ros_collision_detection/ttc_algorithm.h"


/**
 * @brief Class that calculates the Time-To-Collision based on the Circle Algorithm.
 * 
 * A class that calculates the TTC by solving a quartic equation for TTC. The equation is taken from
 * "New Algorithms for Computing the Time-To-Collision in Freeway Traffic Simulation Models"
 * by Jia Hou, George F. List and Xiucheng Guo, https://doi.org/10.1155/2014/761047.
 * 
 */
class CircleAlgorithm : public TTCAlgorithm
{
private:
    /**
     * @brief Represent the Object Motion struct as a string. 
     * 
     * @param object_motion The Object Motion struct to be represented as string.
     * @return String representation of the Object Motion struct.
     */
    std::string convertMotionStructToString(const object_motion_t &object_motion);

    /**
     * @brief Compute the Sine function value of heading.
     * 
     * @param heading The heading of an object.
     * @return The Sine function result.
     */
	double computeSinFromHeading(const float &heading);

    /**
     * @brief Compute the Cosine function value of heading.
     * 
     * @param heading The heading of an object.
     * @return The Cosine function result.
     */
    double computeCosFromHeading(const float &heading);

    /**
     * @brief Adjust a value with the trigonometric value by multiplication.
     * 
     * @param value_to_adjust The value that is adjusted with trigonometric value.
     * @param trigonometric_value The trigonometric value from a trigonometric function.
     * @return The adjusted value.
     */
    double computeHeadingAdjustedValue(const float &value_to_adjust, const double &trigonometric_value);

    /**
     * @brief Compute the radius of the enclosing circle by using the diagonal of the rectangle.
     * 
     * @param length The length of the rectangle.
     * @param width The width of the rectangle.
     * @return The radius of the circle enclosing the rectangle.
     */
    double computeRadiusFromLength(const float &length, const float &width);

public:
    /**
     * @brief Construct a new Circle Algorithm object.
     * 
     */
    CircleAlgorithm();

    /**
     * @brief Initialize the Circle Algorithm.
     * 
     * Overridden method from interface TTCAlgorithm. 
     * 
     * @param parameter_map A key-value map containing parameter names and parameter values.
     */
    void init(parameter_map_t &parameter_map) override {};

    /**
     * @brief Calculate the Time-To-Collision between the Subject Object Motion and the Perceived Object Motion using the polynomial equation from the Circle Algorithm.
     * 
     * @param subject_object_motion Object Motion struct representing the Subject Object Motion.
     * @param perceived_object_motion Object Motion struct representing the Perceived Object Motion.
     * @return Optional that either contains a valid Time-To-Collision or has no valid content.
     */
    boost::optional<double> calculateTTC(
        const object_motion_t &subject_object_motion,
        const object_motion_t &perceived_object_motion
    ) override;
};

#endif // _CIRCLE_ALGORITHM_H_
