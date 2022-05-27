/**
 * @file n_circle_algorithm.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for the N Circle Algorithm class.
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _N_CIRCLE_ALGORITHM_H_
#define _N_CIRCLE_ALGORITHM_H_


#include <ros/ros.h>

#include "ros_collision_detection/circle_equation_solver.h"

#include "ros_collision_detection/ttc_algorithm.h"


/**
 * @brief Class that calculates the Time-To-Collision based on the extended Circle Algorithm with n circles.
 * 
 * A class that calculates the TTC by solving a quartic equation for TTC. The equation is taken from
 * "New Algorithms for Computing the Time-To-Collision in Freeway Traffic Simulation Models"
 * by Jia Hou, George F. List and Xiucheng Guo, https://doi.org/10.1155/2014/761047.
 * 
 */
class NCircleAlgorithm : public TTCAlgorithm
{
private:
    /**
     * @brief The number of circles used to represent an object.
     * 
     */
    int n;

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
     * @brief Compute the coordinate of the front bumper center. 
     * 
     * @param center_pos The center coordinate of an object.
     * @param trigonometric_value The trigonometric value from a trigonometric function.
     * @param length The length of the object.
     * @return The coodinate of the front bumper center.
     */
    double computeFrontBumperPos(const float &center_pos, const double &trigonometric_value, const float &length);

    /**
     * @brief Compute all n circle centers of the circles that represent the object from its front bumper position. 
     * 
     * @param front_bumper_pos_x The x coordinate of the front bumper center.
     * @param front_bumper_pos_y The y coordinate of the front bumper center.
     * @param sin_heading The sine value of the heading.
     * @param cos_heading The cosine value of the heading.
     * @param length The length of the heading.
     * @param circle_count The number of circles used to represent the object.
     * @return The list of all circle positions for the object. 
     */
    std::vector<boost::array<double, 2>> computeAllCircleCenters(const double &front_bumper_pos_x, const double &front_bumper_pos_y, const double &sin_heading, const double &cos_heading, const double &length, const int &circle_count);

    /**
     * @brief Compute the circle center coordinate using the front bumper center coordinate.
     * 
     * @param front_bumper_pos The coordinate of the front bumper center.
     * @param factor How many times part_length the circle center will be away of front bumper center. 
     * @param trigonometric_value The trigonometric value from a trigonometric function.
     * @param part_length The distance between two circle centers. Is equal to object's length / circle_count.
     * @return The coordinate of the circle center. 
     */
    double computeCircleCenter(const double &front_bumper_pos, const int &factor, const double &trigonometric_value, const double &part_length);

    /**
     * @brief Compute the radius of the n circles for object with length and width.
     * 
     * @param length The length of the rectangle.
     * @param width The width of the rectangle.
     * @param circle_count The number of circles that represent the object.
     * @return The radius of the n circles.
     */
    double computeRadius(const float &length, const float &width, const int &circle_count);

    /**
     * @brief Calculate the list of possible TTCs between the Subject Object Motion and the Perceived Object Motion by applying the Circle Algorithm pairwise to the circles that represent them.
     * 
     * @param subject_object_motion Object Motion struct representing the Subject Object Motion.
     * @param perceived_object_motion Object Motion struct representing the Perceived Object Motion.
     * @param circle_count Number of circles used to represent one object.
     * @return List of possible TTCs between the Subject Object Motion and the Perceived Object Motion.
     */
    std::vector<double> calculatePossibleTTCs(const object_motion_t &subject_object_motion, const object_motion_t &perceived_object_motion, int circle_count);

public:
    /**
     * @brief Construct a new N Circle Algorithm object.
     * 
     */
    NCircleAlgorithm();

    /**
     * @brief Initialize the N Circle Algorithm with the number of circles representing one object.
     * 
     * @param parameter_map A key-value map containing parameter names and parameter values. 
     */
    void init(parameter_map_t &parameter_map) override;

    /**
     * @brief Calculate the Time-To-Collision between the Subject Object Motion and the Perceived Object Motion using the polynomial equation from the Circle Algorithm with n circles.
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

#endif // _N_CIRCLE_ALGORITHM_H_
