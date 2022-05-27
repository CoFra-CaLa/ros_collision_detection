/**
 * @file circle_algorithm.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Implementation of the methods of Circle Algorithm class.
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros_collision_detection/circle_algorithm.h"

#define POLYNOMIAL_ARRAY_LENGTH 5   //!< quartic equation has variable of degree 0 to 4


CircleAlgorithm::CircleAlgorithm()
{
    ROS_INFO("CircleAlgorithm::CircleAlgorithm constructed.");
}

std::string CircleAlgorithm::convertMotionStructToString(const object_motion_t &object_motion)
{
	std::stringstream result;

    result << "center_pos_x = " << object_motion.center_pos_x << std::endl;
	result << "center_pos_y = " << object_motion.center_pos_y << std::endl;
	result << "length = " << object_motion.length << std::endl;
	result << "width = " << object_motion.width << std::endl;
	result << "heading = " << object_motion.heading << std::endl;
	result << "speed = " << object_motion.speed << std::endl;
	result << "acceleration = " << object_motion.acceleration << std::endl;

    return result.str();
}

double CircleAlgorithm::computeSinFromHeading(const float &heading)
{
    double result = sin(heading * M_PI / 180.0);
    return result;
}

double CircleAlgorithm::computeCosFromHeading(const float &heading)
{
    double result = cos(heading * M_PI / 180.0);
    return result;
}

double CircleAlgorithm::computeHeadingAdjustedValue(const float &value_to_adjust, const double &trigonometric_value)
{
    return trigonometric_value * value_to_adjust;
}

double CircleAlgorithm::computeRadiusFromLength(const float &length, const float &width)
{
    // radius = 0.5 * sqrt((length)^2 + (width)^2)
    return sqrt(length * length + width * width) / 2;
}

boost::optional<double> CircleAlgorithm::calculateTTC(const object_motion_t &subject_object_motion, const object_motion_t &perceived_object_motion)
{   
    // the Time-To-Collision optional return value
    boost::optional<double> ttc_optional;

    // log the received object motions
    ROS_DEBUG_STREAM("subject object motion: \n" << convertMotionStructToString(subject_object_motion));
    ROS_DEBUG_STREAM("perceived object motion: \n" << convertMotionStructToString(perceived_object_motion));

    if(subject_object_motion.length <= 0 || subject_object_motion.width <= 0 || perceived_object_motion.length <= 0 || perceived_object_motion.width <= 0)
    {
        ROS_ERROR("CircleAlgorithm::calculateTTC: length or width is not allowed to be zero or lower.");
        return ttc_optional;
    }

    if(subject_object_motion.heading < 0 || subject_object_motion.heading > 360 || perceived_object_motion.heading < 0 || perceived_object_motion.heading > 360)
    {
        ROS_ERROR("CircleAlgorithm::calculateTTC: heading value must be in range [0; 360].");
        return ttc_optional;
    }

    double sin_subject_obj_heading = computeSinFromHeading(subject_object_motion.heading);      //!< sin(alpha)
    double cos_subject_obj_heading = computeCosFromHeading(subject_object_motion.heading);      //!< cos(alpha)
    double sin_perceived_obj_heading = computeSinFromHeading(perceived_object_motion.heading);  //!< sin(beta)
    double cos_perceived_obj_heading = computeCosFromHeading(perceived_object_motion.heading);  //!< cos(beta)

    double accel_subject_obj_sin_adjusted = computeHeadingAdjustedValue(subject_object_motion.acceleration, sin_subject_obj_heading);       //!< sin(alpha) * a_i
    double accel_subject_obj_cos_adjusted = computeHeadingAdjustedValue(subject_object_motion.acceleration, cos_subject_obj_heading);       //!< cos(alpha) * a_i
    double accel_perceived_obj_sin_adjusted = computeHeadingAdjustedValue(perceived_object_motion.acceleration, sin_perceived_obj_heading); //!< sin(beta) * a_j
    double accel_perceived_obj_cos_adjusted = computeHeadingAdjustedValue(perceived_object_motion.acceleration, cos_perceived_obj_heading); //!< cos(beta) * a_j

    double speed_subject_obj_sin_adjusted = computeHeadingAdjustedValue(subject_object_motion.speed, sin_subject_obj_heading);          //!< sin(alpha) * v_i
    double speed_subject_obj_cos_adjusted = computeHeadingAdjustedValue(subject_object_motion.speed, cos_subject_obj_heading);          //!< cos(alpha) * v_i
    double speed_perceived_obj_sin_adjusted = computeHeadingAdjustedValue(perceived_object_motion.speed, sin_perceived_obj_heading);    //!< sin(beta) * v_j
    double speed_perceived_obj_cos_adjusted = computeHeadingAdjustedValue(perceived_object_motion.speed, cos_perceived_obj_heading);    //!< cos(beta) * v_j

    double radius_subject_obj = computeRadiusFromLength(subject_object_motion.length, subject_object_motion.width);        //!< r_i
    double radius_perceived_obj = computeRadiusFromLength(perceived_object_motion.length, perceived_object_motion.width);  //!< r_j

    // differences between subject object's values and perceived object's values
    double accel_diff_sin_adjusted = accel_subject_obj_sin_adjusted - accel_perceived_obj_sin_adjusted; //!< sin(alpha) * a_i - sin(beta) * a_j
    double accel_diff_cos_adjusted = accel_subject_obj_cos_adjusted - accel_perceived_obj_cos_adjusted; //!< cos(alpha) * a_i - cos(beta) * a_j
    double speed_diff_sin_adjusted = speed_subject_obj_sin_adjusted - speed_perceived_obj_sin_adjusted; //!< sin(alpha) * v_i - sin(beta) * v_j
    double speed_diff_cos_adjusted = speed_subject_obj_cos_adjusted - speed_perceived_obj_cos_adjusted; //!< cos(alpha) * v_i - cos(beta) * v_j
    double center_pos_x_diff = subject_object_motion.center_pos_x - perceived_object_motion.center_pos_x;   //!< x_i - x_j
    double center_pos_y_diff = subject_object_motion.center_pos_y - perceived_object_motion.center_pos_y;   //!< y_i - y_j

    // sum of the radii of subject object and perceived object
    double radius_sum = radius_subject_obj + radius_perceived_obj;  //!< r_i + r_j

    // squares of the differences
    double accel_diff_square_sin_adjusted = accel_diff_sin_adjusted * accel_diff_sin_adjusted;  //!< (sin(alpha) * a_i - sin(beta) * a_j)^2
    double accel_diff_square_cos_adjusted = accel_diff_cos_adjusted * accel_diff_cos_adjusted;  //!< (cos(alpha) * a_i - cos(beta) * a_j)^2
    double speed_diff_square_sin_adjusted = speed_diff_sin_adjusted * speed_diff_sin_adjusted;  //!< (sin(alpha) * v_i - sin(beta) * v_j)^2
    double speed_diff_square_cos_adjusted = speed_diff_cos_adjusted * speed_diff_cos_adjusted;  //!< (cos(alpha) * v_i - cos(beta) * v_j)^2
    double center_pos_x_diff_square = center_pos_x_diff * center_pos_x_diff;    //!< (x_i - x_j)^2
    double center_pos_y_diff_square = center_pos_y_diff * center_pos_y_diff;    //!< (y_i - y_j)^2

    // square of the sum of the radii
    double radius_sum_square = radius_sum * radius_sum; //!< (r_i + r_j)^2

    // prepare coefficients of the polynomial P(t) = b_0 + b_1 * t^1 + b_2 * t^2 + b_3 * t^3 + b_4 * t^4
    // stored upwards from least power: [b_0, b_1, b_2, b_3, b_4]
    boost::array<double, POLYNOMIAL_ARRAY_LENGTH> coefficients;
    coefficients[0] = circle_equation_solver::computeCoefficientForPowerZero(center_pos_x_diff_square, center_pos_y_diff_square, radius_sum_square);
    coefficients[1] = circle_equation_solver::computeCoefficientForPowerOne(speed_diff_sin_adjusted, speed_diff_cos_adjusted, center_pos_x_diff, center_pos_y_diff);
    coefficients[2] = circle_equation_solver::computeCoefficientForPowerTwo(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_square_sin_adjusted, speed_diff_square_cos_adjusted, center_pos_x_diff, center_pos_y_diff);
    coefficients[3] = circle_equation_solver::computeCoefficientForPowerThree(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_sin_adjusted, speed_diff_cos_adjusted);
    coefficients[4] = circle_equation_solver::computeCoefficientForPowerFour(accel_diff_square_sin_adjusted, accel_diff_square_cos_adjusted);

    // compute the real roots of the polynomial equation with GSL
    std::vector<double> real_positive_roots = circle_equation_solver::solvePolynomialEquationGSL(coefficients);

    if (real_positive_roots.empty())
    {
        // no real positive root found --> no TTC could be computed
        ROS_DEBUG("CircleAlgorithm::calculateTTC: no real positive roots found.");

        // return empty optional
        return ttc_optional;
    }
    
    // smallest real positive root is the TTC
    ttc_optional = *std::min_element(real_positive_roots.begin(), real_positive_roots.end());

    return ttc_optional;
}
