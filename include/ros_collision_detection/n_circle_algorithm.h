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

#include <gsl/gsl_errno.h>
#include <gsl/gsl_poly.h>

#include "ros_collision_detection/ttc_algorithm.h"


/**
 * @brief Class that calculates the Time-To-Collision based on the extended Circle Algorithm with n circles.
 * 
 * A class that calculates the TTC by solving a quartic equation for TTC. The equation is taken from
 * "New Algorithms for Computing the Time-To-Collision in Freeway Traffic Simulation Models"
 * by Jia Hou, George F. List and Xiucheng Guo, https://doi.org/10.1155/2014/761047.
 * 
 * @implements TTCAlgorithm
 * 
 */
class NCircleAlgorithm : public TTCAlgorithm
{
private:
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

    double computeFrontBumperPos(const float &center_pos, const double &trigonometric_value, const float &length);

    double computeRearBumperPos(const float &center_pos, const double &trigonometric_value, const float &length);

    std::vector<boost::array<double, 2>> computeAllCircleCenters(const double &front_bumper_pos_x, const double &front_bumper_pos_y, const double &sin_value, const double &cos_value, const double &length, const int &circle_count);

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
     * @brief Compute the coefficient for the 4th power of variable TTC.
     * 
     * Compute the coefficient for variable TTC with exponent 4 using an rearranged version of the equation for the Circle Algorithm.
     * The coefficient is computed following the formula: 
     * [ (sin(alpha) * a_i - sin(beta) * a_j)^2 + (cos(alpha) * a_i - cos(beta) * a_j)^2 ] * 0.25 
     * 
     * @param accel_diff_sq_sin_adj The squared difference between the sin-adjusted accelerations of object i and object j. 
     * @param accel_diff_sq_cos_adj The squared difference between the cos-adjusted accelerations of object i and object j.
     * @return The coefficient for the 4th power of variable TTC.
     */
    double computeCoefficientForPowerFour(double &accel_diff_sq_sin_adj, double &accel_diff_sq_cos_adj);

    /**
     * @brief Compute the coefficient for the 3rd power of variable TTC.
     * 
     * Compute the coefficient for variable TTC with exponent 3 using an rearranged version of the equation for the Circle Algorithm.
     * The coefficient is computed following the formula:
     * (sin(alpha) * a_i - sin(beta) * a_j) * (sin(alpha) * v_i - sin(beta) * v_j) + 
     * (cos(alpha) * a_i - cos(beta) * a_j) * (cos(alpha) * v_i - cos(beta) * v_j)
     * 
     * @param accel_diff_sin_adj The difference between the sin-adjusted accelerations of object i and object j.
     * @param accel_diff_cos_adj The difference between the cos-adjusted accelerations of object i and object j.
     * @param speed_diff_sin_adj The difference between the sin-adjusted speeds of object i and object j.
     * @param speed_diff_cos_adj The difference between the cos-adjusted speeds of object i and object j.
     * @return The coefficient for the 3rd power of variable TTC. 
     */
    double computeCoefficientForPowerThree(double &accel_diff_sin_adj, double &accel_diff_cos_adj, double &speed_diff_sin_adj, double &speed_diff_cos_adj);

    /**
     * @brief Compute the coefficient for the 2nd power of variable TTC.
     * 
     * Compute the coefficient for variable TTC with exponent 2 using an rearranged version of the equation for the Circle Algorithm.
     * The coefficient is computed following the formula:
     * (sin(alpha) * v_i - sin(beta) * v_j)^2 + (sin(alpha) * a_i - sin(beta) * a_j) * (x_i - x_j) +
     * (cos(alpha) * v_i - cos(beta) * v_j)^2 + (cos(alpha) * a_i - cos(beta) * a_j) * (y_i - y_j)
     * 
     * @param accel_diff_sin_adj The difference between the sin-adjusted accelerations of object i and object j.
     * @param accel_diff_cos_adj The difference between the cos-adjusted accelerations of object i and object j.
     * @param speed_diff_sq_sin_adj The squared difference between the sin-adjusted speeds of object i and object j.
     * @param speed_diff_sq_cos_adj The squared difference between the cos-adjusted speeds of object i and object j.
     * @param center_pos_x_diff The difference between the x-coordinates of object i center and object j center.
     * @param center_pos_y_diff The difference between the y-coordinates of object i center and object j center.
     * @return The coefficient for the 2nd power of variable TTC.
     */
    double computeCoefficientForPowerTwo(double &accel_diff_sin_adj, double &accel_diff_cos_adj, double &speed_diff_sq_sin_adj, double &speed_diff_sq_cos_adj, double &center_pos_x_diff, double &center_pos_y_diff);

    /**
     * @brief Compute the coefficient for the 1st power of variable TTC.
     * 
     * Compute the coefficient for variable TTC with exponent 1 using an rearranged version of the equation for the Circle Algorithm.
     * The coefficient is computed following the formula:
     * [ (sin(alpha) * v_i - sin(beta) * v_j) * (x_i - x_j) +
     *   (cos(alpha) * v_i - cos(beta) * v_j) * (y_i - y_j)  ] * 2
     * 
     * @param speed_diff_sin_adj The difference between the sin-adjusted speeds of object i and object j.
     * @param speed_diff_cos_adj The difference between the cos-adjusted speeds of object i and object j.
     * @param center_pos_x_diff The difference between the x-coordinates of object i center and object j center.
     * @param center_pos_y_diff The difference between the y-coordinates of object i center and object j center.
     * @return The coefficient for the 1st power of variable TTC.
     */
    double computeCoefficientForPowerOne(double &speed_diff_sin_adj, double &speed_diff_cos_adj, double &center_pos_x_diff, double &center_pos_y_diff);

    /**
     * @brief Compute the coefficient for the constant part of the equation.
     * 
     * Compute the coefficient for the constant part using an rearranged version of the equation for the Circle Algorithm.
     * The coefficient is computed following the formula:
     * (x_i - x_j)^2 + (y_i - y_j)^2 - (r_i + r_j)^2
     * 
     * @param center_pos_x_diff_sq The squared difference between the x-coordinates of object i center and object j center.
     * @param center_pos_y_diff_sq The squared difference between the y-coordinates of object i center and object j center.
     * @param radius_sum_sq The squared sum of the radii of object i and object j.
     * @return double 
     */
    double computeCoefficientForPowerZero(double &center_pos_x_diff_sq, double &center_pos_y_diff_sq, double &radius_sum_sq);

    /**
     * @brief Find the polynomial degree that has a non-zero coefficient.
     * 
     * Find the highest degree of t from the polynomial P(t) = c_0 + c_1 * t^1 + c_2 * t^2 + c_3 * t^3 + c_4 * t^4
     * where its coefficient c_* is non-zero.
     * 
     * @param coefficients The coefficients of the different powers of variable TTC.
     * @return The highest degree of TTC with non-zero coefficient. 
     */
    int getHighestPolynomialNonZeroDegree(boost::array<double, 5> &coefficients);

    /**
     * @brief Solve the rearranged polynomial equation of the Circle Algorithm for variable TTC.
     * 
     * The rearranged polynomial P(t) = c_0 + c_1 * t^1 + c_2 * t^2 + c_3 * t^3 + c_4 * t^4, defined by the 
     * coefficients for the powers of t, is solved for t. Then only non-complex, real positive roots t 
     * are returned as possible results for TTC.
     * 
     * @param coefficients The coefficients of the different powers of variable TTC.
     * @return The list of all real positive roots of the polynomial.
     */
    std::vector<double> solvePolynomialEquationGSL(boost::array<double, 5> &coefficients);

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
    NCircleAlgorithm(int circle_count);

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
