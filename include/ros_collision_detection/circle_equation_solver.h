/**
 * @file circle_equation_solver.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for the Circle Equation Solver namespace.
 * @version 0.1
 * @date 2022-05-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _CIRCLE_EQUATION_SOLVER_H_
#define _CIRCLE_EQUATION_SOLVER_H_


#include <ros/ros.h>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_poly.h>

namespace circle_equation_solver
{
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

};
#endif  // _CIRCLE_EQUATION_SOLVER_H_