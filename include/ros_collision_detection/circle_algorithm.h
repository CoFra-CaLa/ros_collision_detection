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

#include <gsl/gsl_errno.h>
#include <gsl/gsl_poly.h>

#include "ros_collision_detection/ttc_algorithm.h"


class CircleAlgorithm : public TTCAlgorithm
{
private:
    void printReceivedMotionStruct(const object_motion_t &object_motion);
	double computeSinFromHeading(const float &heading);
    double computeCosFromHeading(const float &heading);
    double computeHeadingAdjustedValue(const float &value_to_adjust, const double &trigonometry_value);
    double computeRadiusFromLength(const float &length_x, const float &length_y);
    double computeCoefficientForPowerFour(double &accel_diff_sq_sin_adj, double &accel_diff_sq_cos_adj);
    double computeCoefficientForPowerThree(double &accel_diff_sin_adj, double &accel_diff_cos_adj, double &speed_diff_sin_adj, double &speed_diff_cos_adj);
    double computeCoefficientForPowerTwo(double &accel_diff_sin_adj, double &accel_diff_cos_adj, double &speed_diff_sq_sin_adj, double &speed_diff_sq_cos_adj, double &center_pos_x_diff, double &center_pos_y_diff);
    double computeCoefficientForPowerOne(double &speed_diff_sin_adj, double &speed_diff_cos_adj, double &center_pos_x_diff, double &center_pos_y_diff);
    double computeCoefficientForPowerZero(double &center_pos_x_diff_sq, double &center_pos_y_diff_sq, double &radius_sum_sq);
    int getHighestPolynomialNonZeroDegree(boost::array<double, 5> &coefficients);
    std::vector<double> solvePolynomialEquationGSL(boost::array<double, 5> &coefficients);

public:
    CircleAlgorithm();
    boost::optional<double> calculateTTC(
        const object_motion_t &subject_object_motion,
        const object_motion_t &perceived_object_motion
    ) override;
};

#endif // _CIRCLE_ALGORITHM_H_
