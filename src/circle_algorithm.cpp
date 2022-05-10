/**
 * @file circle_algorithm.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief 
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
    ROS_DEBUG("CircleAlgorithm::CircleAlgorithm constructor.");
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

double CircleAlgorithm::computeHeadingAdjustedValue(const float &value_to_adjust, const double &trigonometry_value)
{
    return trigonometry_value * value_to_adjust;
}

double CircleAlgorithm::computeRadiusFromLength(const float &length, const float &width)
{
    // radius = 0.5 * sqrt((length)^2 + (width)^2)
    return sqrt(length * length + width * width) / 2;
}

double CircleAlgorithm::computeCoefficientForPowerFour(double &accel_diff_sq_sin_adj, double &accel_diff_sq_cos_adj)
{
    // [ (sin(alpha) * a_i - sin(beta) * a_j)^2 + (cos(alpha) * a_i - cos(beta) * a_j)^2 ] * 0.25
    double result = (accel_diff_sq_sin_adj + accel_diff_sq_cos_adj) / 4;
    return result;
}

double CircleAlgorithm::computeCoefficientForPowerThree(double &accel_diff_sin_adj, double &accel_diff_cos_adj, double &speed_diff_sin_adj, double &speed_diff_cos_adj)
{
    // (sin(alpha) * a_i - sin(beta) * a_j) * (sin(alpha) * v_i - sin(beta) * v_j) + 
    // (cos(alpha) * a_i - cos(beta) * a_j) * (cos(alpha) * v_i - cos(beta) * v_j)
    return accel_diff_sin_adj * speed_diff_sin_adj + accel_diff_cos_adj * speed_diff_cos_adj;
}

double CircleAlgorithm::computeCoefficientForPowerTwo(double &accel_diff_sin_adj, double &accel_diff_cos_adj, double &speed_diff_sq_sin_adj, double &speed_diff_sq_cos_adj, double &center_pos_x_diff, double &center_pos_y_diff)
{
    // (sin(alpha) * v_i - sin(beta) * v_j)^2 + (sin(alpha) * a_i - sin(beta) * a_j) * (x_i - x_j) +
    // (cos(alpha) * v_i - cos(beta) * v_j)^2 + (cos(alpha) * a_i - cos(beta) * a_j) * (y_i - y_j)
    return speed_diff_sq_sin_adj + accel_diff_sin_adj * center_pos_x_diff + speed_diff_sq_cos_adj + accel_diff_cos_adj * center_pos_y_diff;
}

double CircleAlgorithm::computeCoefficientForPowerOne(double &speed_diff_sin_adj, double &speed_diff_cos_adj, double &center_pos_x_diff, double &center_pos_y_diff)
{
    // [ (sin(alpha) * v_i - sin(beta) * v_j) * (x_i - x_j) +
    //   (cos(alpha) * v_i - cos(beta) * v_j) * (y_i - y_j)  ] * 2
    return (speed_diff_sin_adj * center_pos_x_diff + speed_diff_cos_adj * center_pos_y_diff) * 2;
}

double CircleAlgorithm::computeCoefficientForPowerZero(double &center_pos_x_diff_sq, double &center_pos_y_diff_sq, double &radius_sum_sq)
{
    // (x_i - x_j)^2 + (y_i - y_j)^2 - (r_i + r_j)^2
    return center_pos_x_diff_sq + center_pos_y_diff_sq - radius_sum_sq;
}

int CircleAlgorithm::getHighestPolynomialNonZeroDegree(boost::array<double, 5> &coefficients)
{
    // polynomial coefficients are stored from lower degree to higher degree
    int highest_non_zero_degree = 4;    //!< quartic equation
    for(int i = 0; i < POLYNOMIAL_ARRAY_LENGTH - 1; i++)
    {
        if(coefficients.at(POLYNOMIAL_ARRAY_LENGTH - 1 - i) == 0)
        {
            highest_non_zero_degree--;
        }
        else
        {
            break;
        } 
    }

    // log highest non-zero polynomial degree
    ROS_DEBUG("CircleAlgorithm::getHighestPolynomialNonZeroDegree: Highest polynomial degree: %d", highest_non_zero_degree);

    return highest_non_zero_degree;
}

std::vector<double> CircleAlgorithm::solvePolynomialEquationGSL(boost::array<double, POLYNOMIAL_ARRAY_LENGTH> &coefficients)
{
    // the real positive roots to be returned
    std::vector<double> real_positive_roots;

    // log coefficients
    ROS_DEBUG("CircleAlgorithm::solvePolynomialEquationGSL: computed coefficients:");
    for(int i = 0; i < POLYNOMIAL_ARRAY_LENGTH; i++)
    {
        ROS_DEBUG("coefficient %d = %f", i, coefficients.at(i));
    }

    // get the highest degree of the polynomial coefficient that is not zero
    int highest_polynomial_degree_nzero = getHighestPolynomialNonZeroDegree(coefficients);
    if(highest_polynomial_degree_nzero < 1)
    {
        // all polynomials of degree >= 1 are zero
        // no solution possible for P(t) = b_0 * t^0
        // return empty real_positive_roots vector
        ROS_ERROR("CircleAlgorithm::solvePolynomialEquationGSL: polynomial degree is too low!");
        return real_positive_roots;
    }

    int polynomial_array_length_adjusted = highest_polynomial_degree_nzero + 1; //!< adjusted length of the polynomial coefficient array for GSL

    // prepare array of coefficients for GSL
    double poly_coefficients[polynomial_array_length_adjusted];
    for (int i = 0; i < polynomial_array_length_adjusted; i++)
    {
        poly_coefficients[i] = coefficients.at(i);
    }

    // array stores results of GSL, alternating real and imaginary components
    double complex_results[2*(polynomial_array_length_adjusted - 1)];

    // Workspace necessary for GSL to solve the polynomial equation
    gsl_poly_complex_workspace *gsl_workspace = gsl_poly_complex_workspace_alloc(polynomial_array_length_adjusted);

    // solve the polynomial equation defined by poly_coefficients and store results to array complex_results
    int status = gsl_poly_complex_solve(poly_coefficients, polynomial_array_length_adjusted, gsl_workspace, complex_results);

    // free GSL workspace
    gsl_poly_complex_workspace_free(gsl_workspace);

    if(status == GSL_EFAILED)
    {
        // TODO: handle GSL error
        ROS_ERROR("CircleAlgorithm::solvePolynomialEquationGSL: GSL error");
    }

    // log results from GSL
    ROS_DEBUG("CircleAlgorithm::solvePolynomialEquationGSL: results from GSL: ");
    for(int i = 0; i < polynomial_array_length_adjusted; i++)
    {
        ROS_DEBUG("result %d | real = %+.18f | imag = %+.18f",i, complex_results[2*i], complex_results[2*i+1]);
    }

    // populate result vector only with real roots
    for (int i = 0; i < polynomial_array_length_adjusted; i++)
    {
        // TODO: might need approximate check instead of exact check
        if(complex_results[2*i+1] == 0)   // check if imaginary part is zero --> root is real
        {
            ROS_DEBUG("CircleAlgorithm::solvePolynomialEquationGSL: result %d is real: %+.18f %+.18f", i, complex_results[2*i], complex_results[2*i+1]);
            
            if (complex_results[2*i] > 0) // check if real root is positive
            {
                // only real positive roots are returned
                real_positive_roots.push_back(complex_results[2*i]);
                ROS_DEBUG("CircleAlgorithm::solvePolynomialEquationGSL: result %d is real and positive: %+.18f %+.18f", i, complex_results[2*i], complex_results[2*i+1]);
            }
        }
        else
        {
            ROS_DEBUG("CircleAlgorithm::solvePolynomialEquationGSL: result %d is complex: %+.18f %+.18f", i, complex_results[2*i], complex_results[2*i+1]);
        }
    }

    return real_positive_roots;   
}

boost::optional<double> CircleAlgorithm::calculateTTC(const object_motion_t &subject_object_motion, const object_motion_t &perceived_object_motion)
{   
    // the Time To Collision optional return value
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
    coefficients[0] = computeCoefficientForPowerZero(center_pos_x_diff_square, center_pos_y_diff_square, radius_sum_square);
    coefficients[1] = computeCoefficientForPowerOne(speed_diff_sin_adjusted, speed_diff_cos_adjusted, center_pos_x_diff, center_pos_y_diff);
    coefficients[2] = computeCoefficientForPowerTwo(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_square_sin_adjusted, speed_diff_square_cos_adjusted, center_pos_x_diff, center_pos_y_diff);
    coefficients[3] = computeCoefficientForPowerThree(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_sin_adjusted, speed_diff_cos_adjusted);
    coefficients[4] = computeCoefficientForPowerFour(accel_diff_square_sin_adjusted, accel_diff_square_cos_adjusted);

    // compute the real roots of the polynomial equation with GSL
    std::vector<double> real_positive_roots = solvePolynomialEquationGSL(coefficients);

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
