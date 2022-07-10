/**
 * @file n_circle_algorithm.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Implementation of the methods of extended n Circle Algorithm class.
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros_collision_detection/n_circle_algorithm.h"

#define DEFAULT_CIRCLE_COUNT 1      //!< default number of circles if no circle_count is passed

#define POLYNOMIAL_ARRAY_LENGTH 5   //!< quartic equation has variable of degree 0 to 4


NCircleAlgorithm::NCircleAlgorithm()
{
    ROS_INFO("NCircleAlgorithm::NCircleAlgorithm constructed.");
}

void NCircleAlgorithm::init(parameter_map_t &parameter_map)
{
    try
    {
        boost::variant<int, std::string> variant_circle_count = parameter_map.at("ttc_algorithm_circle_count");
        if(int *circle_count_ptr = boost::get<int>(&variant_circle_count))
        {
            int cirlce_count = *circle_count_ptr;
            if(cirlce_count >= 1)
            {
                n = cirlce_count;
                ROS_INFO("NCircleAlgorithm::init with n = %d.", cirlce_count);
            }
        }
        else
        {
            n = DEFAULT_CIRCLE_COUNT;
            ROS_ERROR("NCircleAlgorithm::init: 'ttc_algorithm_circle_count' could not be retrieved. Using default n=%d.", DEFAULT_CIRCLE_COUNT);
        }
    }
    catch(const std::out_of_range &e)
    {
        n = DEFAULT_CIRCLE_COUNT;
        ROS_ERROR("NCircleAlgorithm::init: no 'ttc_algorithm_circle_count' found. Using default n=%d.", DEFAULT_CIRCLE_COUNT);
    }
}

std::string NCircleAlgorithm::convertMotionStructToString(const object_motion_t &object_motion)
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

double NCircleAlgorithm::computeSinFromHeading(const float &heading)
{
    double result = sin(heading * M_PI / 180.0);
    return result;
}

double NCircleAlgorithm::computeCosFromHeading(const float &heading)
{
    double result = cos(heading * M_PI / 180.0);
    return result;
}

double NCircleAlgorithm::computeHeadingAdjustedValue(const float &value_to_adjust, const double &trigonometric_value)
{
    return trigonometric_value * value_to_adjust;
}

double NCircleAlgorithm::computeFrontBumperPos(const float &center_pos, const double &trigonometric_value, const float &length)
{
    double result = center_pos + (trigonometric_value * length / 2);
    ROS_DEBUG("NCircleAlgorithm::computeFrontBumperPos: result: %f | from value: %f, trigonometry: %f, length: %f.", result, center_pos, trigonometric_value, length);
    return result;
}

std::vector<boost::array<double, 2>> NCircleAlgorithm::computeAllCircleCenters(const double &front_bumper_pos_x, const double &front_bumper_pos_y, const double &sin_heading, const double &cos_heading, const double &length, const int &circle_count)
{
    std::vector<boost::array<double, 2>> circles;

    for(int i = 0; i < circle_count; i++)
    {
        int factor = i + 1;
        double part_length = length / (circle_count + 1);
        boost::array<double, 2> circle_i_pos;
        circle_i_pos[0] = computeCircleCenter(front_bumper_pos_x, factor, sin_heading, part_length);
        circle_i_pos[1] = computeCircleCenter(front_bumper_pos_y, factor, cos_heading, part_length);
        circles.push_back(circle_i_pos);
    }
    
    ROS_DEBUG("NCircleAlgorithm::computeAllCircleCenters: all circles from: (%f,%f) with sin: %f, cos: %f, length: %f, circle_count: %d.", front_bumper_pos_x, front_bumper_pos_y, sin_heading, cos_heading, length, circle_count);
    for(std::vector<boost::array<double, 2>>::iterator it = circles.begin(); it != circles.end(); ++it)
    {
        ROS_DEBUG("circle center: (%f,%f).", (*it)[0], (*it)[1]);
    }
    return circles;
}

double NCircleAlgorithm::computeCircleCenter(const double &front_bumper_pos, const int &factor, const double &trigonometric_value, const double &part_length)
{
    return front_bumper_pos - factor * (trigonometric_value * part_length);
}

double NCircleAlgorithm::computeRadius(const float &length, const float &width, const int &circle_count)
{
    // radius = sqrt( (length / (n+1))^2 + (width / 2)^2 )
    double part_length = length / (n + 1);
    double half_width = width / 2;
    double result = sqrt(part_length * part_length + half_width * half_width);
    ROS_DEBUG("NCircleAlgorithm::computeRadius: radius = %f | length: %f, width: %f, circle_count: %d", result, length, width, circle_count);
    return result;
}

std::vector<double> NCircleAlgorithm::calculatePossibleTTCs(const object_motion_t &subject_object_motion, const object_motion_t &perceived_object_motion, int circle_count)
{
    // list of all computed TTCs from the collision checks between all subject object's circles and perceived object's circles
    std::vector<double> possible_ttc_list;

    // common values necessary for all circles
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

    // differences between subject object's values and perceived object's values
    double accel_diff_sin_adjusted = accel_subject_obj_sin_adjusted - accel_perceived_obj_sin_adjusted; //!< sin(alpha) * a_i - sin(beta) * a_j
    double accel_diff_cos_adjusted = accel_subject_obj_cos_adjusted - accel_perceived_obj_cos_adjusted; //!< cos(alpha) * a_i - cos(beta) * a_j
    double speed_diff_sin_adjusted = speed_subject_obj_sin_adjusted - speed_perceived_obj_sin_adjusted; //!< sin(alpha) * v_i - sin(beta) * v_j
    double speed_diff_cos_adjusted = speed_subject_obj_cos_adjusted - speed_perceived_obj_cos_adjusted; //!< cos(alpha) * v_i - cos(beta) * v_j

    // squares of the differences
    double accel_diff_square_sin_adjusted = accel_diff_sin_adjusted * accel_diff_sin_adjusted;  //!< (sin(alpha) * a_i - sin(beta) * a_j)^2
    double accel_diff_square_cos_adjusted = accel_diff_cos_adjusted * accel_diff_cos_adjusted;  //!< (cos(alpha) * a_i - cos(beta) * a_j)^2
    double speed_diff_square_sin_adjusted = speed_diff_sin_adjusted * speed_diff_sin_adjusted;  //!< (sin(alpha) * v_i - sin(beta) * v_j)^2
    double speed_diff_square_cos_adjusted = speed_diff_cos_adjusted * speed_diff_cos_adjusted;  //!< (cos(alpha) * v_i - cos(beta) * v_j)^2

    // compute front bumper position for subject object and perceived object
    double front_bumper_pos_x_subject_obj = computeFrontBumperPos(subject_object_motion.center_pos_x, sin_subject_obj_heading, subject_object_motion.length);
    double front_bumper_pos_y_subject_obj = computeFrontBumperPos(subject_object_motion.center_pos_y, cos_subject_obj_heading, subject_object_motion.length);

    double front_bumper_pos_x_perceived_obj = computeFrontBumperPos(perceived_object_motion.center_pos_x, sin_perceived_obj_heading, perceived_object_motion.length);
    double front_bumper_pos_y_perceived_obj = computeFrontBumperPos(perceived_object_motion.center_pos_y, cos_perceived_obj_heading, perceived_object_motion.length);

    // split the line [front_bumper_pos, rear_bumper_pos] for subject object and perceived object into n+1 equally long parts and
    // compute coordinates of the n circle centers for subject object and perceived object
    std::vector<boost::array<double, 2>> circles_subject_obj = computeAllCircleCenters(front_bumper_pos_x_subject_obj, front_bumper_pos_y_subject_obj, sin_subject_obj_heading, cos_subject_obj_heading, subject_object_motion.length, n);
    std::vector<boost::array<double, 2>> circles_perceived_obj = computeAllCircleCenters(front_bumper_pos_x_perceived_obj, front_bumper_pos_y_perceived_obj, sin_perceived_obj_heading, cos_perceived_obj_heading, perceived_object_motion.length, n);

    // compute the radius for the n circles for subject object and perceived object
    double circle_radius_subject_obj = computeRadius(subject_object_motion.length, subject_object_motion.width, n);        //!< r_i
    double circle_radius_perceived_obj = computeRadius(perceived_object_motion.length, perceived_object_motion.width, n);  //!< r_j
    
    // sum of the radii of subject object and perceived object
    double radius_sum = circle_radius_subject_obj + circle_radius_perceived_obj;  //!< r_i + r_j

    // square of the sum of the radii
    double radius_sum_square = radius_sum * radius_sum; //!< (r_i + r_j)^2

    // the coefficients for polynomial equation stored upwards from least power: [b_0, b_1, b_2, b_3, b_4]
    boost::array<double, POLYNOMIAL_ARRAY_LENGTH> coefficients;

    // the real roots of the polynomial equation
    std::vector<double> real_positive_roots;

    // collision check between all subject obj circles and perceived obj circles
    for(int i = 0; i < n; i++)      // for all subject obj circles
    {
        for(int j = 0; j < n; j++)  // for all perceived obj circles
        {
            ROS_DEBUG("NCircleAlgorithm::calculatePossibleTTCs Computing for i=%d, j=%d: subject (%f,%f) | perceived (%f,%f).", i, j, circles_subject_obj.at(i).at(0), circles_subject_obj.at(i).at(1), circles_perceived_obj.at(j).at(0), circles_perceived_obj.at(j).at(1));
            double circle_center_pos_x_diff = circles_subject_obj.at(i).at(0) - circles_perceived_obj.at(j).at(0);   //!< x_i - x_j
            double circle_center_pos_y_diff = circles_subject_obj.at(i).at(1) - circles_perceived_obj.at(j).at(1);   //!< y_i - y_j

            // squares of the differences        
            double circle_center_pos_x_diff_square = circle_center_pos_x_diff * circle_center_pos_x_diff;    //!< (x_i - x_j)^2
            double circle_center_pos_y_diff_square = circle_center_pos_y_diff * circle_center_pos_y_diff;    //!< (y_i - y_j)^2

            // prepare coefficients of the polynomial P(t) = b_0 + b_1 * t^1 + b_2 * t^2 + b_3 * t^3 + b_4 * t^4
            coefficients[0] = circle_equation_solver::computeCoefficientForPowerZero(circle_center_pos_x_diff_square, circle_center_pos_y_diff_square, radius_sum_square);
            coefficients[1] = circle_equation_solver::computeCoefficientForPowerOne(speed_diff_sin_adjusted, speed_diff_cos_adjusted, circle_center_pos_x_diff, circle_center_pos_y_diff);
            coefficients[2] = circle_equation_solver::computeCoefficientForPowerTwo(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_square_sin_adjusted, speed_diff_square_cos_adjusted, circle_center_pos_x_diff, circle_center_pos_y_diff);
            coefficients[3] = circle_equation_solver::computeCoefficientForPowerThree(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_sin_adjusted, speed_diff_cos_adjusted);
            coefficients[4] = circle_equation_solver::computeCoefficientForPowerFour(accel_diff_square_sin_adjusted, accel_diff_square_cos_adjusted);

            // compute the real roots of the polynomial equation with GSL
            real_positive_roots = circle_equation_solver::solvePolynomialEquationGSL(coefficients);

            if (real_positive_roots.empty())
            {
                // no real positive root found --> no TTC could be computed
                // no TTC between the two currently used circles
                ROS_DEBUG("NCircleAlgorithm::calculatePossibleTTCs: no real positive roots found.");
            }
            else
            {
                // at least one real positive root found
                // smallest real positive root is the TTC for the two currently used circles
                double ttc_circles_i_j = *std::min_element(real_positive_roots.begin(), real_positive_roots.end());
                possible_ttc_list.push_back(ttc_circles_i_j);
            }
        }   
    }

    return possible_ttc_list;
}

boost::optional<double> NCircleAlgorithm::calculateTTC(const object_motion_t &subject_object_motion, const object_motion_t &perceived_object_motion)
{   
    // the Time-To-Collision optional return value
    boost::optional<double> ttc_optional;

    // log the received object motions
    ROS_DEBUG_STREAM("\nsubject object motion: \n" << convertMotionStructToString(subject_object_motion));
    ROS_DEBUG_STREAM("perceived object motion: \n" << convertMotionStructToString(perceived_object_motion));

    if(subject_object_motion.length <= 0 || subject_object_motion.width <= 0 || perceived_object_motion.length <= 0 || perceived_object_motion.width <= 0)
    {
        ROS_ERROR("NCircleAlgorithm::calculateTTC: length or width is not allowed to be zero or lower.");
        return ttc_optional;
    }

    if(subject_object_motion.heading < 0 || subject_object_motion.heading > 360 || perceived_object_motion.heading < 0 || perceived_object_motion.heading > 360)
    {
        ROS_ERROR("NCircleAlgorithm::calculateTTC: heading value must be in range [0; 360].");
        return ttc_optional;
    }

    // calculate all possible TTCs between subject_object_motion and perceived_object_motion, each represented by n circles
    std::vector<double> possible_ttc_list = calculatePossibleTTCs(subject_object_motion, perceived_object_motion, n);

    if(possible_ttc_list.empty())
    {
        // no possible TTC computed between any of the circles --> no TTC could be computed
        ROS_INFO("NCircleAlgorithm::calculateTTC: no TTC could be found.");

        // return empty optional
        return ttc_optional;
    }
    
    // smallest of the possible TTCs is the TTC
    ROS_DEBUG("NCircleAlgorithm::calculateTTC: list of possible TTCs:");
    for(int i = 0; i < possible_ttc_list.size(); i++)
    {
        ROS_DEBUG("possible ttc [%d] = %f", i, possible_ttc_list[i]);
    }

    ttc_optional = *std::min_element(possible_ttc_list.begin(), possible_ttc_list.end());

    return ttc_optional;
}
