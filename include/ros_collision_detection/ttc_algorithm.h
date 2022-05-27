/**
 * @file ttc_algorithm.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for TTC Algorithm interface.
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _TTC_ALGORITM_H_
#define _TTC_ALGORITM_H_

#include <boost/variant.hpp>
#include <boost/optional.hpp>

/**
 * @brief Map that includes the parameters for TTC Algorithm intialization.
 * 
 */
typedef std::map<std::string, boost::variant<int, std::string>> parameter_map_t;

/**
 * @brief Struct that represents a object's motion.
 * 
 */
typedef struct {
    float center_pos_x; //!< The object's center x-coordinate.
    float center_pos_y; //!< The object's center y-coordinate.
    float length;       //!< The longer side of the object.
    float width;        //!< The shorter side of the object.
    float heading;      //!< The object's heading.
    float speed;        //!< The object's speed.
    float acceleration; //!< The object's acceleration.
} object_motion_t;


/**
 * @brief Interface that defines the TTC calculation method that all concrete TTC Algorithm classes must implement. 
 * 
 */
class TTCAlgorithm
{
public:
    /**
     * @brief Initialize the TTC Algorithm with the passed parameters.
     * 
     * @param parameter_map A key-value map containing parameter names and parameter values.
     */
    virtual void init(parameter_map_t &parameter_map) = 0;

    /**
     * @brief Calculate the Time-To-Collision between the Subject Object Motion and the Perceived Object Motion.
     * 
     * @param subject_object_motion Object Motion struct representing the Subject Object Motion.
     * @param perceived_object_motion Object Motion struct representing the Perceived Object Motion.
     * @return Optional that either contains a valid Time-To-Collision or has no valid content.
     */
    virtual boost::optional<double> calculateTTC(
        const object_motion_t &subject_object_motion,
        const object_motion_t &perceived_object_motion
    ) = 0; 
};

#endif // _TTC_ALGORITM_H_