#include <pluginlib/class_list_macros.h>

#include "ros_collision_detection/ttc_algorithm.h"
#include "ros_collision_detection/circle_algorithm.h"
#include "ros_collision_detection/n_circle_algorithm.h"

PLUGINLIB_EXPORT_CLASS(CircleAlgorithm, TTCAlgorithm)
PLUGINLIB_EXPORT_CLASS(NCircleAlgorithm, TTCAlgorithm)

