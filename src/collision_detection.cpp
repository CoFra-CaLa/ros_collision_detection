/**
 * @file collision_detection.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros_collision_detection/PerceivedObjects.h"
#include "ros_collision_detection/SubjectVehicleMotion.h"
#include <ros_collision_detection/collision_detection.h>

CollisionDetection::CollisionDetection(ros::NodeHandle *nh)
:time_snychronizer(fused_objects_subscriber, ego_position_subscriber, 10)
{
    node_handle = nh;
    //fused_objects_subscriber = node_handle->subscribe<ros_collision_detection::PerceivedObjects>("/fused_objects", 100, &CollisionDetection::callbackFusedObjects, this);
    //ego_position_subscriber = node_handle->subscribe<ros_collision_detection::SubjectVehicleMotion>("/ego_position", 100, &CollisionDetection::callbackEgoPosition, this);
    
    //message_filters::Subscriber<ros_collision_detection::PerceivedObjects> fused_objects_subscriber(*node_handle, "/fused_objects", 100);
    //message_filters::Subscriber<ros_collision_detection::SubjectVehicleMotion> ego_position_subscriber (*node_handle, "/ego_position", 100);
    //message_filters::TimeSynchronizer<ros_collision_detection::PerceivedObjects, ros_collision_detection::SubjectVehicleMotion> time_snychronizer(fused_objects_subscriber, ego_position_subscriber, 100);
    CollisionDetection::init();
}

void CollisionDetection::init()
{
    fused_objects_subscriber.subscribe(*node_handle, "/fused_objects", 100);
    ego_position_subscriber.subscribe(*node_handle, "/ego_position", 100);
    time_snychronizer.registerCallback(boost::bind(&CollisionDetection::callback, this, _1, _2));
}


CollisionDetection::~CollisionDetection()
{
    
}

void CollisionDetection::callback(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects_msg, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg)
{
    ROS_INFO("Seq: %d: x_pos: %f", perceived_objects_msg->header.seq, perceived_objects_msg->perceived_objects[0].object_movement.position.x);
    ROS_INFO("Seq: %d", subject_vehicle_motion_msg->header.seq);
}


void CollisionDetection::callbackFusedObjects(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects_msg) 
{
    //TODO
    uint32_t seq = perceived_objects_msg->header.seq;
    //TODO: do not define variables in callback function
    uint32_t id;
    std::string object_type;
    float x_length;
    float y_length;
    float heading;
    float speed;
    float acceleration;
    float x_position;
    float y_position;
    
    for (int i = 0; i < perceived_objects_msg->perceived_objects.size(); i++)
    {
        id = perceived_objects_msg->perceived_objects[i].object_movement.id;
        object_type = perceived_objects_msg->perceived_objects[i].object_type;
        x_length = perceived_objects_msg->perceived_objects[i].x_length;
        y_length = perceived_objects_msg->perceived_objects[i].y_length;
        heading = perceived_objects_msg->perceived_objects[i].object_movement.heading;
        speed = perceived_objects_msg->perceived_objects[i].object_movement.speed;
        acceleration = perceived_objects_msg->perceived_objects[i].object_movement.acceleration;
        x_position = perceived_objects_msg->perceived_objects[i].object_movement.position.x;
        y_position = perceived_objects_msg->perceived_objects[i].object_movement.position.y;

        ROS_INFO("Received 'PerceivedObjects' message with seq: %d, id: %d, type: %s, x_len: %f, y_len: %f, heading: %f, speed: %f, accel: %f, x_pos: %f, y_pos: %f", \
        seq, id, object_type.c_str(), x_length, y_length, heading, speed, acceleration, x_position, y_position);

    }
}

void CollisionDetection::callbackEgoPosition(const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg) 
{
    //TODO
    ROS_INFO("Received 'SubjectVehicleMotion' message");
}



int main (int argc, char **argv) 
{
    ros::init(argc, argv, "collision_detecion");
    ros::NodeHandle nh;
    CollisionDetection collision_detection (&nh);

    ros::spin();
}