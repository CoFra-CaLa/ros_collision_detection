/**
 * @file collision_detection.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Implementation of the main function of the node and implementation of methods of Collision Detection class.
 * @version 0.1
 * @date 2022-04-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include <ros_collision_detection/collision_detection.h>

// definition of default values for launch parameters
#define DEFAULT_PUBLISH_TOPIC "/collision_warning"
#define DEFAULT_TTC_ALGORITHM_CLASSNAME "CircleAlgorithm"
#define DEFAULT_WARNING_GENERATOR_ALGORITHM_CLASSNAME "TTCOnlyWarningAlgorithm"
#define DEFAULT_TTC_ALGORITHM_CIRCLE_COUNT 1
#define DEFAULT_SUBJECT_VEHICLE_LENGTH 5
#define DEFAULT_SUBJECT_VEHICLE_WIDTH 1.8


CollisionDetection::CollisionDetection(ros::NodeHandle *nh)
:ttc_algorithm_loader("ros_collision_detection", "TTCAlgorithm"),
warning_generator_algorithm_loader("ros_collision_detection", "WarningGeneratorAlgorithm"),
approximate_synchronizer(ApproximateSyncPolicy(10), fused_objects_subscriber, ego_position_subscriber),
warning_generator(collision_warning_publisher)
{
    node_handle = nh;
    CollisionDetection::init();
}

void CollisionDetection::init()
{
    // initialize the components with launch parameter values
    initFromLaunchParameters();

    // register callback from ttc_calculator to warning_generator
    ttc_calculator.addWarningSignalCallback(boost::bind(&WarningGenerator::createWarning, &warning_generator, _1, _2, _3)); 
    
    //collision_warning_publisher = node_handle->advertise<ros_collision_detection::CollisionCheckResult>("/collision_warning", 10);
    fused_objects_subscriber.subscribe(*node_handle, "/fused_objects", 100);
    ego_position_subscriber.subscribe(*node_handle, "/ego_position", 100);
    approximate_synchronizer.registerCallback(boost::bind(&CollisionDetection::callback, this, _1, _2));
    
    // log successful init
    ROS_INFO("collision_detection node successfully initialized.");
}

void CollisionDetection::initFromLaunchParameters()
{
    // Check launch parameters and set defaults if necessary
    checkLaunchParameters();

    // load the concrete TTC Algorithm and Warning Generator classes
    loadPlugins();

    // initialize TTC Calculator and Warning Generator components with launch parameters
    initComponents();
}

void CollisionDetection::checkLaunchParameters()
{
    if(!node_handle->hasParam("publish_topic"))
    {
        // Default value for param "publish_topic" is DEFAULT_PUBLISH_TOPIC
        node_handle->setParam("publish_topic", DEFAULT_PUBLISH_TOPIC);
        ROS_WARN("CollisionDetection::loadPlugins: using default value for 'publish_topic'.");
    }

    if(!node_handle->hasParam("ttc_algorithm_classname"))
    {
        // Default value for param "ttc_algorithm_classname" is DEFAULT_TTC_ALGORITHM_CLASSNAME
        node_handle->setParam("ttc_algorithm_classname", DEFAULT_TTC_ALGORITHM_CLASSNAME);
        ROS_WARN("CollisionDetection::loadPlugins: using default value for 'ttc_algorithm_classname'.");
    }

    if(!node_handle->hasParam("warning_generator_algorithm_classname"))
    {
        // Default value for param "warning_generator_algorithm_classname" is DEFAULT_WARNING_GENERATOR_ALGORITHM_CLASSNAME
        node_handle->setParam("warning_generator_algorithm_classname", DEFAULT_WARNING_GENERATOR_ALGORITHM_CLASSNAME);
        ROS_WARN("CollisionDetection::loadPlugins: using default value for 'warning_generator_algorithm_classname'.");
    }

    if(!node_handle->hasParam("ttc_algorithm_circle_count"))
    {
        // Default value for param "ttc_algorithm_circle_count" is DEFAULT_TTC_ALGORITHM_CIRCLE_COUNT
        node_handle->setParam("ttc_algorithm_circle_count", DEFAULT_TTC_ALGORITHM_CIRCLE_COUNT);
        ROS_WARN("CollisionDetection::loadPlugins: using default value for 'ttc_algorithm_circle_count'.");
    }

    if(!node_handle->hasParam("subject_vehicle_length"))
    {
        // Default value for param "subject_vehicle_length" is DEFAULT_SUBJECT_VEHICLE_LENGTH
        node_handle->setParam("subject_vehicle_length", DEFAULT_SUBJECT_VEHICLE_LENGTH);
        ROS_WARN("CollisionDetection::loadPlugins: using default value for 'subject_vehicle_length'.");
    }

    if(!node_handle->hasParam("subject_vehicle_width"))
    {
        // Default value for param "subject_vehicle_width" is DEFAULT_SUBJECT_VEHICLE_WIDTH
        node_handle->setParam("subject_vehicle_width", DEFAULT_SUBJECT_VEHICLE_WIDTH);
        ROS_WARN("CollisionDetection::loadPlugins: using default value for 'subject_vehicle_width'.");
    }
}

void CollisionDetection::loadPlugins()
{
    std::string ttc_algorithm_classname; //!< The name of the TTC Algorithm class to load
    int ttc_algorithm_circle_count;      //!< The number of circles that should be used to represent a vehicle
    parameter_map_t param_map;           //!< The parameters that are passed to the init method of the TTC Algorithm

    if(node_handle->getParam("ttc_algorithm_classname", ttc_algorithm_classname))
    {
        if(node_handle->getParam("ttc_algorithm_circle_count", ttc_algorithm_circle_count))
        {
            param_map.insert({"ttc_algorithm_circle_count", boost::variant<int, std::string>(ttc_algorithm_circle_count)}); 

            try
            {
                boost::shared_ptr<TTCAlgorithm> ttc_algorithm_ptr = ttc_algorithm_loader.createInstance(ttc_algorithm_classname);
                ttc_algorithm_ptr->init(param_map); // init before shared pointer ownership changes
                ttc_calculator.setTTCAlgorithm(ttc_algorithm_ptr);
            }
            catch(pluginlib::PluginlibException& e)
            {
                ROS_FATAL("CollisionDetection::loadPlugins: cannot load TTC Algorithm plugin: %s", e.what());
                ros::shutdown();
                exit(0);
            }
        }
        else
        {
        ROS_FATAL("CollisionDetection::loadPlugins: parameter 'ttc_algorithm_circle_count' could not be retrieved.");
        ros::shutdown();
        exit(0);
        }
    }
    else
    {
        ROS_FATAL("CollisionDetection::loadPlugins: parameter 'ttc_algorithm_classname' could not be retrieved.");
        ros::shutdown();
        exit(0);
    }

    std::string warning_generator_algorithm_classname; //!< The name of the Warning Generator Algorithm class to load
    
    if(node_handle->getParam("warning_generator_algorithm_classname", warning_generator_algorithm_classname))
    {
        try
        {
            boost::shared_ptr<WarningGeneratorAlgorithm> warning_generator_algorithm_ptr = warning_generator_algorithm_loader.createInstance(warning_generator_algorithm_classname);
            warning_generator.setWarningGeneratorAlgorithm(warning_generator_algorithm_ptr);
        }
        catch(pluginlib::PluginlibException& e)
        {
            ROS_FATAL("CollisionDetection::loadPlugins: cannot load Warning Generator Algorithm plugin: %s", e.what());
            ros::shutdown();
            exit(0);
        }
    }
    else
    {
        ROS_FATAL("CollisionDetection::loadPlugins: parameter 'warning_generator_algorithm_classname' could not be retrieved.");
        ros::shutdown();
        exit(0);
    }
}

void CollisionDetection::initComponents()
{
    // set topic for collision warnings according to parameter
    std::string publish_topic_name;
    if(node_handle->getParam("publish_topic", publish_topic_name))
    {
        collision_warning_publisher = node_handle->advertise<ros_collision_detection::CollisionCheckResult>(publish_topic_name, 10);
    }
    else
    {
        ROS_FATAL("CollisionDetection::initComponents: parameter 'publish_topic' could not be retrieved.");
        ros::shutdown();
        exit(0);
    }

    float subject_vehicle_length;   //!< The length of the subject vehicle
    float subject_vehicle_width;    //!< The width of the subject vehicle

    bool subject_vehicle_length_retrieved = node_handle->getParam("subject_vehicle_length", subject_vehicle_length);
    bool subject_vehicle_width_retrieved = node_handle->getParam("subject_vehicle_width", subject_vehicle_width);

    if(!subject_vehicle_length_retrieved)
    {
        ROS_FATAL("CollisionDetection::initComponents: parameter 'subject_vehicle_length' could not be retrieved.");
        ros::shutdown();
        exit(0);
    }

    if(!subject_vehicle_width_retrieved)
    {
        ROS_FATAL("CollisionDetection::initComponents: parameter 'subject_vehicle_width' could not be retrieved.");
        ros::shutdown();
        exit(0);
    }

    // dimensions of subject vehicle successfully retrieved
    ttc_calculator.setSubjectVehicleDimensions(subject_vehicle_length, subject_vehicle_width);
}

void CollisionDetection::callback(const ros_collision_detection::PerceivedObjectsConstPtr& perceived_objects_msg, const ros_collision_detection::SubjectVehicleMotionConstPtr& subject_vehicle_motion_msg)
{
    // log seq number of the two messages
    ROS_DEBUG("CollisionDetection::callback: Subject vehicle msg: seq = %d | perceived object msg: seq = %d.", subject_vehicle_motion_msg->header.seq, perceived_objects_msg->header.seq);
    
    ttc_calculator.calculateAllTTCs(perceived_objects_msg, subject_vehicle_motion_msg);
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "collision_detecion",ros::init_options::AnonymousName); // create anonymous node if node if same name exists
    ros::NodeHandle nh;
    CollisionDetection collision_detection(&nh);

    ros::spin();
}
