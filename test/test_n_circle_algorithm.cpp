// the class to be tested
#include "ros_collision_detection/n_circle_algorithm.h"

#include <gtest/gtest.h>


object_motion_t createObjectMotionHelper(float center_pos_x, float center_pos_y, float length, float width, float heading, float speed, float acceleration)
{
  object_motion_t result;
  result.center_pos_x = center_pos_x;
  result.center_pos_y = center_pos_y;
  result.length = length;
  result.width = width;
  result.heading = heading;
  result.speed = speed;
  result.acceleration = acceleration;

  return result;
}

NCircleAlgorithm createNCircleAlgorithmHelper(int n)
{
  NCircleAlgorithm nca;
  parameter_map_t param_map; 
  param_map.insert({"ttc_algorithm_circle_count", boost::variant<int, std::string>(n)});
  nca.init(param_map);
  return nca;
}

TEST(N1CircleAlgorithmAllZero, allZero)
{   
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(0, 0, 0, 0, 0, 0, 0);
  object_motion_t perceived_motion = createObjectMotionHelper(0, 0, 0, 0, 0, 0, 0);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);

  // ttc_optional is false if no ttc is delivered from calculateTTC
  EXPECT_FALSE(ttc_optional);
}

TEST(N1CircleAlgorithmWrongInput, negativeLengthXEgo)
{
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(1, 1, -2, 1, 1, 1, 1);
  object_motion_t perceived_motion = createObjectMotionHelper(3, 3, 3, 3, 3, 3, 3);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);

  // we expect a fail because length < 0
  // ttc_optional is false if no ttc is delivered from calculateTTC
  EXPECT_FALSE(ttc_optional); 
}


TEST(N1CircleAlgorithmWrongInput, negativeLengthYEgo)
{
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(1, 1, 1, -2, 1, 1, 1);
  object_motion_t perceived_motion = createObjectMotionHelper(3, 3, 3, 3, 3, 3, 3);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);

  // we expect a fail because width < 0
  // ttc_optional is false if no ttc is delivered from calculateTTC
  EXPECT_FALSE(ttc_optional); 
}

TEST(N1CircleAlgorithmWrongInput, negativeLengthXPerceived)
{
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(1, 1, 1, 1, 1, 1, 1);
  object_motion_t perceived_motion = createObjectMotionHelper(3, 3, -3, 3, 3, 3, 3);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);

  // we expect a fail because length < 0
  // ttc_optional is false if no ttc is delivered from calculateTTC
  EXPECT_FALSE(ttc_optional); 
}


TEST(N1CircleAlgorithmWrongInput, negativeLengthYPerceived)
{
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(1, 1, 1, 1, 1, 1, 1);
  object_motion_t perceived_motion = createObjectMotionHelper(3, 3, 3, -3, 3, 3, 3);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);

  // we expect a fail because width < 0
  // ttc_optional is false if no ttc is delivered from calculateTTC
  EXPECT_FALSE(ttc_optional); 
}

TEST(N1CircleAlgorithmWrongInput, negativeheadingEgo)
{
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(1, 1, 1, 1, -1, 1, 1);
  object_motion_t perceived_motion = createObjectMotionHelper(3, 3, 3, 3, 3, 3, 3);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);

  // we expect a fail because -1 < 0, minimum for heading
  // ttc_optional is false if no ttc is delivered from calculateTTC
  EXPECT_FALSE(ttc_optional); 
}


TEST(N1CircleAlgorithmWrongInput, tooBigHeadingEgo)
{
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(1, 1, 1, 1, 361, 1, 1);
  object_motion_t perceived_motion = createObjectMotionHelper(3, 3, 3, 3, 3, 3, 3);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);

  // we expect a fail because 361 > 360, maximum for heading
  // ttc_optional is false if no ttc is delivered from calculateTTC
  EXPECT_FALSE(ttc_optional); 
}

TEST(N1CircleAlgorithmWrongInput, negativeHeadingPerceived)
{
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(1, 1, 1, 1, 1, 1, 1);
  object_motion_t perceived_motion = createObjectMotionHelper(3, 3, 3, -1, 3, 3, 3);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);

  // we expect a fail because -1 < 0, minimum for heading
  // ttc_optional is false if no ttc is delivered from calculateTTC
  EXPECT_FALSE(ttc_optional); 
}


TEST(N1CircleAlgorithmWrongInput, tooBigHeadingPerceived)
{
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(1, 1, 1, 1, 1, 1, 1);
  object_motion_t perceived_motion = createObjectMotionHelper(3, 3, 3, 3, 361, 3, 3);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);

  // we expect a fail because 361 > 360, maximum for heading
  // ttc_optional is false if no ttc is delivered from calculateTTC
  EXPECT_FALSE(ttc_optional); 
}

TEST(N1CircleAlgorithmCorrectInput, validSolution4thDegreeComputation)
{
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(5, 5, 4.5, 2, 0, 10, 2);
  object_motion_t perceived_motion = createObjectMotionHelper(15, 15, 4.5, 2, 270, 20, 2);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);
  if(ttc_optional)
  {
    // we expect a valid ttc of about 0.4841820401185
    EXPECT_NEAR(0.4841820401185, *ttc_optional, 0.001);  
  }
  else
  {
    ADD_FAILURE() << "TTC does not match the expected 0.4841820401185";
  }
}

TEST(N1CircleAlgorithmCorrectInput, invalidComplexSolution4thDegreeComputation)
{
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(5, 5, 4.5, 2, 0, 10, 2);
  object_motion_t perceived_motion = createObjectMotionHelper(15, 15, 4.5, 2, 90, 20, 2);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);

  // we expect the calculation to fail because only complex roots are found
  // ttc_optional is false if no ttc is delivered from calculateTTC
  EXPECT_FALSE(ttc_optional);
}

/****************************************************************************************
 * no test cases for polynomial equation of 3rd degree
 * because it is impossible to get the prefactor of t^3 nonzero
 * when the prefactor of t^4 is zero
 ****************************************************************************************/

TEST(N1CircleAlgorithmCorrectInput, validSolution2ndDegreeComputation)
{
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(5, 5, 4.5, 2, 0, 10, 0);
  object_motion_t perceived_motion = createObjectMotionHelper(15, 15, 4.5, 2, 270, 20, 0);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);
  if(ttc_optional)
  {
    ROS_INFO("TTC IS VALID.");
    ROS_INFO("TTC IS: %f", *ttc_optional);
    // we expect a valid ttc of about 0.5078045554271
    EXPECT_NEAR(0.5078045554271, *ttc_optional, 0.001);  
  }
  else
  {
    ROS_INFO("TTC IS INVALID.");
    ADD_FAILURE() << "TTC does not match the expected 0.5078045554271";
  }
}

TEST(N1CircleAlgorithmCorrectInput, invalidComplexSolution2ndDegreeComputation)
{
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(5, 5, 4.5, 2, 0, 10, 0);
  object_motion_t perceived_motion = createObjectMotionHelper(15, 15, 4.5, 2, 90, 20, 0);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);

  // we expect the calculation to fail because only complex roots are found
  // ttc_optional is false if no ttc is delivered from calculateTTC
  EXPECT_FALSE(ttc_optional);
}

/****************************************************************************************
 * no test cases for polynomial equation of 1st degree
 * because it is impossible to get the prefactor of t^1 nonzero
 * when the prefactor of t^2 is zero
 ****************************************************************************************/

TEST(N1CircleAlgorithmCorrectInput, invalidComplexSolutionZeroDegreeComputation)
{
  NCircleAlgorithm nca = createNCircleAlgorithmHelper(1);
  object_motion_t ego_motion = createObjectMotionHelper(5, 5, 4.5, 2, 0, 0, 0);
  object_motion_t perceived_motion = createObjectMotionHelper(15, 15, 4.5, 2, 270, 0, 0);

  boost::optional<double> ttc_optional = nca.calculateTTC(ego_motion, perceived_motion);

  // we expect the calculation to fail because the highest polynomial degree is 0
  // --> no solution possible for t where P(t) = b_0 * t^0 = b_0 * 1
  // ttc_optional is false if no ttc is delivered from calculateTTC
  EXPECT_FALSE(ttc_optional);
}

// Run all tests
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  //ros::init(argc, argv, "tester_node");
  //ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}