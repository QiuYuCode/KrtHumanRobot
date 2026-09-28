#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <moveit/planning_scene/planning_scene.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace
{
std::string read_file(const char* path)
{
  std::ifstream input(path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

void check_pose(double shoulder_position)
{
  const auto urdf = urdf::parseURDF(read_file(MODEL_PATH));
  ASSERT_TRUE(urdf);
  auto srdf = std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf->initString(*urdf, read_file(SRDF_PATH)));
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  planning_scene::PlanningScene scene(model);
  auto& state = scene.getCurrentStateNonConst();
  for (const auto& name : model->getVariableNames())
    state.setVariablePosition(name, 0.0);
  state.setVariablePosition("left_arm_link2_joint", shoulder_position);
  state.setVariablePosition("right_arm_link2_joint", shoulder_position);
  state.update();
  collision_detection::CollisionRequest request;
  request.contacts = true;
  request.max_contacts = 100;
  collision_detection::CollisionResult result;
  // Empty group checks the whole robot, including the fixed head and chassis.
  scene.checkSelfCollision(request, result);
  std::stringstream contacts;
  for (const auto& pair : result.contacts)
    contacts << pair.first.first << " / " << pair.first.second << "; ";
  EXPECT_FALSE(result.collision) << contacts.str();
  collision_detection::AllowedCollision::Type type;
  const auto& matrix = scene.getAllowedCollisionMatrix();
  for (const auto& side : {"left", "right"})
  {
    const auto link = std::string(side) + "_arm_link5";
    const bool present = matrix.getEntry("base_link", link, type);
    EXPECT_FALSE(present && type == collision_detection::AllowedCollision::ALWAYS);
  }
}
}  // namespace

TEST(CollisionModel, ZeroGoalIsValidForWholeRobot)
{
  check_pose(0.0);
}

TEST(CollisionModel, HorizontalShoulderPoseIsValidForWholeRobot)
{
  check_pose(0.0);
}

TEST(CollisionModel, VerticalShoulderPoseIsValidForWholeRobot)
{
  check_pose(1.5707963267948966);
}
