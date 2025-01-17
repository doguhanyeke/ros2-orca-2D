#include <RVO.h>

#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"


#include <vector>

class CollisionAvoidanceSystem : public rclcpp::Node {
 private:
  RVO::RVOSimulator *sim;
  std::vector<RVO::Vector2> goals;

  std::string log_file;
  std::string environment_file;
  float time_step;
  float time_t;
  int num_robots;
  float radius;
  int deadlockOccurs;
  float deadlock_threshold;
  std::string benign_output;
  std::string metric_output_file;
  int maxCycle;
  float neighborDist;
  int maxNeighbors;
  float timeHorizon;
  float timeHorizonObst;
  float maxSpeed;
  int agentID;

  std::vector<std::vector<double>> obstacle_positions;
  std::vector<std::vector<double>> goal_positions;
  std::vector<std::vector<double>> robot_initial_positions;

  // ROS 2 Publishers and Subscribers
  // rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_pos_sub_;
  std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr> vehicle_local_pos_sub_vec;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;


 public:
  CollisionAvoidanceSystem(const std::string &node_name);
  ~CollisionAvoidanceSystem();

  // Member functions
  void set_initial_positions_from_file(std::string file_path);
  void set_robot_goal_positions(std::string file_path);
  std::map<std::string, std::string> parseConfigFile(
      const std::string &filename);
  void setupScenario();
  void setPreferredVelocities();
  bool reachedGoal();
  void set_agent_positions(const std::vector<RVO::Vector2> &positions);
  void set_agent_position(const RVO::Vector2 &position, int id);
  void set_agent_velocities(const std::vector<RVO::Vector2> &velocities);
  void set_agent_velocity(const RVO::Vector2 &velocity, int id);

  std::vector<RVO::Vector2> get_agent_positions();
  std::vector<RVO::Vector2> get_agent_velocities();
  void updateVisualization();
  void calculate_next_positions();
  void calculate_next_positions(int attackedRobotId,
                                RVO::Vector2 attackedRobotPosition);
  void print_initial_positions();

  void set_initial_positions_from_yaml();
  void set_robot_goal_positions_from_yaml();
  void set_obstacle_positions_from_yaml();

  void vehicle_local_pos_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg, int robot_id);
};
