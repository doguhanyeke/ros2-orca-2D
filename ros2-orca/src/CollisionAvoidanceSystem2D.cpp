#include "CollisionAvoidanceSystem2D.h"
#include <RVO.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

CollisionAvoidanceSystem::CollisionAvoidanceSystem(const std::string &node_name)
    : Node(node_name) {
  // TODO: add one more constructor to give obstacles 
  // sim = new RVO::RVOSimulator();
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.history=RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos_profile.depth=1;
  qos_profile.reliability=RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  qos_profile.durability=RMW_QOS_POLICY_DURABILITY_VOLATILE;
  auto qos_ = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
  
  // std::string config_path2;
  // this->get_parameter("config_path", config_path2);
  // RCLCPP_ERROR(this->get_logger(), "%s", config_path2.c_str());
  // auto config = parseConfigFile(config_path2);
  // Accessing the parsed values
  // int _numRobots = std::stoi(config["num_robots"]);
  // double _timeStep = std::stod(config["time_step"]);
  // double _radius = std::stod(config["radius"]);
  // double _neighborDist = std::stod(config["neighborDist"]);
  // int _maxNeighbors = std::stoi(config["maxNeighbors"]);
  // double _timeHorizon = std::stod(config["timeHorizon"]);
  // double _timeHorizonObst = std::stod(config["timeHorizonObst"]);
  // double _maxSpeed = std::stod(config["maxSpeed"]);

  this->declare_parameter("drone_id", 0);
  this->get_parameter("drone_id", this->agentID);
  RCLCPP_ERROR(this->get_logger(),"robot id %d", this->agentID);

  this->declare_parameter<std::vector<double>>("obstacle_positions");
  std::vector<double> obstacle_positions_flat;
  this->get_parameter("obstacle_positions", obstacle_positions_flat);
  for (size_t i = 0; i < obstacle_positions_flat.size(); i += 2)
  {
    std::vector<double> obstacle;
    obstacle.push_back(obstacle_positions_flat[i]);
    obstacle.push_back(obstacle_positions_flat[i + 1]);
    this->obstacle_positions.push_back(obstacle);
    RCLCPP_ERROR(this->get_logger(),"%f %f", obstacle_positions_flat[i], obstacle_positions_flat[i+1]);
  }

  this->declare_parameter<std::vector<double>>("goal_positions");
  std::vector<double> goal_positions_flat;
  this->get_parameter("goal_positions", goal_positions_flat);
  for (size_t i = 0; i < goal_positions_flat.size(); i += 2)
  {
    std::vector<double> agent_goal;
    agent_goal.push_back(goal_positions_flat[i]);
    agent_goal.push_back(goal_positions_flat[i + 1]);
    this->goal_positions.push_back(agent_goal);
    RCLCPP_ERROR(this->get_logger(),"%f %f", goal_positions_flat[i], goal_positions_flat[i+1]);
  }

  this->declare_parameter<std::vector<double>>("robot_initial_positions");
  std::vector<double> robot_initial_positions_flat;
  this->get_parameter("robot_initial_positions", robot_initial_positions_flat);
  for (size_t i = 0; i < robot_initial_positions_flat.size(); i += 2)
  {
    std::vector<double> agent_pos;
    agent_pos.push_back(robot_initial_positions_flat[i]);
    agent_pos.push_back(robot_initial_positions_flat[i + 1]);
    this->robot_initial_positions.push_back(agent_pos);
    RCLCPP_ERROR(this->get_logger(),"%f %f", robot_initial_positions_flat[i], robot_initial_positions_flat[i+1]);
  }

  this->declare_parameter("time_step", 0.0);
  this->get_parameter("time_step", this->time_step);
  RCLCPP_ERROR(this->get_logger(),"time_step %f", this->time_step);

  this->declare_parameter("num_robots", 0);
  this->get_parameter("num_robots", this->num_robots);
  RCLCPP_ERROR(this->get_logger(),"num_robots %d", this->num_robots);

  this->declare_parameter("radius", 0.0);
  this->get_parameter("radius", this->radius);
  RCLCPP_ERROR(this->get_logger(),"radius %f", this->radius);
  
  this->declare_parameter("neighborDist", 0.0);
  this->get_parameter("neighborDist", this->neighborDist);
  RCLCPP_ERROR(this->get_logger(),"neighborDist %f", this->neighborDist);

  this->declare_parameter("maxNeighbors", 0);
  this->get_parameter("maxNeighbors", this->maxNeighbors);
  RCLCPP_ERROR(this->get_logger(),"maxNeighbors %d", this->maxNeighbors);

  this->declare_parameter("timeHorizon", 0.0);
  this->get_parameter("timeHorizon", this->timeHorizon);
  RCLCPP_ERROR(this->get_logger(),"timeHorizon %f", this->timeHorizon);

  this->declare_parameter("timeHorizonObst", 0.0);
  this->get_parameter("timeHorizonObst", this->timeHorizonObst);
  RCLCPP_ERROR(this->get_logger(),"timeHorizonObst %f", this->timeHorizonObst);

  this->declare_parameter("maxSpeed", 0.0);
  this->get_parameter("maxSpeed", this->maxSpeed);
  RCLCPP_ERROR(this->get_logger(),"maxSpeed %f", this->maxSpeed);

  sim = new RVO::RVOSimulator(this->time_step, this->neighborDist, this->maxNeighbors, this->timeHorizon, this->timeHorizonObst, this->radius, this->maxSpeed, RVO::Vector2(0.0, 0.0));


  // Initialize ROS 2 publishers and subscribers
  /*
  for(int i=0; i<_numRobots; i++){
    vehicle_local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "px4_1/fmu/out/vehicle_local_position",qos_,
      std::bind(&CollisionAvoidanceSystem::vehicle_local_pos_callback, this,
                std::placeholders::_1));
  }
  trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "px4_1/fmu/orca2/trajectory_setpoint", 10);
  */
  // Initialize ROS 2 publishers and subscribers
  // RCLCPP_ERROR(this->get_logger(),"number of robots: %d", _numRobots);
  for(int i = 0; i < this->num_robots; i++) {
    std::string vehicle_local_pos_topic = "px4_" + std::to_string(i + 1) + "/fmu/out/vehicle_local_position";
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_pos_sub_;
    vehicle_local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    vehicle_local_pos_topic,
    qos_,
    [this, i](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        this->vehicle_local_pos_callback(msg, i + 1);
    });
    vehicle_local_pos_sub_vec.push_back(vehicle_local_pos_sub_);
  }

  std::string trajectory_setpoint_topic = "px4_" + std::to_string(this->agentID) + "/fmu/orca2/trajectory_setpoint";
  trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(trajectory_setpoint_topic, 10);
  
  this->time_t = 0.0;
  // this->time_step = _timeStep;
  // this->radius = _radius;
  // this->num_robots = _numRobots;
  // this->neighborDist = _neighborDist;
  // this->maxNeighbors = _maxNeighbors;
  // this->timeHorizon = _timeHorizon;
  // this->timeHorizonObst = _timeHorizonObst;
  // this->maxSpeed = _maxSpeed;

  this->goals = std::vector<RVO::Vector2>();
  this->setupScenario();
}

CollisionAvoidanceSystem::~CollisionAvoidanceSystem() { delete sim; }

void CollisionAvoidanceSystem::vehicle_local_pos_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg, int robot_id) {
    // Handle incoming messages here
    // set new positions via set_agent_positions
    //RCLCPP_ERROR(this->get_logger(),"I am here for robot %d", robot_id);

    RVO::Vector2 new_position = RVO::Vector2(msg->y + this->robot_initial_positions[robot_id-1][0], msg->x + this->robot_initial_positions[robot_id-1][1]);
    this->set_agent_position(RVO::Vector2(new_position.x(), new_position.y()), robot_id-1);
    RVO::Vector2 new_velocity = RVO::Vector2(msg->vy, msg->vx);
    this->set_agent_velocity(RVO::Vector2(new_velocity.x(), new_velocity.y()), robot_id-1);
    this->calculate_next_positions();
    std::vector<RVO::Vector2> agent_positions = this->get_agent_positions();

    // RCLCPP_ERROR(this->get_logger()," Robot %d Position %f %f %f", robot_id, new_position.x(), new_position.y(), new_position.z());
    
    // RCLCPP_ERROR(this->get_logger()," Robot %d  Velocity %f %f %f", robot_id,  new_velocity.x(), new_velocity.y(), new_velocity.z());
    
    // RCLCPP_ERROR(this->get_logger()," Robot %d  ORCA Velocity %f %f %f", robot_id,  this->sim->getAgentVelocity(this->agentID-1).x(), this->sim->getAgentVelocity(this->agentID-1).y(), this->sim->getAgentVelocity(this->agentID-1).z());

    auto trajectory_setpoint =  px4_msgs::msg::TrajectorySetpoint();
    // trajectory_setpoint.position[1] = agent_positions[this->agentID - 1].y() + 10.0;
    // trajectory_setpoint.position[0] = agent_positions[this->agentID - 1].x() + 5.0;
    // trajectory_setpoint.position[2] = -agent_positions[this->agentID - 1].z() - 5.0;
    
    trajectory_setpoint.velocity[1] = this->sim->getAgentVelocity(this->agentID-1).x();
    trajectory_setpoint.velocity[0] = this->sim->getAgentVelocity(this->agentID-1).y();
    trajectory_setpoint.velocity[2] = -0.5;


    this->trajectory_setpoint_pub_->publish(trajectory_setpoint);
}

void CollisionAvoidanceSystem::set_initial_positions_from_yaml(){
  for(auto drone_position: this->robot_initial_positions){
    sim->addAgent(RVO::Vector2(drone_position[0], drone_position[1]));
  }
}

void CollisionAvoidanceSystem::set_initial_positions_from_file(
  std::string file_path) {
  // open file for read
  std::ifstream input_file(file_path);
  // Check if the file is opened successfully
  if (input_file.is_open()) {
    std::string line;

    // Read the file line by line
    while (std::getline(input_file, line)) {
      std::istringstream line_stream(line);
      double x;
      double y;
      char delimiter;

      // Read the values separated by a comma
      // std::shared_ptr<std::vector<Eigen::Vector2d>>
      if (line_stream >> x >> delimiter >> y) {
        sim->addAgent(RVO::Vector2(x, y));
      } else {
        std::cerr << "Error reading line in orca2: " << line << std::endl;
      }
    }

    // Close the file
    input_file.close();
  } else {
    std::cerr << "Unable to open file: " << file_path << std::endl;
  }

  return;
}

void CollisionAvoidanceSystem::set_robot_goal_positions_from_yaml() {
  for(auto drone_goal: this->goal_positions){
    this->goals.push_back(RVO::Vector2(drone_goal[0], drone_goal[1]));
  }
}

void CollisionAvoidanceSystem::set_robot_goal_positions(std::string file_path) {
  // open file for read
  std::ifstream input_file(file_path);
  // Check if the file is opened successfully
  if (input_file.is_open()) {
    std::string line;

    // Read the file line by line
    while (std::getline(input_file, line)) {
      std::istringstream line_stream(line);
      double x, y;
      char delimiter;

      // Read the values separated by a comma
      // std::shared_ptr<std::vector<Eigen::Vector2d>>
      if (line_stream >> x >> delimiter >> y) {
        this->goals.push_back(RVO::Vector2(x, y));
      } else {
        std::cerr << "Error reading line in orca2: " << line << std::endl;
      }
    }

    // Close the file
    input_file.close();
  } else {
    std::cerr << "Unable to open file: " << file_path << std::endl;
  }

  return;
}

std::map<std::string, std::string> CollisionAvoidanceSystem::parseConfigFile(
    const std::string &filename) {
  std::map<std::string, std::string> config;
  std::ifstream configFile(filename);
  std::string line;

  if (!configFile.is_open()) {
    throw std::runtime_error("Unable to open config file: " + filename);
  }

  while (getline(configFile, line)) {
    std::istringstream is_line(line);
    std::string key, value;
    if (std::getline(is_line, key, '=') && std::getline(is_line, value)) {
      // Remove potential quotation marks from value
      value.erase(std::remove(value.begin(), value.end(), '\"'), value.end());
      config[key] = value;
    }
  }

  return config;
}

void CollisionAvoidanceSystem::set_obstacle_positions_from_yaml() {
    // Each obstacle has 4 corners
    const size_t corners_per_obstacle = 4;
    
    // Process points in groups of 4 to create multiple obstacles
    for (size_t i = 0; i < this->obstacle_positions.size(); i += corners_per_obstacle) {
        std::vector<RVO::Vector2> obstacle;
        
        // Add 4 corners for this obstacle
        for (size_t j = 0; j < corners_per_obstacle && (i + j) < this->obstacle_positions.size(); ++j) {
            obstacle.push_back(RVO::Vector2(
                this->obstacle_positions[i + j][0],
                this->obstacle_positions[i + j][1]
            ));
        }
        
        // Only add complete obstacles (with 4 corners)
        if (obstacle.size() == corners_per_obstacle) {
            sim->addObstacle(obstacle);
        }
    }
    
    // Process all obstacles after adding them
    sim->processObstacles();
}

void CollisionAvoidanceSystem::setupScenario() {
  sim->setTimeStep(time_step);
  //sim->setAgentDefaults(neighborDist, maxNeighbors, timeHorizon, radius, maxSpeed);

  // std::string robot_positions_path;
  // this->get_parameter("robot_initial_positions_path", robot_positions_path);
  // set_initial_positions_from_file(robot_positions_path);
  this->set_initial_positions_from_yaml();

  // std::string goals_path;
  // this->get_parameter("goals_path", goals_path);
  // set_robot_goal_positions(goals_path);
  this->set_robot_goal_positions_from_yaml();
  this->set_obstacle_positions_from_yaml();
}

void CollisionAvoidanceSystem::setPreferredVelocities() {
  for (std::size_t i = 0U; i < sim->getNumAgents(); ++i) {
    RVO::Vector2 goalVector = this->goals[i] - sim->getAgentPosition(i);
    if (RVO::absSq(goalVector) > 1.0F) {
      goalVector = RVO::normalize(goalVector);
    }
    sim->setAgentPrefVelocity(i, goalVector);
  }
}

bool CollisionAvoidanceSystem::reachedGoal() {
  for (std::size_t i = 0U; i < sim->getNumAgents(); ++i) {
    if (RVO::absSq(sim->getAgentPosition(i) - this->goals[i]) >
        5.0F * sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
      return false;
    }
  }
  return true;
}

void CollisionAvoidanceSystem::set_agent_positions(
    const std::vector<RVO::Vector2> &positions) {
  for (std::size_t i = 0; i < positions.size(); ++i) {
    // Directly use positions[i] since it's already RVO::Vector3
    sim->setAgentPosition(i, positions[i]);
  }
}

void CollisionAvoidanceSystem::set_agent_position(
    const RVO::Vector2 &position, int id) {
  sim->setAgentPosition(id, position);
}

void CollisionAvoidanceSystem::set_agent_velocity(
    const RVO::Vector2 &velocity, int id) {
  sim->setAgentVelocity(id, velocity);
}

void CollisionAvoidanceSystem::set_agent_velocities(
    const std::vector<RVO::Vector2> &velocities) {
  for (std::size_t i = 0; i < velocities.size(); ++i) {
    // Directly use velocities[i] since it's already RVO::Vector3
    sim->setAgentVelocity(i, velocities[i]);
  }
}

std::vector<RVO::Vector2> CollisionAvoidanceSystem::get_agent_positions() {
  std::vector<RVO::Vector2> positions;
  for (std::size_t i = 0; i < sim->getNumAgents(); ++i) {
    positions.push_back(sim->getAgentPosition(i));
  }
  return positions;
}

std::vector<RVO::Vector2> CollisionAvoidanceSystem::get_agent_velocities() {
  std::vector<RVO::Vector2> velocities;
  for (std::size_t i = 0; i < sim->getNumAgents(); ++i) {
    velocities.push_back(sim->getAgentVelocity(i));
  }
  return velocities;
}

void CollisionAvoidanceSystem::updateVisualization() {
  /* Output the current global time. */
  std::cout << sim->getGlobalTime();

  /* Output the position for all the agents. */
  for (std::size_t i = 0U; i < sim->getNumAgents(); ++i) {
    std::cout << " " << sim->getAgentPosition(i);
  }

  std::cout << std::endl;
}

void CollisionAvoidanceSystem::calculate_next_positions() {
  updateVisualization();
  setPreferredVelocities();
  sim->doStep();
}

void CollisionAvoidanceSystem::calculate_next_positions(
    int attackedRobotId, RVO::Vector2 attackedRobotPosition) {
  updateVisualization();
  setPreferredVelocities();
  sim->doStep(attackedRobotId, attackedRobotPosition);
}

// print initial positions
void CollisionAvoidanceSystem::print_initial_positions() {
  for (std::size_t i = 0U; i < sim->getNumAgents(); ++i) {
    std::cout << " " << sim->getAgentPosition(i);
  }
  std::cout << std::endl;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<CollisionAvoidanceSystem>("collision_avoidance_system");
  rclcpp::spin(node);

  // set new positions via set_agent_positions
  /*
  std::vector<RVO::Vector3> new_positions;
  new_positions.push_back(RVO::Vector3(0.0, 0.0, 5.0));
  new_positions.push_back(RVO::Vector3(0.0, 0.0, 5.0));
  new_positions.push_back(RVO::Vector3(0.0, 0.0, 5.0));
  system->set_agent_positions(new_positions);
  */

  // auto counter = 0;
  // while (!node->reachedGoal()) {
  //   // system->calculate_next_positions();
  //   node->calculate_next_positions(1, node->get_agent_positions()[1]);
  //   node->updateVisualization();
  //   counter++;
  //   //rclcpp::spin(node);
  // }

  rclcpp::shutdown();
  return 0;
}
