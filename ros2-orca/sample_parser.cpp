class Attack {
public:
    double time_t;
    double x;
    double y;
    double z;
    int victim_robot_id;
};

this->declare_parameter<std::vector<int64_t>>("attack_parameters");

std::vector<int64_t> attack_parameters;
this->get_parameter("attack_parameters", attack_parameters);

for (size_t i = 0; i < attack_parameters.size(); i += 5)
{
    Attack a;
    a.time_t = attack_parameters[i];
    a.x = attack_parameters[i + 1];
    a.y = attack_parameters[i + 2];
    a.z = attack_parameters[i + 3];
    a.victim_robot_id = static_cast<int>(attack_parameters[i + 4]);
    RCLCPP_ERROR(this->get_logger(), "%f %f %f %f %d", a.time_t, a.x, a.y, a.z, a.victim_robot_id);
}