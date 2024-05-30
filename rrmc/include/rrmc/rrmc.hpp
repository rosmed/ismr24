#include <rclcpp/rclcpp.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

class rrmc : public rclcpp::Node {

private:
  
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_velocities;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_target;
  
  KDL::Chain chain;
  KDL::ChainIkSolverVel_wdls* ik_solver = {NULL};
  KDL::ChainFkSolverPos_recursive* fk_solver = {NULL};

  KDL::Vector linear_vel;
  bool active;
  
public:

  rrmc( const std::string& name );
  
  void ik_callback( const sensor_msgs::msg::JointState& js );
  void tgt_callback( const geometry_msgs::msg::Twist& vw );

};

