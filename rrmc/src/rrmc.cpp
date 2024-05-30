#include <rrmc/rrmc.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

rrmc::rrmc( const std::string& name ) :
  Node(name),
  active(false){

  pub_velocities = create_publisher<std_msgs::msg::Float64MultiArray>("velocities", 10);
  sub_js =
    create_subscription<sensor_msgs::msg::JointState>("joint_states",
						      10,
						      std::bind(&rrmc::ik_callback,
								this,
								std::placeholders::_1));
  sub_target =
    create_subscription<geometry_msgs::msg::Twist>("command",
						   10,
						   std::bind(&rrmc::tgt_callback,
							     this,
							     std::placeholders::_1));
  
  rclcpp::ParameterValue new_string = declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
  std::string robot_description = new_string.get<std::string>();
  
  KDL::Tree tree;
  if( kdl_parser::treeFromString( robot_description, tree ) ){
    if( tree.getChain( "base_link", "tool0", chain ) ){
      ik_solver = new KDL::ChainIkSolverVel_wdls( chain );
    }
    else{ std::cout << "Failed to parse chain" << std::endl; }
  }
  else{ std::cout << "Failed to parse tree" << std::endl; }
  
}

void rrmc::ik_callback( const sensor_msgs::msg::JointState& js ){

  if( ik_solver ){
    
    KDL::JntArray q_kdl( js.position.size() );
    for( std::size_t i=0; i<q_kdl.rows(); i++ )
      { q_kdl(i) = js.position[i]; }

    KDL::JntArray qd_kdl( q_kdl.rows() );
    KDL::Twist vw( linear_vel, KDL::Vector::Zero() );
    
    switch( ik_solver->CartToJnt( q_kdl, vw, qd_kdl ) ){
    case KDL::SolverI::E_SVD_FAILED:
      RCLCPP_ERROR(rclcpp::get_logger("rrmc::ik_callback"), " IK solver failed (SVD).");
      break;
    case KDL::ChainIkSolverVel_wdls::E_CONVERGE_PINV_SINGULAR:
      RCLCPP_ERROR(rclcpp::get_logger("rrmc::ik_callback"), " IK solver failed (singular).");
      break;
    case KDL::SolverI::E_NOERROR:
      std_msgs::msg::Float64MultiArray qd_msg;
      std_msgs::msg::MultiArrayDimension dim;
      dim.label = std::string("qd");
      dim.size = qd_kdl.rows();
      dim.stride = qd_kdl.rows();
      
      qd_msg.layout.dim.push_back(dim);
      for( unsigned int i=0; i<qd_kdl.rows(); i++ )
	qd_msg.data.push_back( qd_kdl(i) );
      
      if(active)
	pub_velocities->publish(qd_msg);
      
      break;
    }
    
  }
}

void rrmc::tgt_callback( const geometry_msgs::msg::Twist& vw ){
  linear_vel.x( vw.linear.x );
  linear_vel.y( vw.linear.y );
  linear_vel.z( vw.linear.z );
  active = true;
}
