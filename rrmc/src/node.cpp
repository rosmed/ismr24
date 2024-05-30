#include <rrmc/rrmc.hpp>

int main( int argc, char** argv ){

  rclcpp::init(argc,argv);
  rclcpp::spin( std::make_shared<rrmc>("rrmc") );
  rclcpp::shutdown();

  return 0;
}
