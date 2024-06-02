#ifndef _US_SENSOR_HPP_
#define _US_SENSOR_HPP_

#include <ignition/sensors/Sensor.hh>
#include <ignition/sensors/SensorTypes.hh>
#include <ignition/transport/Node.hh>

class USSensor : public ignition::sensors::Sensor {
  
private:
  
  /// \brief Node for communication
  ignition::transport::Node node;
  
  /// \brief Publishes sensor data
  ignition::transport::Node::Publisher pub;

public: gz::msgs::Image image;
public: std::string plusdir;
public: std::string plusconfig;

public:
  USSensor(){}

  /// \brief Load the sensor with SDF parameters.
  /// \param[in] _sdf SDF Sensor parameters.
  /// \return True if loading was successful
  virtual bool Load(const sdf::Sensor &_sdf) override;
  
  /// \brief Update the sensor and generate data
  /// \param[in] _now The current time
  /// \return True if the update was successfull
  virtual bool Update(const std::chrono::steady_clock::duration &_now) override;
  
};

#endif
