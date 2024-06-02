#include <math.h>

#include <ignition/msgs/double.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/sensors/Noise.hh>
#include <ignition/sensors/Util.hh>

#include <us_system/us_sensor.hpp>

bool USSensor::Load(const sdf::Sensor &_sdf){

  auto type = ignition::sensors::customType(_sdf);

  if ("USSensor" != type){
    ignerr << "Trying to load US sensor, but got type [" << type << "] instead." << std::endl;
    return false;
  }

  // Load common sensor params
  ignition::sensors::Sensor::Load(_sdf);
  
  if(!_sdf.Element()->HasElement("US")){
    ignerr << "No custom configuration for US sensor" << std::endl;
    return false;
  }

  // Advertise topic where data will be published
  this->pub = this->node.Advertise<ignition::msgs::Image>("image_raw");
  
  // Load custom sensor params
  auto customElem = _sdf.Element()->GetElement("US");
  
  if(!customElem->HasElement("PlusDir")){
    ignerr << "No PlusDir element" << std::endl;
    return false;
  }
  else{
    const std::string string = customElem->Get<std::string>("PlusDir");
    plusdir = string;
    igndbg << "PlusDir set to " << plusdir << std::endl;
  }
  
  if(!customElem->HasElement("PlusConfig")){
    ignerr << "No PlusConfig element" << std::endl;
    return false;
  }
  else{
    const std::string string = customElem->Get<std::string>("PlusConfig");
    plusconfig = string;
    igndbg << "PlusConfig " << plusconfig << std::endl;
  }

  return true;
}

//////////////////////////////////////////////////
bool USSensor::Update(const std::chrono::steady_clock::duration &/*_now*/){
  
  pub.Publish( image );

  return true;
}
