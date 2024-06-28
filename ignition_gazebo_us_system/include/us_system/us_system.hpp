#include <ignition/gazebo/System.hh>

#include <us_system/us_sensor.hpp>

#include <ignition/transport/Node.hh>
#include <ignition/gazebo/Model.hh>
#include <gz/common/Image.hh>

#include "PlusConfigure.h"
//#include "igsioMath.h"
#include "PlusTrackedFrame.h"
#include "vtkAppendPolyData.h"
#include "vtkCubeSource.h"
#include "vtkImageData.h"
#include "vtkMatrix4x4.h"
#include "vtkPointData.h"
#include "vtkSTLWriter.h"
#include "vtkPlusSequenceIO.h"
#include "vtkSmartPointer.h"
#include "vtkTimerLog.h"
#include "vtkPlusTrackedFrameList.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkPlusTransformRepository.h"
#include "vtkPlusUsSimulatorAlgo.h"
#include "vtkXMLImageDataWriter.h"
#include "vtkXMLUtilities.h"
#include "vtksys/CommandLineArguments.hxx"

class us_system:
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate,
  public ignition::gazebo::ISystemPostUpdate {

private:

  vtkSmartPointer<vtkPlusTransformRepository> transformRepository;
  vtkSmartPointer<vtkPlusUsSimulatorAlgo> usSimulator;
  vtkSmartPointer<vtkPlusTrackedFrameList > trackedFrameList;
  vtkSmartPointer<vtkPlusTrackedFrameList> simulatedUltrasoundFrameList;
  vtkSmartPointer<vtkPlusConfig> plusConfig;

  std::unique_ptr<USSensor> sensor{nullptr};
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};

public:

  us_system();
  ~us_system();

  void Configure( const ignition::gazebo::Entity& entity,
		  const std::shared_ptr<const sdf::Element>& sdf,
		  ignition::gazebo::EntityComponentManager& ecm,
		  ignition::gazebo::EventManager& eventmgr ) override;

  void PreUpdate( const ignition::gazebo::UpdateInfo& info,
		  ignition::gazebo::EntityComponentManager& ecm ) override;
  void PostUpdate( const ignition::gazebo::UpdateInfo& info,
		   const ignition::gazebo::EntityComponentManager& ecm ) override;
  
};
