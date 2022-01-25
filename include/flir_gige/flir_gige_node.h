#ifndef FLIR_GIGE_NODE_H_
#define FLIR_GIGE_NODE_H_

#include "flir_gige/flir_gige_ros.h"
#include "flir_gige/FlirGigeDynConfig.h"
#include "camera_base/camera_node_base.h"

namespace flir_gige {

class FlirGigeNode : public camera_base::CameraNodeBase<FlirGigeDynConfig> {
 public:
  FlirGigeNode()
      : CameraNodeBase(), flir_gige_ros_(*this) {}

  virtual void Acquire() override;
  virtual void Setup(FlirGigeDynConfig &config) override;

 private:
  FlirGigeRos flir_gige_ros_;
};

}  // namespace flir_gige

#endif  // FLIR_GIGE_NODE_H_
