#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace rendering {

/// PoseBundleToDrawMessage converts a PoseBundle on its single abstract-valued
/// input port to a Drake Visualizer Interface LCM draw message,
/// lcmt_viewer_draw, on its single abstract-valued output port.
///
/// The draw message will contain one link for each pose in the PoseBundle. The
/// name of the link will be the name of the corresponding pose. The robot_num
/// will be the corresponding model instance ID.
template<typename T>
class PoseBundleToDrawMessageTemplated : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseBundleToDrawMessageTemplated);

  PoseBundleToDrawMessageTemplated();

  template <typename U>
  explicit PoseBundleToDrawMessageTemplated(const PoseBundleToDrawMessageTemplated<U>&) :
      PoseBundleToDrawMessageTemplated() {}

  ~PoseBundleToDrawMessageTemplated() override;

 private:
  // Copies the input poses into the draw message.
  void CalcViewerDrawMessage(const Context<T>& context,
                             lcmt_viewer_draw* output) const;
};

typedef PoseBundleToDrawMessageTemplated<double> PoseBundleToDrawMessage;
}  // namespace rendering
}  // namespace systems
}  // namespace drake
