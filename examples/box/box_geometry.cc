#include "drake/examples/box/box_geometry.h"

#include <memory>
#include <utility>

#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace box {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector4d;
using geometry::Box;
using geometry::Cylinder;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::MakePhongIllustrationProperties;
using geometry::Sphere;
using std::make_unique;
const double LENGTH_SCALE = 0.3;

const BoxGeometry* BoxGeometry::AddToBuilder(
    systems::DiagramBuilder<double>* builder,
    const systems::OutputPort<double>& box_state_port,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  auto box_geometry = builder->AddSystem(
      std::unique_ptr<BoxGeometry>(
          new BoxGeometry(scene_graph)));
  // input is state, output is geometry
  builder->Connect(
      box_state_port,
      box_geometry->get_input_port(0));
  builder->Connect(
      box_geometry->get_output_port(0),
      scene_graph->get_source_pose_port(box_geometry->source_id_));

  return box_geometry;
}

BoxGeometry::BoxGeometry(geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource("box");
  frame_id_ = scene_graph->RegisterFrame(source_id_, GeometryFrame("arm"));

  this->DeclareVectorInputPort("state", drake::systems::BasicVector<double>(2));
  this->DeclareAbstractOutputPort(
      "geometry_pose", &BoxGeometry::OutputGeometryPose);

  // TODO(jwnimmer-tri) This registration fails to reflect any non-default
  // parameters.  Ideally, it should happen in an Initialize event that
  // modifies the Context, or the output port should express the geometries
  // themselves instead of just their poses, or etc.
  // TODO: change this to just a box of size length.
  drake::systems::BasicVector<double> params (3);
  params[0] = 1.0;
  params[1] = 1.0;
  params[2] = 0.0;

  const double length = params[1];
  const double mass = params[0];

  // The base.
  GeometryId id = scene_graph->RegisterGeometry(
      source_id_,frame_id_,
      make_unique<GeometryInstance>(Isometry3d(Translation3d(0., 0., .025)),
                                    make_unique<Box>(LENGTH_SCALE * length, 
                                    LENGTH_SCALE * std::sqrt(mass / length), 
                                    LENGTH_SCALE * std::sqrt(mass / length)), "box"));
  scene_graph->AssignRole(
      source_id_, id, MakePhongIllustrationProperties(Vector4d(.3, .6, .4, 1)));
}

BoxGeometry::~BoxGeometry() = default;

void BoxGeometry::OutputGeometryPose(
    const systems::Context<double>& context,
    geometry::FramePoseVector<double>* poses) const {
  DRAKE_DEMAND(frame_id_.is_valid());

  const auto& input = get_input_port(0).Eval<drake::systems::BasicVector<double>>(context);
  // TODO: change this to a translation
  const double q = input[0];
  std::cout << "Geom: " << frame_id_ << " " << q * LENGTH_SCALE << std::endl;
  math::RigidTransformd pose(Eigen::Vector3d(LENGTH_SCALE * q, 0.,0.));

  *poses = {{frame_id_, pose.GetAsIsometry3()}};
}

}  // namespace box
}  // namespace examples
}  // namespace drake
