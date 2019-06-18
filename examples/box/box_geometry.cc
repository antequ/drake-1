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
// CGS units
#define CGS
#ifdef CGS
const double LENGTH_SCALE = 0.3;
const double DENSITY = 1.07; /* lego ABS plastic */
#else
// m, kg, s
const double LENGTH_SCALE = 3.;
const double DENSITY = 1070.;
#endif

const BoxGeometry* BoxGeometry::AddToBuilder(
    systems::DiagramBuilder<double>* builder,
    const BoxPlant<double>& box, 
    const systems::OutputPort<double>& box_state_output_port,
    geometry::SceneGraph<double>* scene_graph, 
    std::string srcName) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  auto box_geometry = builder->AddSystem(
      std::unique_ptr<BoxGeometry>(
          new BoxGeometry(scene_graph, box, srcName)));
  // input is state, output is geometry
  builder->Connect(
      box_state_output_port,
      box_geometry->get_input_port(0));
  builder->Connect(
      box_geometry->get_output_port(0),
      scene_graph->get_source_pose_port(box_geometry->source_id_));

  return box_geometry;
}

BoxGeometry::BoxGeometry(geometry::SceneGraph<double>* scene_graph, const BoxPlant<double>& box, std::string srcName) {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  std::string boxname = "box" + srcName;
  source_id_ = scene_graph->RegisterSource( boxname );
  frame_id_ = scene_graph->RegisterFrame(source_id_, GeometryFrame("boxstate" + srcName));

  this->DeclareVectorInputPort("state", drake::systems::BasicVector<double>(2));
  this->DeclareAbstractOutputPort(
      "geometry_pose", &BoxGeometry::OutputGeometryPose);


  const double length = box.get_length();
  double volume = 1.0;
  if (box.get_inv_mass() != 0.) 
     volume = 1.0 / (box.get_inv_mass() * DENSITY);
  using std::sqrt;

  // The base.
  GeometryId id = scene_graph->RegisterGeometry(
      source_id_,frame_id_,
      make_unique<GeometryInstance>(Isometry3d(Translation3d(0., 0., 0.5 * LENGTH_SCALE * sqrt(volume / length))),
                                    make_unique<Box>(LENGTH_SCALE * length, 
                                    LENGTH_SCALE * sqrt(volume / length), 
                                    LENGTH_SCALE * sqrt(volume / length)), boxname));
  if( srcName != "2" )
    scene_graph->AssignRole(
      source_id_, id, MakePhongIllustrationProperties(Vector4d(.3, .6, .4, 1)));
  else
    scene_graph->AssignRole(
      source_id_, id, MakePhongIllustrationProperties(Vector4d(.4, .3, .6, 1)));
}

BoxGeometry::~BoxGeometry() = default;

void BoxGeometry::OutputGeometryPose(
    const systems::Context<double>& context,
    geometry::FramePoseVector<double>* poses) const {
  DRAKE_DEMAND(frame_id_.is_valid());

  const auto& input = get_input_port(0).Eval<drake::systems::BasicVector<double>>(context);
  // TODO: change this to a translation
  const double q = input[0] ;
  //std::cout << "Geom: " << frame_id_ << " " << q * LENGTH_SCALE << std::endl;
  math::RigidTransformd pose(Eigen::Vector3d(LENGTH_SCALE * q, 0.,0.));

  *poses = {{frame_id_, pose.GetAsIsometry3()}};
}

}  // namespace box
}  // namespace examples
}  // namespace drake
