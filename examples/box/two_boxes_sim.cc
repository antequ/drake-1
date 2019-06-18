#include <gflags/gflags.h>

#include "drake/examples/box/box_geometry.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/sine.h"
#include "drake/examples/box/two_boxes_plant.h"
#include "drake/systems/primitives/signal_logger.h"

#include <fstream>

namespace drake {
namespace examples {
namespace box {
namespace {
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(penalty_k, 20.0,
              "k stiffness constant for contact penalty force");

DEFINE_double(penalty_d, 0.0,
              "d damping constant for contact penalty force");

DEFINE_double(box1_init_v, 2.0,
              "initial velocity for box 1");

DEFINE_double(box_d, 0.2,
              "box damping");

int DoMain() {
  systems::DiagramBuilder<double> builder;

  auto source1 = builder.AddSystem<systems::Sine>(4 * M_PI * M_PI * 1.0 * 0. /* amplitude */, 
                                                  2 * M_PI /* omega */, M_PI / 2.0 /* phase */, 1 /* vector size */);
  source1->set_name("source1");

  auto two_boxes = builder.AddSystem<TwoBoxesPlant>(1.0 /* mass */,
      FLAGS_box_d, 1.0 /* length */, FLAGS_penalty_k, FLAGS_penalty_d);

      // ADD AND CONNECT LOGGER TO SYSTEM
  builder.Connect(source1->get_output_port(0), two_boxes->get_input_port(0));
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  AddGeometryToBuilder(&builder, *two_boxes, scene_graph);
  ConnectDrakeVisualizer(&builder, *scene_graph);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& context = diagram->GetMutableSubsystemContext(*two_boxes,
                                          &simulator.get_mutable_context());
  
  
  drake::VectorX<double> initState1(2);
  initState1 << -1. /* position */, FLAGS_box1_init_v /* velocity */;
  drake::VectorX<double> initState2(2);
  initState2 << 1. /* position */, 0. /* velocity */;
  two_boxes->SetBox1State(&context, initState1);
  two_boxes->SetBox2State(&context, initState2);

  const double initial_energy = two_boxes->CalcBoxesTotalEnergy(context);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(10);

  const double final_energy = two_boxes->CalcBoxesTotalEnergy(context);

  // Adds a numerical sanity test on total energy.
  DRAKE_DEMAND(initial_energy + 0.5 * M_PI * M_PI + 0.5 >=  final_energy);
  return 0;
}

}  // namespace
}  // namespace box
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::box::DoMain();
}
