#include <gflags/gflags.h>

#include "drake/examples/box/box_geometry.h"
#include "drake/examples/box/box_plant.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/sine.h"

namespace drake {
namespace examples {
namespace box {
namespace {
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto source = builder.AddSystem<systems::Sine>(4 * M_PI * M_PI * 1.0 /* amplitude */, 
                                                  2 * M_PI /* omega */, M_PI / 2.0 /* phase */, 1 /* vector size */);
  source->set_name("source");
  auto box = builder.AddSystem<BoxPlant>();
  box->set_name("box");
  builder.Connect(source->get_output_port(0), box->get_input_port());
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  BoxGeometry::AddToBuilder(
      &builder, box->get_state_output_port(), scene_graph);
  ConnectDrakeVisualizer(&builder, *scene_graph);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& box_context =
      diagram->GetMutableSubsystemContext(*box,
                                          &simulator.get_mutable_context());
  drake::systems::BasicVector<double>& state = box->get_mutable_state(&box_context);
  state[0] = 1.; // position
  state[1] = 0.; // velocity

  const double initial_energy = box->CalcTotalEnergy(box_context);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(10);

  const double final_energy = box->CalcTotalEnergy(box_context);

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
