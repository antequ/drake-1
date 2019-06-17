#include <gflags/gflags.h>

#include "drake/examples/box/box_geometry.h"
#include "drake/examples/box/box_plant.h"
#include "drake/examples/box/spring_plant.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/sine.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/gain.h"

namespace drake {
namespace examples {
namespace box {
namespace {
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto source1 = builder.AddSystem<systems::Sine>(4 * M_PI * M_PI * 1.0 /* amplitude */, 
                                                  2 * M_PI /* omega */, M_PI / 2.0 /* phase */, 1 /* vector size */);
  source1->set_name("source1");
  systems::BasicVector<double> sourceValue(1);
  sourceValue[0] = 0.;
  auto source2 = builder.AddSystem<systems::ConstantVectorSource>( sourceValue );
  source2->set_name("source2");
  auto box1 = builder.AddSystem<BoxPlant>();
  box1->set_name("box1");
  //builder.Connect(source1->get_output_port(0), box1->get_input_port());
  auto box2 = builder.AddSystem<BoxPlant>();
  box2->set_name("box2");
  //builder.Connect(source2->get_output_port(), box2->get_input_port());

  auto spring = builder.AddSystem<SpringPlant>(10. /* k */, 0.5 /* d */, 1. /* rest length */);
  // connect spring inputs
  builder.Connect(box1->get_output_port(), spring->get_first_box_input_port());
  builder.Connect(box2->get_output_port(), spring->get_second_box_input_port());
  // use spring's output in the positive for box 2
  auto adder2 = builder.AddSystem<systems::Adder>(2, 1);
  builder.Connect(source2->get_output_port(), adder2->get_input_port(0));
  builder.Connect(spring->get_output_port(), adder2->get_input_port(1));
  builder.Connect(adder2->get_output_port(), box2->get_input_port());
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();

  // use spring's output in the negative for box 1
  auto adder1 = builder.AddSystem<systems::Adder>(2, 1);
  auto negater = builder.AddSystem<systems::Gain>(double(-1.),1);
  builder.Connect(spring->get_output_port(), negater->get_input_port() );
  builder.Connect(source1->get_output_port(0), adder1->get_input_port(0) );
  builder.Connect(negater->get_output_port(), adder1->get_input_port(1) );
  builder.Connect(adder1->get_output_port(), box1->get_input_port());
  BoxGeometry::AddToBuilder(
      &builder, box1->get_state_output_port(), scene_graph, "1"); 
  BoxGeometry::AddToBuilder(
      &builder, box2->get_state_output_port(), scene_graph, "2");


  ConnectDrakeVisualizer(&builder, *scene_graph);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& box1_context =
      diagram->GetMutableSubsystemContext(*box1,
                                          &simulator.get_mutable_context()); 
  systems::Context<double>& box2_context =
      diagram->GetMutableSubsystemContext(*box2,
                                          &simulator.get_mutable_context());
  
  
  drake::VectorX<double> initState1(2);
  initState1 << -1. /* position */, 0. /* velocity */;
  drake::VectorX<double> initState2(2);
  initState2 << 1. /* position */, 0. /* velocity */;
  box1->set_initial_state(&box1_context, initState1);
  box2->set_initial_state(&box2_context, initState2);

  const double initial_energy = box1->CalcTotalEnergy(box1_context)+box2->CalcTotalEnergy(box2_context);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(10);

  const double final_energy = box1->CalcTotalEnergy(box1_context)+box2->CalcTotalEnergy(box2_context);

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
