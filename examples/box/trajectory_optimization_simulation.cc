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
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/solvers/solve.h"

using drake::solvers::SolutionResult;
using drake::systems::System;

namespace drake {
namespace examples {
namespace box {

using trajectories::PiecewisePolynomial;

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

DEFINE_double(box1_force_limit, 3.0,
              "Box1 force limit (N) ");

DEFINE_double(box2_final_position, 6.0,
              "Goal position for box 2 ");

DEFINE_double(input_cost, 10.0,
              "Input cost ");

DEFINE_double(box_d, 0.2,
              "box damping");

int DoMain() {
  systems::DiagramBuilder<double> builder;
  //auto source1 = builder.AddSystem<systems::Sine>(4 * M_PI * M_PI * 1.0 * 0. /* amplitude */, 
  //                                                2 * M_PI /* omega */, M_PI / 2.0 /* phase */, 1 /* vector size */);
  //source1->set_name("source1");
  //unused(source1);
  //systems::BasicVector<double> sourceValue(1);
  //sourceValue[0] = 0.;
  //auto source2 = builder.AddSystem<systems::ConstantVectorSource>( sourceValue );
  //source2->set_name("source2");
  auto box1 = builder.AddSystem<BoxPlant>(1.0 /* mass */, 1.0 /* length */, FLAGS_box_d);
  box1->set_name("box1");
  //builder.Connect(source1->get_output_port(0), box1->get_input_port());
  auto box2 = builder.AddSystem<BoxPlant>(1.0 /* mass */, 1.0 /* length */, FLAGS_box_d);
  box2->set_name("box2");
  //builder.Connect(source2->get_output_port(), box2->get_input_port());

  auto spring = builder.AddSystem<SpringPlant>(FLAGS_penalty_k /* k */, 
    FLAGS_penalty_d /* d */, 1. /* rest length */);
  // connect spring inputs
  builder.Connect(box1->get_output_port(), spring->get_first_box_input_port());
  builder.Connect(box2->get_output_port(), spring->get_second_box_input_port());
  // use spring's output in the positive for box 2
  auto adder2 = builder.AddSystem<systems::Adder>(1, 1); // (2,1 for 2 inputs)
  //builder.Connect(source2->get_output_port(), adder2->get_input_port(1));
  builder.Connect(spring->get_force_output_port(), adder2->get_input_port(0));
  builder.Connect(adder2->get_output_port(), box2->get_input_port());


  // use spring's output in the negative for box 1
  auto adder1 = builder.AddSystem<systems::Adder>(2, 1);
  auto negater = builder.AddSystem<systems::Gain>(double(-1.),1);
  builder.Connect(spring->get_force_output_port(), negater->get_input_port() );
  //builder.Connect(source1->get_output_port(0), adder1->get_input_port(0) );
  builder.Connect(negater->get_output_port(), adder1->get_input_port(1) );
  builder.Connect(adder1->get_output_port(), box1->get_input_port());

  
  builder.ExportInput(adder1->get_input_port(0)); // 1D force on box1
  //builder.ExportOutput(box2->get_output_port()); // box2 state
  
  auto diagram = builder.Build();

#if 0
  auto box1_ad = System<double>::ToAutoDiffXd(*box1);
  auto spring_ad = System<double>::ToAutoDiffXd(*spring);
  System<double>::ToAutoDiffXd(*negater);
  System<double>::ToAutoDiffXd(*adder1);
  System<double>::ToAutoDiffXd(*adder2);
  //System<double>::ToAutoDiffXd(*source1);
  //System<double>::ToAutoDiffXd(*source2);
  System<double>::ToAutoDiffXd(*box2);
  auto diagram_ad = System<double>::ToAutoDiffXd(*diagram);  
#endif

  auto context = diagram->CreateDefaultContext();
  const int kNumTimeSamples = 21;
  const double kMinimumTimeStep = 0.05;
  const double kMaximumTimeStep = 0.5;
  systems::trajectory_optimization::DirectCollocation dircol(diagram.get(),
      *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep,
      systems::InputPortSelection::kUseFirstInputIfItExists,
      false /* non cont states fixed */);
  
  dircol.AddEqualTimeIntervalsConstraints();

  const double kForceLimit = FLAGS_box1_force_limit;
  const solvers::VectorXDecisionVariable& u = dircol.input();
  // for now, input is a 1D force vector on box 1
  dircol.AddConstraintToAllKnotPoints(-kForceLimit <= u(0));
  dircol.AddConstraintToAllKnotPoints(u(0) <= kForceLimit);
  
  drake::VectorX<double> initState1(2);
  initState1 << -1. /* position */, FLAGS_box1_init_v /* velocity */;
  drake::VectorX<double> initState2(2);
  initState2 << 1. /* position */, 0. /* velocity */;
  systems::Context<double>& box1_context =
      diagram->GetMutableSubsystemContext(*box1,
                                          context.get()); 
  systems::Context<double>& box2_context =
      diagram->GetMutableSubsystemContext(*box2,
                                          context.get());
  box1->set_initial_state(&box1_context, initState1);
  box2->set_initial_state(&box2_context, initState2);

  drake::VectorX<double> finalState(4);
  finalState << FLAGS_box2_final_position /* b1 position */, FLAGS_box1_init_v /* b1 velocity */,
                FLAGS_box2_final_position /* b2 position */, 0. /* b2 vel */ ; 


  drake::VectorX<double> initState = context->get_continuous_state_vector().CopyToVector();
  DRAKE_DEMAND(context->num_continuous_states() == 4);
  std::cout << initState << std::endl;
  dircol.AddLinearConstraint(dircol.initial_state() ==
                             initState);
  dircol.AddLinearConstraint(dircol.final_state()(3) /* box 2 position */ ==
                             FLAGS_box2_final_position);

  const double R = FLAGS_input_cost;  // Cost on input "effort"
  dircol.AddRunningCost(((R * u) * u));
  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initState, finalState});
  dircol.SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);
  const auto result = solvers::Solve(dircol);
  if (!result.is_success()) {
    std::cerr << "Failed to solve optimization for the trajectory"
              << std::endl;
    return 1;
  }

  systems::DiagramBuilder<double> finalBuilder;
  const auto* boxes = finalBuilder.AddSystem(std::move(diagram));
  const PiecewisePolynomial<double> pp_traj =
      dircol.ReconstructInputTrajectory(result);
  auto input_trajectory = finalBuilder.AddSystem<systems::TrajectorySource>(pp_traj);
  input_trajectory->set_name("input trajectory");
  finalBuilder.Connect(input_trajectory->get_output_port(), boxes->get_input_port(0));

  auto scene_graph = finalBuilder.AddSystem<geometry::SceneGraph>();
  BoxGeometry::AddToBuilder(
      &finalBuilder, box1->get_state_output_port(), scene_graph, "1"); 
  BoxGeometry::AddToBuilder(
      &finalBuilder, box2->get_state_output_port(), scene_graph, "2");
  ConnectDrakeVisualizer(&finalBuilder, *scene_graph);
  auto finalDiagram = finalBuilder.Build();
  //const double initial_energy = box1->CalcTotalEnergy(box1_context)+box2->CalcTotalEnergy(box2_context);
  systems::Simulator<double> simulator(*finalDiagram);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(pp_traj.end_time());

  const double final_energy = box1->CalcTotalEnergy(box1_context)+box2->CalcTotalEnergy(box2_context);
  unused(final_energy);
  // Adds a numerical sanity test on total energy.
  //DRAKE_DEMAND(initial_energy + 0.5 * M_PI * M_PI + 0.5 >=  final_energy);

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
