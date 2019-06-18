#include <gflags/gflags.h>

#include "drake/examples/box/box_geometry.h"
#include "drake/examples/box/two_boxes_plant.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/sine.h"
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

DEFINE_double(box2_final_position, 3.0,
              "Goal position for box 2 ");

DEFINE_double(input_cost, 10.0,
              "Input cost ");

DEFINE_double(box_d, 0.2,
              "box damping");

int DoMain() {
  auto two_boxes = std::make_unique<TwoBoxesPlant<double>>(1.0 /* mass */,
      FLAGS_box_d, 1.0 /* length */, FLAGS_penalty_k, FLAGS_penalty_d);
  two_boxes->set_name("twoboxes");
  auto context = two_boxes->CreateDefaultContext();

  const int kNumTimeSamples = 28;
  const double kMinimumTimeStep = 0.05;
  const double kMaximumTimeStep = 0.5;
  systems::trajectory_optimization::DirectCollocation dircol(two_boxes.get(),
      *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep,
      systems::InputPortSelection::kUseFirstInputIfItExists,
      false /* non cont states fixed */);
  
  dircol.AddEqualTimeIntervalsConstraints();

  const double kForceLimit = FLAGS_box1_force_limit;
  const solvers::VectorXDecisionVariable& u = dircol.input();
  const solvers::VectorXDecisionVariable& x = dircol.state();
  // for now, input is a 1D force vector on box 1
  dircol.AddConstraintToAllKnotPoints(-kForceLimit <= u(0));
  dircol.AddConstraintToAllKnotPoints(u(0) <= kForceLimit);
  
  drake::VectorX<double> initState1(2);
  initState1 << -1. /* position */, FLAGS_box1_init_v /* velocity */;
  drake::VectorX<double> initState2(2);
  initState2 << 1. /* position */, 0. /* velocity */;
  two_boxes->SetBox1State(context.get(), initState1);
  two_boxes->SetBox2State(context.get(), initState2);

  drake::VectorX<double> finalState(4);
  finalState << -FLAGS_box2_final_position /* b1 position */, 0. /* b1 velocity */,
                FLAGS_box2_final_position /* b2 position */, 0.05 /* b2 vel */ ; 


  drake::VectorX<double> initState = context->get_continuous_state_vector().CopyToVector();
  DRAKE_DEMAND(context->num_continuous_states() == 4);
  std::cout << initState << std::endl;
  dircol.AddLinearConstraint(dircol.initial_state() ==
                             initState);
  dircol.AddLinearConstraint(dircol.final_state()  ==
                             finalState);      
  //dircol.AddLinearConstraint(dircol.final_state()(2)  ==
  //                           finalState(2));
  //dircol.AddLinearConstraint(dircol.final_state()(3)  ==
  //                           finalState(3));
  const double R = FLAGS_input_cost;  // Cost on input "effort"
  drake::VectorX<double> initU(1);
  initU << 0.3;
  auto xd = x.block(0,0,4,1) - finalState.block(0,0,4,1);
  //unused(xd); //+ xd.cast<symbolic::Expression>().dot(xd)
  dircol.AddRunningCost(((R * u) * u)(0) + xd.cast<symbolic::Expression>().dot(xd));
  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initState, finalState});
  auto traj_init_u = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initU, -initU});
  dircol.SetInitialTrajectory(traj_init_u, traj_init_x);
  const auto result = solvers::Solve(dircol);
  if (!result.is_success()) {
    std::cerr << "Failed to solve optimization for the trajectory"
              << std::endl;
    return 1;
  }

  systems::DiagramBuilder<double> finalBuilder;
  auto* boxes = finalBuilder.AddSystem(std::move(two_boxes));
  const PiecewisePolynomial<double> pp_traj =
      dircol.ReconstructInputTrajectory(result);
  auto input_trajectory = finalBuilder.AddSystem<systems::TrajectorySource>(pp_traj);
  input_trajectory->set_name("input trajectory");
  finalBuilder.Connect(input_trajectory->get_output_port(), boxes->get_input_port(0));
  auto scene_graph = finalBuilder.AddSystem<geometry::SceneGraph>();
  AddGeometryToBuilder(&finalBuilder, *boxes, scene_graph);
  ConnectDrakeVisualizer(&finalBuilder, *scene_graph);
  auto finalDiagram = finalBuilder.Build();

  //const double initial_energy = box1->CalcTotalEnergy(box1_context)+box2->CalcTotalEnergy(box2_context);
  systems::Simulator<double> simulator(*finalDiagram);
  systems::Context<double>& simContext = simulator.get_mutable_context();
  systems::Context<double>& boxesContext = finalDiagram->GetMutableSubsystemContext(*boxes, &simContext);
  boxes->SetBox1State(&boxesContext, initState1);
  boxes->SetBox2State(&boxesContext, initState2);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(pp_traj.end_time());


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
