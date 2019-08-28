#include <memory>
#include <string>

#include <gflags/gflags.h>
#include "fmt/ostream.h"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/radau_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/bogacki_shampine3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/sine.h"

#include <chrono>
#include <iostream>
#include <fstream>
#undef PRINT_VAR
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

namespace drake {
namespace examples {
namespace simple_gripper {
namespace {

using Eigen::Vector3d;
using geometry::SceneGraph;
using geometry::Sphere;
using lcm::DrakeLcm;
using math::RigidTransformd;
using math::RollPitchYawd;
using multibody::Body;
using multibody::CoulombFriction;
using multibody::ConnectContactResultsToDrakeVisualizer;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::PrismaticJoint;
using systems::ImplicitEulerIntegrator;
using systems::RungeKutta2Integrator;
using systems::RungeKutta3Integrator;
using systems::SemiExplicitEulerIntegrator;
using systems::Sine;

// TODO(amcastro-tri): Consider moving this large set of parameters to a
// configuration file (e.g. YAML).
DEFINE_double(target_realtime_rate, 0.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation. [s].");

DEFINE_double(grip_width, 0.095,
              "The initial distance between the gripper fingers. [m].");

// Integration parameters:
DEFINE_string(integration_scheme, "implicit_euler",
              "Integration scheme to be used. Available options are: "
              "'semi_explicit_euler','runge_kutta2','runge_kutta3',"
              "'implicit_euler'");
DEFINE_double(max_time_step, 1.0e-3,
              "Maximum time step used for the integrators. [s]. "
              "If negative, a value based on parameter penetration_allowance "
              "is used.");
DEFINE_double(accuracy, 1.0e-4, "Sets the simulation accuracy for variable step"
              "size integrators with error control.");
DEFINE_bool(time_stepping, true, "If 'true', the plant is modeled as a "
    "discrete system with periodic updates of period 'max_time_step'."
    "If 'false', the plant is modeled as a continuous system.");

DEFINE_bool(iteration_limit, true, "Set true to use iteration limiter.");
DEFINE_bool(fixed_step, false, "Set true to force fixed timesteps.");
DEFINE_double(fixed_tolerance, 1.e-4, "Tolerance for Newton iterations of fixed implicit integrators.");
DEFINE_bool(autodiff, false, "Set true to use AutoDiff in Jacobian computation.");

DEFINE_bool(full_newton, false, "set this to ensure implicit integrators do the full Newton-Raphson.");
DEFINE_bool(convergence_control, false, "set this to allow convergence control.");

DEFINE_string(truth_integration_scheme, "runge_kutta2",
              "Integration scheme for computing truth (fixed). Available options are: "
              "'fixed_implicit_euler', 'implicit_euler' (ec), 'semi_explicit_euler',"
              "'runge_kutta2', 'runge_kutta3' (ec), 'bogacki_shampine3' (ec), 'radau'");
DEFINE_bool(truth_autodiff, false, "Set true to use AutoDiff in Jacobian computation in truth.");
DEFINE_double(truth_integration_step, 3.0e-7,
              "Timestep size for integrating the truth.");

DEFINE_bool(truth_fixed_step, true, "Use fixed steps for truth.");
DEFINE_double(truth_accuracy, 1e-17, "Target accuracy for truth.");
DEFINE_double(error_reporting_step, 2.5e-2,
              "Period between which local error is calculated.");

DEFINE_bool(visualize, false, "Set true to visualize.");
// Contact parameters
DEFINE_double(penetration_allowance, 1.0e-2,
              "Penetration allowance. [m]. "
              "See MultibodyPlant::set_penetration_allowance().");
DEFINE_double(v_stiction_tolerance, 1.0e-4,
              "The maximum slipping speed allowed during stiction. [m/s]");

// Pads parameters
DEFINE_int32(ring_samples, 8,
             "The number of spheres used to sample the pad ring");
DEFINE_double(ring_orient, 0, "Rotation of the pads around x-axis. [degrees]");
DEFINE_double(ring_static_friction, 0.05, "The coefficient of static friction "
              "for the ring pad.");
DEFINE_double(ring_dynamic_friction, 0.05, "The coefficient of dynamic friction "
              "for the ring pad.");

// Parameters for rotating the mug.
DEFINE_double(rx, 0, "The x-rotation of the mug around its origin - the center "
              "of its bottom. [degrees]. Extrinsic rotation order: X, Y, Z");
DEFINE_double(ry, 0, "The y-rotation of the mug around its origin - the center "
              "of its bottom. [degrees]. Extrinsic rotation order: X, Y, Z");
DEFINE_double(rz, 0, "The z-rotation of the mug around its origin - the center "
              "of its bottom. [degrees]. Extrinsic rotation order: X, Y, Z");

// Gripping force.
DEFINE_double(gripper_force, 10, "The force to be applied by the gripper. [N]. "
              "A value of 0 indicates a fixed grip width as set with option "
              "grip_width.");

// Parameters for shaking the mug.
DEFINE_double(amplitude, 0.15, "The amplitude of the harmonic oscillations "
              "carried out by the gripper. [m].");
DEFINE_double(frequency, 2.0, "The frequency of the harmonic oscillations "
              "carried out by the gripper. [Hz].");

DEFINE_string(contact_model, "point",
              "Contact model. Options are: 'point', 'hydroelastic'.");              
DEFINE_double(elastic_modulus, 2.5e5, "Elastic modulus, in Pa.");
DEFINE_double(dissipation, 5.0, "dissipation, in s/m.");            

DEFINE_string(meta_filename, "boxsim",
                "Filename for meta output. \".csv\" will be postpended.");

DEFINE_string(errors_filename, "boxlerr",
                "Filename for local error output. \".csv\" will be postpended.");

// The pad was measured as a torus with the following major and minor radii.
const double kPadMajorRadius = 14e-3;  // 14 mm.
const double kPadMinorRadius = 6e-3;   // 6 mm.

// This uses the parameters of the ring to add collision geometries to a
// rigid body for a finger. The collision geometries, consisting of a set of
// small spheres, approximates a torus attached to the finger.
//
// @param[in] plant the MultiBodyPlant in which to add the pads.
// @param[in] pad_offset the ring offset along the x-axis in the finger
// coordinate frame, i.e., how far the ring protrudes from the center of the
// finger.
// @param[in] finger the Body representing the finger
void AddGripperPads(MultibodyPlant<double>* plant,
                    const double pad_offset, const Body<double>& finger) {
  const int sample_count = FLAGS_ring_samples;
  const double sample_rotation = FLAGS_ring_orient * M_PI / 180.0;  // radians.
  const double d_theta = 2 * M_PI / sample_count;
  const double torus_center_y_position_F = 0.0265;
  CoulombFriction<double> friction(
        FLAGS_ring_static_friction, FLAGS_ring_static_friction);

  if (FLAGS_contact_model == "hydroelastic") {
    // we make the squishy red pads for the hydro model thicker so that, for the
    // same level of penetration as for poiint contact (~5.6 mm), we don't get
    // the mug to cross the center of the squishy box shapped pads (that'd make
    // hydro fail).
    double hydro_pad_offset = pad_offset > 0 ? pad_offset - kPadMinorRadius
                                             : pad_offset + kPadMinorRadius;
    // For the same reason, twice as thick as the point contact model effective
    // thickness (the diameter of the spheres).
    double hydro_pad_thickness = 4 * kPadMinorRadius;
    Vector3d p_FSo(hydro_pad_offset, torus_center_y_position_F, 0.0);    
    const RigidTransformd X_FS(p_FSo);
    const double box_size = 2 * kPadMajorRadius;
    const geometry::Box box(hydro_pad_thickness, box_size, box_size);
    const auto box_id = plant->RegisterCollisionGeometry(finger, X_FS, box,
                                     finger.name() + "_collision", friction);
    const Vector4<double> red(0.8, 0.2, 0.2, 1.0);
    plant->RegisterVisualGeometry(finger, X_FS, box, finger.name() + "_visual", red);
    plant->set_elastic_modulus(box_id, FLAGS_elastic_modulus);
    plant->set_hydroelastics_dissipation(box_id, FLAGS_dissipation);
    return;
  }

  Vector3d p_FSo;  // Position of the sphere frame S in the finger frame F.
  // The finger frame is defined in simpler_gripper.sdf so that:
  //  - x axis pointing to the right of the gripper.
  //  - y axis pointing forward in the direction of the fingers.
  //  - z axis points up.
  //  - It's origin Fo is right at the geometric center of the finger.
  for (int i = 0; i < sample_count; ++i) {
    // The y-offset of the center of the torus in the finger frame F.    
    p_FSo(0) = pad_offset;  // Offset from the center of the gripper.
    p_FSo(1) =
        std::cos(d_theta * i + sample_rotation) * kPadMajorRadius +
            torus_center_y_position_F;
    p_FSo(2) = std::sin(d_theta * i + sample_rotation) * kPadMajorRadius;

    // Pose of the sphere frame S in the finger frame F.
    const RigidTransformd X_FS(p_FSo);
  
    plant->RegisterCollisionGeometry(finger, X_FS, Sphere(kPadMinorRadius),
                                     "collision" + std::to_string(i), friction);

    const Vector4<double> red(0.8, 0.2, 0.2, 1.0);
    plant->RegisterVisualGeometry(finger, X_FS, Sphere(kPadMinorRadius),
                                  "visual" + std::to_string(i), red);
  }
}
void StoreEigenCSV(const std::string& filename, const Eigen::VectorXd& times, const Eigen::MatrixXd& data,
                const Eigen::MatrixXi& metadata) {
  /* csv format from  https://stackoverflow.com/questions/18400596/how-can-a-eigen-matrix-be-written-to-file-in-csv-format */
  const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision,
                                  Eigen::DontAlignCols, ", ", "\n");
  DRAKE_DEMAND(times.rows() == data.rows());
  DRAKE_DEMAND(times.rows() == metadata.rows());
  
  std::ofstream file(filename);
  file << "t, runtime_us, n_steps, truth_n_steps, sim mug_qw, sim mug_qx, sim mug_qy, sim mug_qz, "
    << "sim mug_x, sim mug_y, sim mug_z, sim grip_z, sim grip_w, sim mug_wx, sim mug_wy, sim mug_wz, sim mug_vx, sim mug_vy, "
    << "sim mug_vz, sim grip_vz, sim grip_vw, truth mug_qw, truth mug_qx, truth mug_qy, truth mug_qz, "
    << "truth mug_x, truth mug_y, truth mug_z, truth grip_z, truth grip_w, truth mug_wx, truth mug_wy, truth mug_wz, truth mug_vx, truth mug_vy, "
    << "truth mug_vz, truth grip_vz, truth grip_vw" << std::endl;
  /* horizontally concatenate times and data */
  MatrixX<double> OutMatrix(times.rows(), times.cols() + metadata.cols() + data.cols());
  OutMatrix << times, metadata.cast<double>(), data; 
     /* can also do this with blocks */
  file << OutMatrix.format(CSVFormat);
  file.close();
}

 void SetupSystem(bool discrete, bool visualize, double time_step_size, 
 DrakeLcm& lcm, MultibodyPlant<double>*& plant_ptr,
 std::unique_ptr<systems::Diagram<double>>& diagram, 
 std::unique_ptr<systems::Context<double>>& diagram_context)
 {

  systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  DRAKE_DEMAND(time_step_size > 0);

  plant_ptr =
      discrete ?
      builder.AddSystem<MultibodyPlant>(time_step_size) :
      builder.AddSystem<MultibodyPlant>();
  MultibodyPlant<double>& plant = *plant_ptr;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  Parser parser(&plant);
  std::string full_name =
      FindResourceOrThrow("drake/examples/simple_gripper/simple_gripper.sdf");
  parser.AddModelFromFile(full_name);

  full_name =
      FindResourceOrThrow("drake/examples/simple_gripper/simple_mug.sdf");
  parser.AddModelFromFile(full_name);

  // Obtain the "translate_joint" axis so that we know the direction of the
  // forced motions. We do not apply gravity if motions are forced in the
  // vertical direction so that the gripper doesn't start free falling. See note
  // below on how we apply these motions. A better strategy would be using
  // constraints but we keep it simple for this demo.
  const PrismaticJoint<double>& translate_joint =
      plant.GetJointByName<PrismaticJoint>("translate_joint");
  const Vector3d axis = translate_joint.translation_axis();
  if (axis.isApprox(Vector3d::UnitZ())) {
    fmt::print("Gripper motions forced in the vertical direction.\n");
    plant.mutable_gravity_field().set_gravity_vector(Vector3d::Zero());
  } else if (axis.isApprox(Vector3d::UnitX())) {
    fmt::print("Gripper motions forced in the horizontal direction.\n");
  } else {
    throw std::runtime_error(
        "Only horizontal or vertical motions of the gripper are supported for "
        "this example. The joint axis in the SDF file must either be the "
        "x-axis or the z-axis");
  }

  // Add the pads.
  const Body<double>& left_finger = plant.GetBodyByName("left_finger");
  const Body<double>& right_finger = plant.GetBodyByName("right_finger");

  if (FLAGS_contact_model == "hydroelastic") {
    plant.use_hydroelastic_model(true);
  } else if (FLAGS_contact_model == "point") {
    plant.use_hydroelastic_model(false);
  } else {
    throw std::runtime_error("Invalid contact model: '" + FLAGS_contact_model +
                             "'.");
  }

  // Pads offset from the center of a finger. pad_offset = 0 means the center of
  // the spheres is located right at the center of the finger.
  const double pad_offset = 0.0046;
  if (FLAGS_gripper_force == 0) {
    // We then fix everything to the right finger and leave the left finger
    // "free" with no applied forces (thus we see it not moving).
    const double finger_width = 0.007;  // From the visual in the SDF file.
    AddGripperPads(&plant, -pad_offset, right_finger);
    AddGripperPads(&plant,
                   -(FLAGS_grip_width + finger_width) + pad_offset,
                   right_finger);
  } else {
    AddGripperPads(&plant, -pad_offset, right_finger);
    AddGripperPads(&plant, +pad_offset, left_finger);
  }

  // Now the model is complete.
  plant.Finalize();

  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_v_stiction_tolerance);

  // Print maximum time step and the time scale introduced by the compliance in
  // the contact model as a reference to the user.
  fmt::print("Maximum time step = {:10.6f} s\n", time_step_size);
  fmt::print("Compliance time scale = {:10.6f} s\n",
             plant.get_contact_penalty_method_time_scale());

  // from simple_griper.sdf, there are two actuators. One actuator on the
  // prismatic joint named "finger_sliding_joint" to actuate the left finger and
  // a second actuator on the prismatic joint named "translate_joint" to impose
  // motions of the gripper.
  DRAKE_DEMAND(plant.num_actuators() == 2);
  DRAKE_DEMAND(plant.num_actuated_dofs() == 2);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  if( visualize )
  {
    geometry::ConnectDrakeVisualizer(&builder, scene_graph, &lcm);

    // Publish contact results for visualization.
    // (Currently only available when time stepping.)
    //if (FLAGS_time_stepping)
    //  ConnectContactResultsToDrakeVisualizer(&builder, plant, &lcm);
  }

  // Sinusoidal force input. We want the gripper to follow a trajectory of the
  // form x(t) = X0 * sin(ω⋅t). By differentiating once, we can compute the
  // velocity initial condition, and by differentiating twice, we get the input
  // force we need to apply.
  // The mass of the mug is ignored.
  // TODO(amcastro-tri): add a PD controller to precisely control the
  // trajectory of the gripper. Even better, add a motion constraint when MBP
  // supports it.

  // The mass of the gripper in simple_gripper.sdf.
  // TODO(amcastro-tri): we should call MultibodyPlant::CalcMass() here.
  const double mass = 1.0890;  // kg.
  const double omega = 2 * M_PI * FLAGS_frequency;  // rad/s.
  const double x0 = FLAGS_amplitude;  // meters.
  const double v0 = -x0 * omega;  // Velocity amplitude, initial velocity, m/s.
  const double a0 = omega * omega * x0;  // Acceleration amplitude, m/s².
  const double f0 = mass * a0;  // Force amplitude, Newton.
  fmt::print("Acceleration amplitude = {:8.4f} m/s²\n", a0);

  // Notice we are using the same Sine source to:
  //   1. Generate a harmonic forcing of the gripper with amplitude f0 and
  //      angular frequency omega.
  //   2. Impose a constant force to the left finger. That is, a harmonic
  //      forcing with "zero" frequency.
  const Vector2<double> amplitudes(f0, FLAGS_gripper_force);
  const Vector2<double> frequencies(omega, 0.0);
  const Vector2<double> phases(0.0, M_PI_2);
  const auto& harmonic_force = *builder.AddSystem<Sine>(
      amplitudes, frequencies, phases);

  builder.Connect(harmonic_force.get_output_port(0),
                  plant.get_actuation_input_port());
  
  diagram = builder.Build();

  // Create a context for this system:
  diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Get joints so that we can set initial conditions.
  const PrismaticJoint<double>& finger_slider =
      plant.GetJointByName<PrismaticJoint>("finger_sliding_joint");

  // Set initial position of the left finger.
  finger_slider.set_translation(&plant_context, -FLAGS_grip_width);  

  // Get mug body so we can set its initial pose.
  const Body<double>& mug = plant.GetBodyByName("main_body");

  // Initialize the mug pose to be right in the middle between the fingers.
  const Vector3d& p_WBr = plant.EvalBodyPoseInWorld(
      plant_context, right_finger).translation();
  const Vector3d& p_WBl = plant.EvalBodyPoseInWorld(
      plant_context, left_finger).translation();
  const double mug_y_W = (p_WBr(1) + p_WBl(1)) / 2.0;

  RigidTransformd X_WM(
      RollPitchYawd(FLAGS_rx * M_PI / 180, FLAGS_ry * M_PI / 180,
                    (FLAGS_rz * M_PI / 180) + M_PI),
      Vector3d(0.0, mug_y_W, 0.0));
  plant.SetFreeBodyPose(&plant_context, mug, X_WM);

  // Set the initial height of the gripper and its initial velocity so that with
  // the applied harmonic forces it continues to move in a harmonic oscillation
  // around this initial position.
  translate_joint.set_translation(&plant_context, 0.0);
  translate_joint.set_translation_rate(&plant_context, v0);
 }

int do_main() {
  DrakeLcm lcm;
  MultibodyPlant<double>* sim_plant_ptr = nullptr;
  std::unique_ptr<systems::Diagram<double>> sim_diagram = nullptr;
  std::unique_ptr<systems::Context<double>> sim_diagram_context = nullptr;
  SetupSystem(FLAGS_time_stepping, FLAGS_visualize, FLAGS_max_time_step, lcm, sim_plant_ptr,
                            sim_diagram, sim_diagram_context);
  MultibodyPlant<double>& sim_plant = *sim_plant_ptr;

  DrakeLcm truth_lcm;
  MultibodyPlant<double>* truth_plant_ptr = nullptr;
  std::unique_ptr<systems::Diagram<double>> truth_diagram = nullptr;
  std::unique_ptr<systems::Context<double>> truth_diagram_context = nullptr;
  SetupSystem(FLAGS_time_stepping, false, FLAGS_truth_integration_step, truth_lcm, truth_plant_ptr,
                            truth_diagram, truth_diagram_context);
  MultibodyPlant<double>& truth_plant = *truth_plant_ptr;


  // Set up simulators.
  systems::Simulator<double> simulator(*sim_diagram, std::move(sim_diagram_context));
  systems::Simulator<double> truth_simulator(*truth_diagram, std::move(truth_diagram_context));
  
  systems::Context<double>& truth_context = truth_simulator.get_mutable_context();

  systems::IntegratorBase<double>* integrator{nullptr};
  systems::IntegratorBase<double>* truth_integrator{nullptr}; 
if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        simulator.reset_integrator<systems::ImplicitEulerIntegrator<double>>(
            *sim_diagram, &simulator.get_mutable_context());
    if(FLAGS_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
    if(FLAGS_fixed_step)
    {
      integrator->set_target_accuracy(FLAGS_fixed_tolerance);
    }
    static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_reuse(!FLAGS_full_newton);

  } else if (FLAGS_integration_scheme == "runge_kutta2") {
    integrator =
        simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
            *sim_diagram, FLAGS_max_time_step, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        simulator.reset_integrator<systems::RungeKutta3Integrator<double>>(
            *sim_diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "bogacki_shampine3") {
    integrator =
        simulator.reset_integrator<systems::BogackiShampine3Integrator<double>>(
            *sim_diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
            *sim_diagram, FLAGS_max_time_step, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "fixed_implicit_euler" || FLAGS_integration_scheme == "radau1") {
    integrator =
        simulator.reset_integrator<systems::RadauIntegrator<double,1>>(
            *sim_diagram, &simulator.get_mutable_context());
    if(FLAGS_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
    if(FLAGS_fixed_step)
    {
      integrator->set_target_accuracy(FLAGS_fixed_tolerance);
    }
    static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_reuse(!FLAGS_full_newton);
  } else if (FLAGS_integration_scheme == "radau" || FLAGS_integration_scheme == "radau3") {
    integrator =
        simulator.reset_integrator<systems::RadauIntegrator<double,2>>(
            *sim_diagram, &simulator.get_mutable_context());
    if(FLAGS_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
    if(FLAGS_fixed_step)
    {
      integrator->set_target_accuracy(FLAGS_fixed_tolerance);
    }
    static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_reuse(!FLAGS_full_newton);
  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
            "' not supported for this example.");
  }

      // Set the iteration limiter method.
  std::unique_ptr<systems::Context<double>> scratch_ctx_k = sim_plant.CreateDefaultContext();
  std::unique_ptr<systems::Context<double>> scratch_ctx_kp1 = sim_plant.CreateDefaultContext();
    auto iteration_limiter = [&sim_diagram, &sim_plant, &scratch_ctx_k, &scratch_ctx_kp1](const systems::Context<double>& ctx0,
    const systems::ContinuousState<double>& x_k, const systems::ContinuousState<double>& x_kp1) -> double {
         const systems::Context<double>& plant_ctx0 = sim_diagram->GetSubsystemContext(sim_plant, ctx0);
         /* this method is very poorly named but gets the subsystem continuous state */
         Eigen::VectorXd v_k = sim_diagram->GetSubsystemDerivatives(sim_plant, x_k).get_generalized_velocity().CopyToVector();
         Eigen::VectorXd v_kp1 = sim_diagram->GetSubsystemDerivatives(sim_plant, x_kp1).get_generalized_velocity().CopyToVector();
         return sim_plant.CalcIterationLimiterAlpha(plant_ctx0, v_k, v_kp1, scratch_ctx_k.get(), scratch_ctx_kp1.get());
      };
  if(FLAGS_iteration_limit)
  {
    integrator->set_iteration_limiter(iteration_limiter);
  } 
  integrator->set_maximum_step_size(FLAGS_max_time_step);
  if (integrator->supports_error_estimation())
    integrator->set_fixed_step_mode( FLAGS_fixed_step );
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(FLAGS_accuracy);

  integrator->set_convergence_control(FLAGS_convergence_control);
  if (FLAGS_visualize )
  {
    // The error controlled integrators might need to take very small time steps
    // to compute a solution to the desired accuracy. Therefore, to visualize
    // these very short transients, we publish every time step.
    simulator.set_publish_every_time_step(true);
  }
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();


  if (FLAGS_truth_integration_scheme == "implicit_euler") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::ImplicitEulerIntegrator<double>>(
            *truth_diagram, &truth_simulator.get_mutable_context());
    if(FLAGS_truth_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(truth_integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
  } else if (FLAGS_truth_integration_scheme == "runge_kutta2") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
            *truth_diagram, FLAGS_truth_integration_step,
            &truth_simulator.get_mutable_context());
  } else if (FLAGS_truth_integration_scheme == "runge_kutta3") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::RungeKutta3Integrator<double>>(
            *truth_diagram, &truth_simulator.get_mutable_context());
  } else if (FLAGS_truth_integration_scheme == "bogacki_shampine3") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::BogackiShampine3Integrator<double>>(
            *truth_diagram, &truth_simulator.get_mutable_context());
  } else if (FLAGS_truth_integration_scheme == "semi_explicit_euler") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
            *truth_diagram, FLAGS_truth_integration_step, &truth_simulator.get_mutable_context());
  } else if (FLAGS_truth_integration_scheme == "fixed_implicit_euler") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::RadauIntegrator<double,1>>(
            *truth_diagram, &truth_simulator.get_mutable_context());
    if(FLAGS_truth_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(truth_integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
  } else if (FLAGS_truth_integration_scheme == "radau") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::RadauIntegrator<double>>(
            *truth_diagram, &truth_simulator.get_mutable_context());
    if(FLAGS_truth_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(truth_integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
  } else {
    throw std::runtime_error(
        "Truth integration scheme '" + FLAGS_truth_integration_scheme +
            "' not supported for this example.");
  }
  
  truth_integrator->set_maximum_step_size(FLAGS_truth_integration_step);
  if (truth_integrator->supports_error_estimation())
    truth_integrator->set_fixed_step_mode( FLAGS_truth_fixed_step );
  if (!truth_integrator->get_fixed_step_mode())
    truth_integrator->set_target_accuracy(FLAGS_truth_accuracy);
  if (FLAGS_visualize )
  {
    // The error controlled integrators might need to take very small time steps
    // to compute a solution to the desired accuracy. Therefore, to visualize
    // these very short transients, we publish every time step.
    truth_simulator.set_publish_every_time_step(true);
  }
  truth_simulator.set_target_realtime_rate(0.0);
  truth_simulator.Initialize();

  int nsteps = std::ceil(FLAGS_simulation_time / FLAGS_error_reporting_step);
  const systems::Context<double>& init_sim_plant_context = 
    sim_diagram->GetSubsystemContext(sim_plant, simulator.get_context());
  //truth_context.get_mutable_state().SetFrom(simulator.get_context().get_state());
  systems::Context<double>& truth_sim_plant_context = 
    truth_diagram->GetMutableSubsystemContext(truth_plant, &truth_context);
  truth_plant.SetPositionsAndVelocities(&truth_sim_plant_context, sim_plant.GetPositionsAndVelocities(init_sim_plant_context));

  int nstate = sim_plant.GetPositionsAndVelocities(init_sim_plant_context).rows();
  std::cout << "nstate: " << nstate << std::endl;
  int nmetadata = 3; /* nanosecs for sim, num steps for sim, num steps for truth */
  Eigen::VectorXd times = Eigen::VectorXd::Zero(nsteps+1);
  //Eigen::MatrixXd quick_state_transfer = Eigen::MatrixXd::Zero(nsteps+1, nstate );
  Eigen::MatrixXd error_results = Eigen::MatrixXd::Zero(2 * nstate ,nsteps+1 );
  //Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic> error_meta = Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic>::Zero(nsteps+1, nmetadata);
  Eigen::MatrixXi error_meta = Eigen::MatrixXi::Zero(nmetadata,nsteps+1 );
#ifdef INITIALIZE_STATE_STORAGE
  std::vector<systems::State<double>> quick_state_transfer (nsteps);
  std::cout << "Initializing state transfer vector..." << std::endl;
  // initialize state sizes
  {
    const systems::State<double>& simstate = simulator.get_context().get_state();
    for(int clone_ind = 0; clone_ind < nsteps; clone_ind++)
    {
      quick_state_transfer[clone_ind].set_abstract_state(simstate.get_abstract_state().Clone());
      quick_state_transfer[clone_ind].set_continuous_state(simstate.get_continuous_state().Clone());
      quick_state_transfer[clone_ind].set_discrete_state(simstate.get_discrete_state().Clone());
    }
      // sanity check
      quick_state_transfer[0].SetFrom(simstate);
  }
  std::cout << "finished initializing state transfer vector!" << std::endl;
#endif
  error_results.block(0,0,nstate,1) = sim_plant.GetPositionsAndVelocities(init_sim_plant_context);
  error_results.block(nstate,0,nstate,1) = sim_plant.GetPositionsAndVelocities(init_sim_plant_context);
  std::cout << "intial state " << sim_plant.GetPositionsAndVelocities(init_sim_plant_context) << std::endl;
  std::cout << "intialized error_results matrix, rows 1 and 2:\n" << error_results.block(0,0,2,2*nstate) << std::endl;
  // time 0 states
  // let's do the simulation first!
  double time = 0;
  int progress_out_rate = std::min(nsteps / 100, 1);
  //int64_t modulus = 1;
  //modulus = 0xffffffffu;
#ifdef PRINT_PROGRESS_INFO
  const PrismaticJoint<double>& finger_slider =
      sim_plant.GetJointByName<PrismaticJoint>("finger_sliding_joint");
  std::cout << "joint translation: " << finger_slider.get_translation(init_sim_plant_context) << std::endl;
#endif
  auto startClockTime = std::chrono::steady_clock::now();
  for(int next_step_ind = 1; next_step_ind <= nsteps; ++next_step_ind)
  {
    double next_time = time + next_step_ind * FLAGS_error_reporting_step;
    if ( next_step_ind == nsteps )
    {
      next_time = FLAGS_simulation_time;
    }

    //quick_state_transfer[next_step_ind - 1].SetFrom(simulator.get_context().get_state());
    simulator.AdvanceTo(next_time);
    const systems::Context<double>& sim_plant_context =
      sim_diagram->GetSubsystemContext(sim_plant, simulator.get_context());
    error_results.block(0,next_step_ind,nstate,1) = sim_plant.GetPositionsAndVelocities(sim_plant_context);
    auto currentClockTime = std::chrono::steady_clock::now();
    error_meta(0, next_step_ind) = (std::chrono::duration_cast<std::chrono::microseconds>(currentClockTime - startClockTime).count() );
    error_meta(1, next_step_ind) = simulator.get_num_steps_taken();

#ifdef PRINT_PROGRESS_INFO
    if(next_step_ind % progress_out_rate == 0)
    {
      std::cout << "joint translation: " << finger_slider.get_translation(sim_plant_context) << std::endl;
      std::stringstream to_out;
      if(FLAGS_time_stepping)
         to_out << "test sim " << "discrete " << FLAGS_max_time_step << "s: " << next_time << " s.";
      else
        to_out << "test sim " << FLAGS_integration_scheme << (FLAGS_fixed_step ? " fixed, " : " ec, ") << FLAGS_max_time_step << "s : " << next_time << " s.";
      std::cout << to_out.str() << std::endl;
    }
#endif
  }
  std::cout << "finished the test simulation! Duration: " << error_meta(0, nsteps) << " us. Starting control ..." << std::endl;


  for(int next_step_ind = 1; next_step_ind <= nsteps; ++next_step_ind)
  {
    double next_time = time + next_step_ind * FLAGS_error_reporting_step;
    if ( next_step_ind == nsteps )
    {
      next_time = FLAGS_simulation_time;
    }
    systems::Context<double>& curr_truth_context = truth_simulator.get_mutable_context();
    //curr_truth_context.get_mutable_state().SetFrom(quick_state_transfer[next_step_ind - 1]);
    systems::Context<double>& curr_truth_plant_context = truth_diagram->GetMutableSubsystemContext(truth_plant,&curr_truth_context);
    truth_plant.SetPositionsAndVelocities(&curr_truth_plant_context, error_results.block(0, next_step_ind - 1, nstate,1) );
    truth_simulator.AdvanceTo(next_time);
    const systems::Context<double>& truth_plant_context =
      truth_diagram->GetSubsystemContext(truth_plant, truth_simulator.get_context());
    times(next_step_ind) = next_time;
    error_results.block(nstate, next_step_ind,nstate,1) = truth_plant.GetPositionsAndVelocities(truth_plant_context);
    error_meta(2, next_step_ind) = truth_simulator.get_num_steps_taken();

    if(next_step_ind % progress_out_rate == 0)
    {
      std::stringstream to_out;
      if(FLAGS_time_stepping)
         to_out << "truth sim " << "discrete " << FLAGS_max_time_step << "s: " << next_time << " s.";
      else
        to_out << "truth sim " << FLAGS_integration_scheme << (FLAGS_fixed_step ? " fixed, " : " ec, ") << FLAGS_max_time_step << "s : " << next_time << " s.";
      std::cout << to_out.str() << std::endl;
    }
  }
  StoreEigenCSV(FLAGS_errors_filename + ".csv", times, error_results.transpose(), error_meta.transpose());
  std::ofstream file(FLAGS_meta_filename + ".csv");
  file << "steps, duration, max_dt, runtime (us)\n";
  file << simulator.get_num_steps_taken() << ", " << FLAGS_simulation_time << ", " << FLAGS_max_time_step << ", " << error_meta(0, nsteps) << std::endl;
  file.close();
  //unused(time);
  //unused(times);
  //unused(error_results);
  //unused(error_meta);
  //unused(quick_state_transfer);
  //unused(truth_plant);
  //unused(truth_simulator);

  //simulator.AdvanceTo(FLAGS_simulation_time);

  if (FLAGS_time_stepping) {
    fmt::print("Used time stepping with dt={}\n", FLAGS_max_time_step);
    fmt::print("Number of time steps taken = {:d}\n",
               simulator.get_num_steps_taken());
  } else {
    fmt::print("Stats for integrator {}:\n", FLAGS_integration_scheme);
    fmt::print("Number of time steps taken = {:d}\n",
               integrator->get_num_steps_taken());
    if (!integrator->get_fixed_step_mode()) {
      fmt::print("Initial time step taken = {:10.6g} s\n",
                 integrator->get_actual_initial_step_size_taken());
      fmt::print("Largest time step taken = {:10.6g} s\n",
                 integrator->get_largest_step_size_taken());
      fmt::print("Smallest adapted step size = {:10.6g} s\n",
                 integrator->get_smallest_adapted_step_size_taken());
      fmt::print("Number of steps shrunk due to error control = {:d}\n",
                 integrator->get_num_step_shrinkages_from_error_control());
    }
  }


  return 0;
}

}  // namespace
}  // namespace simple_gripper
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Demo used to exercise MultibodyPlant's contact modeling in a gripping "
      "scenario. SceneGraph is used for both visualization and contact "
      "handling. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::simple_gripper::do_main();
}
