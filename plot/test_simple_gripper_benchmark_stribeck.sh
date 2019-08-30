#!/bin/bash

# script for generating work precision csv plots

#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-6 --run_filename="boxout6" --meta_filename="boxsim6"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --run_filename="boxout5" --meta_filename="boxsim5"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --run_filename="boxout45" --meta_filename="boxsim45"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --run_filename="boxout4" --meta_filename="boxsim4"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --run_filename="boxout35" --meta_filename="boxsim35"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --run_filename="boxout3" --meta_filename="boxsim3"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --run_filename="boxout25" --meta_filename="boxsim25"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --run_filename="boxout2" --meta_filename="boxsim2"

cd /home/antequ/code/github/drake
bazel build //examples/simple_gripper:simple_gripper_benchmark
mkdir -p /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark.runfiles/drake




# IS, localized errors
export TIME_STEPPING=true
export SIMULATION_TIME=2.5
cd '/home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark.runfiles/drake'

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=2.5e-2 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors --errors_filename="implicit_stribeck_1" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-2 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors --errors_filename="implicit_stribeck_2" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-3 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors --errors_filename="implicit_stribeck_3" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-3 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors --errors_filename="implicit_stribeck_4" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-4 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors --errors_filename="implicit_stribeck_5" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-4 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors --errors_filename="implicit_stribeck_6" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-5 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors --errors_filename="implicit_stribeck_7" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-5 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors --errors_filename="implicit_stribeck_8" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-6 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors --errors_filename="implicit_stribeck_9" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-6 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors --errors_filename="implicit_stribeck_10" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-7 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors --errors_filename="implicit_stribeck_11" &


# IS, globalized errors
export TIME_STEPPING=true
export SIMULATION_TIME=2.5
cd '/home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark.runfiles/drake'

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=2.5e-2 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors=false --errors_filename="implicit_stribeck_global_1" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-2 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors=false --errors_filename="implicit_stribeck_global_2" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-3 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors=false --errors_filename="implicit_stribeck_global_3" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-3 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors=false --errors_filename="implicit_stribeck_global_4" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-4 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors=false --errors_filename="implicit_stribeck_global_5" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-4 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors=false --errors_filename="implicit_stribeck_global_6" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-5 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors=false --errors_filename="implicit_stribeck_global_7" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-5 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors=false --errors_filename="implicit_stribeck_global_8" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-6 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors=false --errors_filename="implicit_stribeck_global_9" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-6 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors=false --errors_filename="implicit_stribeck_global_10" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-7 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --localize_errors=false --errors_filename="implicit_stribeck_global_11" &


time bazel run //examples/simple_gripper:simple_gripper_benchmark -- --target_realtime_rate=0 --amplitude 0.15 --ring_static_friction=0.05 --contact_model=point --time_stepping=false --full_newton --convergence_control --fixed_step --max_time_step=8e-3 --truth_integration_step=5e-4 --iteration_limit=true --error_reporting_step=2.4e-2 --errors_filename="point_contact_benchmark_itlimiter8ms" --visualize --simulation_time=5

time bazel run //examples/simple_gripper:simple_gripper_benchmark -- --target_realtime_rate=0 --amplitude 0.15 --ring_static_friction=0.05 --contact_model=point --time_stepping=false --full_newton --convergence_control --fixed_step --max_time_step=8e-3 --truth_integration_step=5e-4 --iteration_limit=false --error_reporting_step=2.4e-2 --errors_filename="point_contact_benchmark_noitlimiter8ms" --visualize --simulation_time=5

time bazel run //examples/simple_gripper:simple_gripper_benchmark -- --target_realtime_rate=0 --amplitude 0.15 --ring_static_friction=0.05 --contact_model=point --time_stepping=false --full_newton --convergence_control --fixed_step --max_time_step=5e-3 --truth_integration_step=5e-4 --iteration_limit=false --errors_filename="point_contact_benchmark_itlimiter5ms" --visualize --simulation_time=5


time bazel run //examples/simple_gripper:simple_gripper_benchmark -- --target_realtime_rate=0 --amplitude 0.15 --ring_static_friction=0.05 --contact_model=point --time_stepping=false --full_newton --convergence_control --fixed_step --max_time_step=5e-3 --truth_integration_step=5e-4 --iteration_limit=true --errors_filename="point_contact_benchmark_itlimiter5ms" --visualize --simulation_time=5


time bazel run //examples/simple_gripper:simple_gripper_benchmark -- --target_realtime_rate=0 --amplitude 0.15 --ring_static_friction=0.05 --contact_model=point --time_stepping=false --full_newton --convergence_control --fixed_step --max_time_step=3e-3 --truth_integration_step=5e-4 --iteration_limit=false --error_reporting_step=2.4e-2 --errors_filename="point_contact_benchmark_noitlimiter3ms" --visualize --simulation_time=5


time bazel run //examples/simple_gripper:simple_gripper_benchmark -- --target_realtime_rate=0 --amplitude 0.15 --ring_static_friction=0.05 --contact_model=point --time_stepping=false --full_newton --convergence_control --fixed_step --max_time_step=3e-3 --truth_integration_step=5e-4 --iteration_limit=true --error_reporting_step=2.4e-2 --errors_filename="point_contact_benchmark_itlimiter3ms" --visualize --simulation_time=5

cd ../../../../..



#time bazel run --config snopt //examples/box:integrator_benchmark  -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1.7e-5 --fixed_step=$FIXED --box_v0 0.0 --truth_integration_scheme="runge_kutta3" --fixed_tolerance=1e-9 --truth_integration_step=1e-6 --error_reporting_step=2.5e-2 --errors_filename="box${TEST_SCHEME}lerr48" 

#time bazel run --config snopt //examples/box:integrator_benchmark  -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=2.4e-5 --fixed_step=$FIXED --box_v0 0.0 --truth_integration_scheme="runge_kutta3" --fixed_tolerance=1e-9 --truth_integration_step=1e-6 --error_reporting_step=2.5e-2 --errors_filename="box${TEST_SCHEME}lerr47"



