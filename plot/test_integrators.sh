#!/bin/bash

# script for generating work precision csv plots

#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --run_filename="boxout6" --meta_filename="boxsim6"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --run_filename="boxout5" --meta_filename="boxsim5"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-5 --run_filename="boxout45" --meta_filename="boxsim45"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --run_filename="boxout4" --meta_filename="boxsim4"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --run_filename="boxout35" --meta_filename="boxsim35"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --run_filename="boxout3" --meta_filename="boxsim3"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --run_filename="boxout25" --meta_filename="boxsim25"
#bazel run --config snopt //examples/box:passive_simulation -- --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="runge_kutta2" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --run_filename="boxout2" --meta_filename="boxsim2"

cd /home/antequ/code/github/drake
bazel build --config snopt //examples/box:integrator_benchmark

export TEST_SCHEME=implicit_euler
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr25" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr3" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr35" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr4" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr45"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr5"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr55"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr6" &



export TEST_SCHEME=implicit_euler
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}illerr25" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}illerr3" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}illerr35" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}illerr4" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}illerr45"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}illerr5"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}illerr55"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}illerr6" &


export TEST_SCHEME=runge_kutta2
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 3e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr25"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr3"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 3e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr35"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr4"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 3e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr45"   &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr5"   &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 3e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr55"   &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr6" &


export TEST_SCHEME=radau
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr25" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr3"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr35" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr4" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr45"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-7 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr5" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-7 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr55"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-8 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr6" &

export TEST_SCHEME=fixed_implicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr25" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr3"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr35" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr4" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr45"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-7 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr5" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-7 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr55"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-8 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}nlerr6" &


export TEST_SCHEME=implicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}fnlerr25" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}fnlerr3"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}fnlerr35" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}fnlerr4" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}fnlerr45"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-7 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}fnlerr5" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-7 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}fnlerr55"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-8 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}fnlerr6" &




export TEST_SCHEME=radau
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr25" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=5e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr3"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=2e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr35" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr4" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr45"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr5" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr55"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-7 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr6" &



export TEST_SCHEME=fixed_implicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr25" &
# 3e-3 doesn't work
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=2e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr3"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr35" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr4" &
#1e-4 doesn't work. neither does 3-e5
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=2e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr45"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr5" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr55"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-7 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --iteration_limit=true --errors_filename="box${TEST_SCHEME}fillerr6" &



export TEST_SCHEME=runge_kutta3
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr25" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr3" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr35" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr4" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr45"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-12 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr5"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-14 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr55"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-15 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-8 --errors_filename="box${TEST_SCHEME}lerr6"  &



export TEST_SCHEME=bogacki_shampine3
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr25" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr3" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr35" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr4" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr45"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-12 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr5"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-14 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr55"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-15 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr6"  &



export TEST_SCHEME=bogacki_shampine3
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr25" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr3"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr35" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr4" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr45"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr5" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr55"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr6" &




export TEST_SCHEME=runge_kutta3
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr25" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr3"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr35" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr4" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr45"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr5" &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr55"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flerr6" &


export TEST_SCHEME=semi_explicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 3e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr25"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr3"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 3e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr35"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr4"  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 3e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr45"   &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr5"   &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 3e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr55"   &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr6" &


# no friction
export TEST_SCHEME=runge_kutta2
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf25" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf3" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf35" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf4" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf45" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf5" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf55" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf6" --box_mu_s 0. &


# no friction
export TEST_SCHEME=radau
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="radau" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf25" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="radau" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf3" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="radau" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf35" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="radau" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf4" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="radau" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf45" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="radau" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf5" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="radau" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf55" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="radau" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf6" --box_mu_s 0. &


# no friction

export TEST_SCHEME=runge_kutta3
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf25"  --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf3" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf35" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf4" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf45" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-12 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf5" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-14 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf55" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-15 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf6" --box_mu_s 0.  &



# no friction

export TEST_SCHEME=bogacki_shampine3
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf25"  --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf3" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf35" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf4" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf45" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-12 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf5" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-14 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf55" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-15 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf6" --box_mu_s 0. &


# no friction
export TEST_SCHEME=implicit_euler
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf25" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf3" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf35" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf4" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf45" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf5" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf55" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf6" --box_mu_s 0. &


# no friction
export TEST_SCHEME=fixed_implicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf25" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf3" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf35" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf4" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf45" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf5" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf55" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-7 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf6" --box_mu_s 0. &


# no friction
export TEST_SCHEME=bogacki_shampine3
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf25" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf3" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf35" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf4" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf45" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf5" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf55" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf6" --box_mu_s 0. &



# no friction
export TEST_SCHEME=runge_kutta3
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf25" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf3" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf35" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf4" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf45" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf5" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf55" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf6" --box_mu_s 0. &



# no friction
export TEST_SCHEME=implicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf25" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf3" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf35" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf4" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf45" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf5" --box_mu_s 0. &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf55" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-7 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}flnf6" --box_mu_s 0. &



# no friction
export TEST_SCHEME=semi_explicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/'
time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf25" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf3" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf35" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf4" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf45" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf5" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-6 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf55" --box_mu_s 0.  &

time /home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1e-7 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lnf6" --box_mu_s 0. &



cd ../../../../..



#time bazel run --config snopt //examples/box:integrator_benchmark  -- --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=1.7e-5 --fixed_step=$FIXED --box_v0 0.0 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr48" 

#time bazel run --config snopt //examples/box:integrator_benchmark  -- --simulation_time 2.5 --box_v_s 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff="true" --max_time_step=2.4e-5 --fixed_step=$FIXED --box_v0 0.0 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-7 --errors_filename="box${TEST_SCHEME}lerr47"



