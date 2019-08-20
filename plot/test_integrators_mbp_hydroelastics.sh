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
bazel build --config snopt //examples/box2d:box2d
mkdir -p /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/

export TEST_SCHEME=implicit_euler
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr25" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr3" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr35" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr4" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr45"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr5"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr55"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr6" &




if [ 0 -eq 1 ]
then
export TEST_SCHEME=implicit_euler
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}illerr25" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}illerr3" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}illerr35" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}illerr4" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}illerr45"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}illerr5"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}illerr55"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}illerr6" &
fi

export TEST_SCHEME=runge_kutta2
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr25"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr3"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 3e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr35"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr4"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 3e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr45"   &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr5"   &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 3e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr55"   &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr6" &


export TEST_SCHEME=radau
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr25" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr3"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr35" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr4" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr45"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr5" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr55"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr6" &

export TEST_SCHEME=fixed_implicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr25" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr3"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr35" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr4" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr45"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr5" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr55"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}nlerr6" &


export TEST_SCHEME=implicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fnlerr25" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fnlerr3"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fnlerr35" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fnlerr4" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fnlerr45"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fnlerr5" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fnlerr55"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fnlerr6" &




if [ 0 -eq 1 ]
then

export TEST_SCHEME=radau
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr25" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr3"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr35" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr4" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr45"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr5" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr55"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr6" &


#time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-3 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr25" --visualize

export TEST_SCHEME=fixed_implicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr25" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr3"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr35" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr4" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr45"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr5" &
#3e-5 doesn't work
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr55"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr6" &


export TEST_SCHEME=implicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr25" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr3"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr35" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr4" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr45"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr5" &
#3e-5 doesn't work
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr55"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --iteration_limit=true --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}fillerr6" &
fi


export TEST_SCHEME=runge_kutta3
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr25" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr3" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr35" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr4" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr45"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr5"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-12 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr55"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-13 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr6"  &



export TEST_SCHEME=bogacki_shampine3
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr25" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr3" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr35" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr4" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr45"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr5"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-12 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr55"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-13 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr6"  &



export TEST_SCHEME=bogacki_shampine3
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr25" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr3"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr35" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr4" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr45"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr5" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr55"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr6" &




export TEST_SCHEME=runge_kutta3
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr25" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr3"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr35" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr4" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr45"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr5" &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1.5e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr55"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=7e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flerr6" &


export TEST_SCHEME=semi_explicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr25"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr3"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 3e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr35"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr4"  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 3e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr45"   &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr5"   &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 3e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr55"   &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr6" &


# no friction
export TEST_SCHEME=runge_kutta2
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf25" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf3" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf35" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf4" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf45" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf5" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf55" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf6" --use_friction=false &


# no friction
export TEST_SCHEME=radau
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="radau" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf25" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="radau" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf3" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="radau" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf35" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="radau" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf4" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="radau" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf45" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="radau" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf5" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="radau" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf55" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="radau" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf6" --use_friction=false &


# no friction

export TEST_SCHEME=runge_kutta3
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf25"  --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf3" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf35" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf4" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf45" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf5" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-12 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf55" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-13 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf6" --use_friction=false  &



# no friction

export TEST_SCHEME=bogacki_shampine3
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf25"  --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf3" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf35" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf4" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf45" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf5" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 3e-12 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf55" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-13 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf6" --use_friction=false &


# no friction
export TEST_SCHEME=implicit_euler
export FIXED=false
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf25" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf3" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf35" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf4" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf45" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf5" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf55" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf6" --use_friction=false &


# no friction
export TEST_SCHEME=fixed_implicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf25" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf3" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf35" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf4" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf45" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf5" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf55" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf6" --use_friction=false &


# no friction
export TEST_SCHEME=bogacki_shampine3
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf25" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf3" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf35" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf4" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf45" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf5" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf55" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf6" --use_friction=false &



# no friction
export TEST_SCHEME=runge_kutta3
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf25" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf3" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf35" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf4" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf45" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf5" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1.5e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf55" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=7e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf6" --use_friction=false &



# no friction
export TEST_SCHEME=implicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf25" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf3" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf35" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf4" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf45" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf5" --use_friction=false &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf55" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}flnf6" --use_friction=false &



# no friction
export TEST_SCHEME=semi_explicit_euler
export FIXED=true
cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=$FIXED --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf25" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=$FIXED --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf3" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-3 --fixed_step=$FIXED --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf35" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-4 --fixed_step=$FIXED --accuracy 1e-6 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf4" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-4 --fixed_step=$FIXED --accuracy 1e-7 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf45" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-5 --fixed_step=$FIXED --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf5" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-5 --fixed_step=$FIXED --accuracy 1e-9 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf55" --use_friction=false  &

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-6 --fixed_step=$FIXED --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lnf6" --use_friction=false &



cd ../../../../..



#time bazel run --config snopt //examples/box:integrator_benchmark  -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1.7e-5 --fixed_step=$FIXED --box_v0 0.0 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr48" 

#time bazel run --config snopt //examples/box:integrator_benchmark  -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=2.4e-5 --fixed_step=$FIXED --box_v0 0.0 --truth_integration_scheme="runge_kutta3" --truth_integration_step=3e-6 --use_hydroelastics_model=true --errors_filename="box${TEST_SCHEME}lerr47"


#Hydroelastic choking vs not choking:
time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="implicit_euler" --autodiff=false --max_time_step=3e-2 --fixed_step=false --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-1 --iteration_limit=false --visualize=true --target_realtime_rate="1" --use_hydroelastics_model
time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="implicit_euler" --autodiff=false --max_time_step=3e-2 --fixed_step=false --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-1 --iteration_limit=true --visualize=true --target_realtime_rate="1" --use_hydroelastics_model

#Point choking vs not choking:
time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="implicit_euler" --autodiff=false --max_time_step=3e-2 --fixed_step=false --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-1 --iteration_limit=false --visualize=true --target_realtime_rate="1"
time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="implicit_euler" --autodiff=false --max_time_step=3e-2 --fixed_step=false --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-1 --iteration_limit=true --visualize=true --target_realtime_rate="1" 

