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





export TIME_STEPPING=true
export SIMULATION_TIME=2.5
cd '/home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark.runfiles/drake'

nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=2.5e-2 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --errors_filename="implicit_stribeck_1" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-2 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --errors_filename="implicit_stribeck_2" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-3 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --errors_filename="implicit_stribeck_3" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-3 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --errors_filename="implicit_stribeck_4" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-4 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --errors_filename="implicit_stribeck_5" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-4 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --errors_filename="implicit_stribeck_6" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-5 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --errors_filename="implicit_stribeck_7" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-5 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --errors_filename="implicit_stribeck_8" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-6 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --errors_filename="implicit_stribeck_9" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=1e-6 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --errors_filename="implicit_stribeck_10" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark --visualize=false --time_stepping=$TIME_STEPPING --simulation_time=$SIMULATION_TIME --max_time_step=3e-7 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --errors_filename="implicit_stribeck_11" &


# RK3 EC ftff
export TEST_SCHEME=runge_kutta3
export FIXED=false
export CONV=true
export FNR=false
export ITLIM=false
# 'fixed' step vs error control
if [ $FIXED == "true" ]; then export STRF="fx"; else export STRF="ec"; fi
# convergence control vs throw
if [ $CONV == "true" ]; then export STRC="cc"; else export STRC="tr"; fi
# full newton vs quasi newton
if [ $FNR == "true" ]; then export STRN="fn"; else export STRN="qn"; fi
# TALS vs default
if [ $ITLIM == "true" ]; then export STRI="ta"; else export STRI="df"; fi

cd '/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/'
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --integration_scheme="$TEST_SCHEME"  --fixed_step=$FIXED --convergence_control=$CONV --full_newton=$FNR --target_realtime_rate="0" --iteration_limit=$ITLIM --max_time_step=2.5e-2 --accuracy 1e-2 --truth_integration_scheme="runge_kutta3" --fixed_tolerance=1e-9 --truth_integration_step=1e-6 --error_reporting_step=2.5e-2 --errors_filename="finalbox20${TEST_SCHEME}_${STRF}_${STRC}_${STRN}_${STRI}1" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --integration_scheme="$TEST_SCHEME"  --fixed_step=$FIXED --convergence_control=$CONV --full_newton=$FNR --target_realtime_rate="0" --iteration_limit=$ITLIM --max_time_step=2.5e-2 --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --fixed_tolerance=1e-9 --truth_integration_step=1e-6 --error_reporting_step=2.5e-2 --errors_filename="finalbox20${TEST_SCHEME}_${STRF}_${STRC}_${STRN}_${STRI}2" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --integration_scheme="$TEST_SCHEME"  --fixed_step=$FIXED --convergence_control=$CONV --full_newton=$FNR --target_realtime_rate="0" --iteration_limit=$ITLIM --max_time_step=2.5e-2 --accuracy 1e-4 --truth_integration_scheme="runge_kutta3" --fixed_tolerance=1e-9 --truth_integration_step=1e-6 --error_reporting_step=2.5e-2 --errors_filename="finalbox20${TEST_SCHEME}_${STRF}_${STRC}_${STRN}_${STRI}3" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --integration_scheme="$TEST_SCHEME"  --fixed_step=$FIXED --convergence_control=$CONV --full_newton=$FNR --target_realtime_rate="0" --iteration_limit=$ITLIM --max_time_step=2.5e-2 --accuracy 1e-5 --truth_integration_scheme="runge_kutta3" --fixed_tolerance=1e-9 --truth_integration_step=1e-6 --error_reporting_step=2.5e-2 --errors_filename="finalbox20${TEST_SCHEME}_${STRF}_${STRC}_${STRN}_${STRI}4" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --integration_scheme="$TEST_SCHEME"  --fixed_step=$FIXED --convergence_control=$CONV --full_newton=$FNR --target_realtime_rate="0" --iteration_limit=$ITLIM --max_time_step=2.5e-2 --accuracy 1e-8 --truth_integration_scheme="runge_kutta3" --fixed_tolerance=1e-9 --truth_integration_step=1e-6 --error_reporting_step=2.5e-2 --errors_filename="finalbox20${TEST_SCHEME}_${STRF}_${STRC}_${STRN}_${STRI}5" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --integration_scheme="$TEST_SCHEME"  --fixed_step=$FIXED --convergence_control=$CONV --full_newton=$FNR --target_realtime_rate="0" --iteration_limit=$ITLIM --max_time_step=2.5e-2 --accuracy 1e-10 --truth_integration_scheme="runge_kutta3" --fixed_tolerance=1e-9 --truth_integration_step=1e-7 --error_reporting_step=2.5e-2 --errors_filename="finalbox20${TEST_SCHEME}_${STRF}_${STRC}_${STRN}_${STRI}6" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --integration_scheme="$TEST_SCHEME"  --fixed_step=$FIXED --convergence_control=$CONV --full_newton=$FNR --target_realtime_rate="0" --iteration_limit=$ITLIM --max_time_step=2.5e-2 --accuracy 1e-12 --truth_integration_scheme="runge_kutta3" --fixed_tolerance=1e-9 --truth_integration_step=3e-8 --error_reporting_step=2.5e-2 --errors_filename="finalbox20${TEST_SCHEME}_${STRF}_${STRC}_${STRN}_${STRI}7" &
nice -n 2 time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --integration_scheme="$TEST_SCHEME"  --fixed_step=$FIXED --convergence_control=$CONV --full_newton=$FNR --target_realtime_rate="0" --iteration_limit=$ITLIM --max_time_step=2.5e-2 --accuracy 1e-14 --truth_integration_scheme="runge_kutta3" --fixed_tolerance=1e-9 --truth_integration_step=3e-8 --error_reporting_step=2.5e-2 --errors_filename="finalbox20${TEST_SCHEME}_${STRF}_${STRC}_${STRN}_${STRI}8" &


cd ../../../../..



#time bazel run --config snopt //examples/box:integrator_benchmark  -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=1.7e-5 --fixed_step=$FIXED --box_v0 0.0 --truth_integration_scheme="runge_kutta3" --fixed_tolerance=1e-9 --truth_integration_step=1e-6 --error_reporting_step=2.5e-2 --errors_filename="box${TEST_SCHEME}lerr48" 

#time bazel run --config snopt //examples/box:integrator_benchmark  -- --simulation_time 2.5 --v_stiction_tolerance 1e-4 --integration_scheme="$TEST_SCHEME" --target_realtime_rate="0" --autodiff=false --max_time_step=2.4e-5 --fixed_step=$FIXED --box_v0 0.0 --truth_integration_scheme="runge_kutta3" --fixed_tolerance=1e-9 --truth_integration_step=1e-6 --error_reporting_step=2.5e-2 --errors_filename="box${TEST_SCHEME}lerr47"



