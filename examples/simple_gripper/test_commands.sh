#!/bin/bash
#simple gripper commands to compare time stepping implicit stribeck vs continuous iteration limiter
# ignore these three
#time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=1e-2 --time_stepping=false --accuracy 1e-4 --iteration_limit=false


#time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=1e-2 --time_stepping=false --accuracy 1e-4 --iteration_limit=true

#time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=1e-2 --time_stepping=true --accuracy 1e-3

# continuous iteration limited, 3ms time steps
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=3e-3 --time_stepping=false --accuracy 1e-3 --iteration_limit=true --simulation_time 30 --fixed_step=true --fixed_tolerance 3e-4 1> /dev/null
# discrete implicit stribeck, 3ms time steps
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=3e-3 --time_stepping=true --accuracy 1e-3 --iteration_limit=true --simulation_time 30 --fixed_step=true --fixed_tolerance 3e-4 1> /dev/null

# continuous iteration limited, 10ms time steps
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=1e-2 --time_stepping=false --accuracy 3e-3 --iteration_limit=true --simulation_time 30 --fixed_step=true --fixed_tolerance 1e-3 1> /dev/null
# discrete implicit stribeck, 10ms time steps
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=1e-2 --time_stepping=true --accuracy 3e-3 --iteration_limit=true --simulation_time 30 --fixed_step=true --fixed_tolerance 1e-3 1> /dev/null

#turn off visualization
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=1e-2 --accuracy 3e-3 --iteration_limit=true --simulation_time 100 --fixed_step=true --fixed_tolerance 1e-3 --time_stepping=false --visualize=false 1> /dev/null
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=1e-2 --accuracy 3e-3 --iteration_limit=true --simulation_time 100 --fixed_step=true --fixed_tolerance 1e-3 --time_stepping=true --visualize=false 1> /dev/null


