# continuous iteration limited, 3ms time steps
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=3e-3 --time_stepping=false --accuracy 1e-3 --iteration_limit=true --simulation_time 30 --fixed_step=true --fixed_tolerance 3e-4 1> /dev/null
# discrete implicit stribeck, 3ms time steps
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=3e-3 --time_stepping=true --accuracy 1e-3 --iteration_limit=true --simulation_time 30 --fixed_step=true --fixed_tolerance 3e-4 1> /dev/null#!/bin/bash
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

#turn off visualization, 100 s simulation
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=1e-2 --accuracy 3e-3 --iteration_limit=true --simulation_time 100 --fixed_step=true --fixed_tolerance 1e-3 --time_stepping=false --visualize=false 1> /dev/null
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=1e-2 --accuracy 3e-3 --iteration_limit=true --simulation_time 100 --fixed_step=true --fixed_tolerance 1e-3 --time_stepping=true --visualize=false 1> /dev/null


# discrete implicit stribeck, 3ms time steps, 100s
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=3e-3 --time_stepping=true --accuracy 1e-3 --iteration_limit=true --simulation_time 100 --fixed_step=true --fixed_tolerance 3e-4 1> /dev/null
sleep 4;
# continuous iteration limited, 3ms time steps, 100s
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=3e-3 --time_stepping=false --accuracy 1e-3 --iteration_limit=true --simulation_time 100 --fixed_step=true --fixed_tolerance 3e-4 1> /dev/null
sleep 4;
# continuous no iteration limiter, error controlled, 3ms time steps, 100s
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=3e-3 --time_stepping=false --accuracy 3e-4 --iteration_limit=false --simulation_time 100 --fixed_step=false  #1> /dev/null

# continuous it limited, ec, 3ms ts, 100s
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='point' --target_realtime_rate=0 --max_time_step=3e-3 --time_stepping=false --accuracy 3e-4 --iteration_limit=true --simulation_time 100 --fixed_step=false  


# continuous hydroelastics 3ms fixed ts no it limiter - show
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='hydroelastic' --target_realtime_rate=0 --max_time_step=3e-3 --time_stepping=false --accuracy 1e-3 --iteration_limit=false --simulation_time 30 --fixed_step=true --fixed_tolerance 3e-4


# continuous hydroelastics 1ms fixed ts no it limit - show
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='hydroelastic' --target_realtime_rate=0 --max_time_step=1e-3 --time_stepping=false --accuracy 1e-3 --iteration_limit=false --simulation_time 30 --fixed_step=true --fixed_tolerance 1e-4 


# continuous hydroelastics 0.3ms fixed ts no it limit
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='hydroelastic' --target_realtime_rate=0 --max_time_step=3e-4 --time_stepping=false --accuracy 1e-3 --iteration_limit=false --simulation_time 30 --fixed_step=true --fixed_tolerance 1e-4 

# continuous hydroelastics error controlled, max 3ms time steps - show
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='hydroelastic' --target_realtime_rate=0 --max_time_step=3e-3 --time_stepping=false --accuracy 3e-4 --iteration_limit=false --simulation_time 30 --fixed_step=false 

# continuous hydroelastics error controlled, max 3ms time steps, it limited
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='hydroelastic' --target_realtime_rate=0 --max_time_step=3e-3 --time_stepping=false --accuracy 3e-4 --iteration_limit=true --simulation_time 30 --fixed_step=false

# 
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='hydroelastic' --target_realtime_rate=0 --max_time_step=3e-2 --time_stepping=false --accuracy 1e-4 --iteration_limit=false --simulation_time 30 --fixed_step=false --amplitude=0

# continuous hydroelastics 3ms fixed ts it limited - show
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='hydroelastic' --target_realtime_rate=0 --max_time_step=3e-3 --time_stepping=false --accuracy 1e-3 --iteration_limit=true --simulation_time 30 --fixed_step=true --fixed_tolerance 3e-4 

# continuous hydroelastics 1ms fixed ts it limited
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='hydroelastic' --target_realtime_rate=0 --max_time_step=1e-3 --time_stepping=false --accuracy 1e-3 --iteration_limit=true --simulation_time 30 --fixed_step=true --fixed_tolerance 1e-4 

# continuous hydroelastics 0.3ms fixed ts it limited
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='hydroelastic' --target_realtime_rate=0 --max_time_step=3e-4 --time_stepping=false --accuracy 1e-3 --iteration_limit=true --simulation_time 30 --fixed_step=true --fixed_tolerance 1e-4 

# good run hydroelastics: 5ms fixed
time bazel run //examples/simple_gripper:simple_gripper -- --contact_model='hydroelastic' --target_realtime_rate=0 --max_time_step=5e-3 --time_stepping=false --accuracy 1e-3 --iteration_limit=true --simulation_time 30 --fixed_step=true --fixed_tolerance 3e-4 


 cd "/home/antequ/Videos/director/gripperpointcontactitlim"

    ffmpeg -r 30 -i frame_%07d.tiff \
           -vcodec libx264 \
           -preset slow \
           -crf 18 \
           -pix_fmt yuv420p \
           ../gripperpointcontactitlim.mp4

for INDEX in {001 .. 120}
do
	echo $INDEX
done
 cd "/home/antequ/Videos/director/gripperpointcontactitlim"

    ffmpeg -r 30 -i frame_%07d.tiff \
           -vcodec libx264 \
           -preset slow \
           -crf 18 \
           -pix_fmt yuv420p \
           ../gripperpointcontactitlim.mp4

for INDEX in {0001..0050}
do
       cp frame_0000000.tiff frame_000$INDEX.tiff
done

for INDEX in {0171..0282}
do
       cp frame_0000056.tiff frame_000$INDEX.tiff
done

for INDEX in {0519..0630}
do
       cp frame_0000056.tiff frame_000$INDEX.tiff
done

for INDEX in {0172..0276}
do
       cp frame_0000171.tiff frame_000$INDEX.tiff
done

for INDEX in {0520..0624}
do
       cp frame_0000519.tiff frame_000$INDEX.tiff
done


time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 200 --v_stiction_tolerance 1e-4 --integration_scheme="implicit_euler" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=true --accuracy 1e-3 --fixed_tolerance=3e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-2 --visualize=true --iteration_limit=true

time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 200 --v_stiction_tolerance 1e-4 --integration_scheme="implicit_euler" --target_realtime_rate="0" --autodiff=false --max_time_step=1e-2 --fixed_step=true --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-2 --visualize=true --iteration_limit=true --use_discrete_states

time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 200 --v_stiction_tolerance 1e-4 --integration_scheme="implicit_euler" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=true --accuracy 1e-3 --fixed_tolerance=1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-2 --visualize=true --iteration_limit=true

time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 200 --v_stiction_tolerance 1e-4 --integration_scheme="implicit_euler" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=true --accuracy 1e-3 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-2 --visualize=true --iteration_limit=true --use_discrete_states

time /home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d --simulation_time 200 --v_stiction_tolerance 1e-4 --integration_scheme="implicit_euler" --target_realtime_rate="0" --autodiff=false --max_time_step=3e-3 --fixed_step=false --accuracy 3e-3 --fixed_tolerance=1e-4 --truth_integration_scheme="runge_kutta3" --truth_integration_step=1e-2 --visualize=true --iteration_limit=true


