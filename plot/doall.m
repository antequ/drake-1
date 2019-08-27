
result = csvread('/home/antequ/code/github/drake/bazel-bin/examples/box/passive_simulation.runfiles/drake/boxout5.csv', 1, 0);
q1e5 = interp1(result(:,1),result(:,3),[2.3]);

result = csvread('/home/antequ/code/github/drake/bazel-bin/examples/box/passive_simulation.runfiles/drake/boxout4.csv', 1, 0);
q1e4 = interp1(result(:,1),result(:,3),[2.3]);

result = csvread('/home/antequ/code/github/drake/bazel-bin/examples/box/passive_simulation.runfiles/drake/boxout3.csv', 1, 0);
q1e3 = interp1(result(:,1),result(:,3),[2.3]);

result = csvread('/home/antequ/code/github/drake/bazel-bin/examples/box/passive_simulation.runfiles/drake/boxout2.csv', 1, 0);
q1e2 = interp1(result(:,1),result(:,3),[2.3]);

% ref: 1e-6, 
% 1e-5, 3e-5, 1e-4, 3e-4, 1e-3, 3e-3

result = csvread('/home/antequ/code/github/drake/bazel-bin/examples/box/passive_simulation.runfiles/drake/boxout6.csv', 1, 0);
q1e6 = interp1(result(:,1),result(:,3),[2.3]);

result = csvread('/home/antequ/code/github/drake/bazel-bin/examples/box/passive_simulation.runfiles/drake/boxout5.csv', 1, 0);
q1e5 = interp1(result(:,1),result(:,3),[2.3]);


result = csvread('/home/antequ/code/github/drake/bazel-bin/examples/box/passive_simulation.runfiles/drake/boxout45.csv', 1, 0);
q1e45 = interp1(result(:,1),result(:,3),[2.3]);

result = csvread('/home/antequ/code/github/drake/bazel-bin/examples/box/passive_simulation.runfiles/drake/boxout4.csv', 1, 0);
q1e4 = interp1(result(:,1),result(:,3),[2.3]);


result = csvread('/home/antequ/code/github/drake/bazel-bin/examples/box/passive_simulation.runfiles/drake/boxout35.csv', 1, 0);
q1e35 = interp1(result(:,1),result(:,3),[2.3]);

result = csvread('/home/antequ/code/github/drake/bazel-bin/examples/box/passive_simulation.runfiles/drake/boxout3.csv', 1, 0);
q1e3 = interp1(result(:,1),result(:,3),[2.3]);


result = csvread('/home/antequ/code/github/drake/bazel-bin/examples/box/passive_simulation.runfiles/drake/boxout25.csv', 1, 0);
q1e25 = interp1(result(:,1),result(:,3),[2.3]);


err = [q1e5 q1e45 q1e4 q1e35 q1e3 q1e25] - q1e6;
h = [1e-5 3e-5 1e-4 3e-4 1e-3 3e-3];
