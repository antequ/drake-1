
scheme_fname='implicit_stribeck_global_';
%scheme_fname='implicit_stribeck_global_';


fileprefix = ['/home/antequ/code/github/drake/bazel-bin/examples/simple_gripper/simple_gripper_benchmark.runfiles/drake/' scheme_fname ];
ncol = 38;
nrow = 101;
result1 = nan(nrow, ncol);
result2 = nan(nrow, ncol);
result3 = nan(nrow, ncol);
result4 = nan(nrow, ncol);
result5 = nan(nrow, ncol);
result6 = nan(nrow, ncol);
result7 = nan(nrow, ncol);
result8 = nan(nrow, ncol);
result9 = nan(nrow, ncol);
result10 = nan(nrow, ncol);
result11 = nan(nrow, ncol);

if(isfile([fileprefix '1.csv']))
    %result1 = csvread([fileprefix '1.csv'], 1, 0);
end
if(isfile([fileprefix '2.csv']))
    %result2 = csvread([fileprefix '2.csv'], 1, 0);
end
if(isfile([fileprefix '3.csv']))
result3 = csvread([fileprefix '3.csv'], 1, 0);
end
if(isfile([fileprefix '4.csv']))
result4 = csvread([fileprefix '4.csv'], 1, 0);
end
if(isfile([fileprefix '5.csv']))
result5 = csvread([fileprefix '5.csv'], 1, 0);
end
%result47 = csvread([fileprefix '47.csv'], 1, 0);
%result48 = csvread([fileprefix '48.csv'], 1, 0);
if(isfile([fileprefix '6.csv']))
result6 = csvread([fileprefix '6.csv'], 1, 0);
end
if(isfile([fileprefix '7.csv']))
result7 = csvread([fileprefix '7.csv'], 1, 0);
end
if(isfile([fileprefix '8.csv']))
result8 = csvread([fileprefix '8.csv'], 1, 0);
end
if(isfile([fileprefix '9.csv']))
result9 = csvread([fileprefix '9.csv'], 1, 0);
end
if(isfile([fileprefix '10.csv']))
result10 = csvread([fileprefix '10.csv'], 1, 0);
end
if(isfile([fileprefix '11.csv']))
result11 = csvread([fileprefix '11.csv'], 1, 0);
end

simstate_start = 5;
simstate_end = 21;
constate_start = 22;
constate_end = 38;
diff1 = result1(:,simstate_start:simstate_end) - result1(:,constate_start:constate_end);
diff2 = result2(:,simstate_start:simstate_end) - result2(:,constate_start:constate_end);
diff3 = result3(:,simstate_start:simstate_end) - result3(:,constate_start:constate_end);
diff4 = result4(:,simstate_start:simstate_end) - result4(:,constate_start:constate_end);
diff5 = result5(:,simstate_start:simstate_end) - result5(:,constate_start:constate_end);
%diff47 = result47(:,5:7) - result47(:,8:10);
%diff48 = result48(:,5:7) - result48(:,8:10);
diff6 = result6(:,simstate_start:simstate_end) - result6(:,constate_start:constate_end);
diff7 = result7(:,simstate_start:simstate_end) - result7(:,constate_start:constate_end);
diff8 = result8(:,simstate_start:simstate_end) - result8(:,constate_start:constate_end);
diff9 = result9(:,simstate_start:simstate_end) - result9(:,constate_start:constate_end);
diff10 = result10(:,simstate_start:simstate_end) - result10(:,constate_start:constate_end);
diff11 = result11(:,simstate_start:simstate_end) - result11(:,constate_start:constate_end);

%rotational
normstart = 10;
normend = 12;
% position
%normstart = 1;
%normend = 4;
rstateerr1 = vecnorm(diff1(:, normstart:normend),2, 2);
rstateerr2 = vecnorm(diff2(:, normstart:normend),2, 2);
rstateerr3 = vecnorm(diff3(:, normstart:normend),2, 2);
rstateerr4 = vecnorm(diff4(:, normstart:normend),2, 2);
rstateerr5 = vecnorm(diff5(:, normstart:normend),2, 2);
%stateerr47 = vecnorm(diff47(:, 1:2),2, 2);
%stateerr48 = vecnorm(diff48(:, 1:2),2, 2);
rstateerr6 = vecnorm(diff6(:, normstart:normend),2, 2);
rstateerr7 = vecnorm(diff7(:, normstart:normend),2, 2);
rstateerr8 = vecnorm(diff8(:, normstart:normend),2, 2);
rstateerr9 = vecnorm(diff9(:, normstart:normend),2, 2);
rstateerr10 = vecnorm(diff10(:, normstart:normend),2, 2);
rstateerr11 = vecnorm(diff11(:, normstart:normend),2, 2);

%translational
normstart = 13;
normend = 17;
% position
%normstart = 5;
%normend = 9;
tstateerr1 = vecnorm(diff1(:, normstart:normend),2, 2);
tstateerr2 = vecnorm(diff2(:, normstart:normend),2, 2);
tstateerr3 = vecnorm(diff3(:, normstart:normend),2, 2);
tstateerr4 = vecnorm(diff4(:, normstart:normend),2, 2);
tstateerr5 = vecnorm(diff5(:, normstart:normend),2, 2);
%stateerr47 = vecnorm(diff47(:, 1:2),2, 2);
%stateerr48 = vecnorm(diff48(:, 1:2),2, 2);
tstateerr6 = vecnorm(diff6(:, normstart:normend),2, 2);
tstateerr7 = vecnorm(diff7(:, normstart:normend),2, 2);
tstateerr8 = vecnorm(diff8(:, normstart:normend),2, 2);
tstateerr9 = vecnorm(diff9(:, normstart:normend),2, 2);
tstateerr10 = vecnorm(diff10(:, normstart:normend),2, 2);
tstateerr11 = vecnorm(diff11(:, normstart:normend),2, 2);

time = result11(:, 1);

if(0)
figure(); plot(time, result1(:,[6 9])); title([scheme_fname ' velocity comparison largest step vs truth']) 
figure(); plot(time, result2(:,[6 9])); title([scheme_fname ' velocity comparison second largest step vs truth'])
figure(); plot(time, result3(:,[6 9])); title([scheme_fname ' velocity comparison third largest step vs truth'])
figure(); plot(time, result4(:,[6 9])); title([scheme_fname ' velocity comparison fourth largest step vs truth'])
figure(); plot(time, result5(:,[6 9])); title([scheme_fname ' velocity comparison fifth largest step vs truth'])
figure(); plot(time, result6(:,[6 9])); title([scheme_fname ' velocity comparison sixth largest step vs truth'])
figure(); plot(time, result7(:,[6 9])); title([scheme_fname ' velocity comparison seventh largest step vs truth'])
figure(); plot(time, result8(:,[6 9])); title([scheme_fname ' velocity comparison eighth largest step vs truth'])
end

if(0)
nder1 = result1(end, 2);
nder2 = result2(end, 2);
nder3 = result3(end, 2);
nder4 = result4(end, 2);
nder5 = result5(end, 2);
%nder47 = result47(end, 2);
%nder48 = result48(end, 2);
nder6 = result6(end, 2);
nder7 = result7(end, 2);
nder8 = result8(end, 2);
nders = [nder1 nder2 nder3 nder4 nder5 nder6 nder7 nder8];
end
hs = [2.5e-2 1e-2 3e-3 1e-3 3e-4 1e-4 3e-5 1e-5 3e-6 1e-6 3e-7];

l2_re1 = norm(rstateerr1,2);
l2_re2 = norm(rstateerr2,2);
l2_re3 = norm(rstateerr3,2);
l2_re4 = norm(rstateerr4,2);
l2_re5 = norm(rstateerr5,2);
%l2_e47 = norm(stateerr47,2);
%l2_e48 = norm(stateerr48,2);
l2_re6 = norm(rstateerr6,2);
l2_re7 = norm(rstateerr7,2);
l2_re8 = norm(rstateerr8,2);
l2_re9 = norm(rstateerr9,2);
l2_re10 = norm(rstateerr10,2);
l2_re11 = norm(rstateerr11,2);
l2_rerrs = [l2_re1 l2_re2 l2_re3 l2_re4 l2_re5 l2_re6 l2_re7 l2_re8 l2_re9 l2_re10 l2_re11] ./ sqrt(size(rstateerr11,1));

l2_te1 = norm(tstateerr1,2);
l2_te2 = norm(tstateerr2,2);
l2_te3 = norm(tstateerr3,2);
l2_te4 = norm(tstateerr4,2);
l2_te5 = norm(tstateerr5,2);
%l2_e47 = norm(stateerr47,2);
%l2_e48 = norm(stateerr48,2);
l2_te6 = norm(tstateerr6,2);
l2_te7 = norm(tstateerr7,2);
l2_te8 = norm(tstateerr8,2);
l2_te9 = norm(tstateerr9,2);
l2_te10 = norm(tstateerr10,2);
l2_te11 = norm(tstateerr11,2);
l2_terrs = [l2_te1 l2_te2 l2_te3 l2_te4 l2_te5 l2_te6 l2_te7 l2_te8 l2_te9 l2_te10 l2_te11] ./ sqrt(size(tstateerr11,1));


%% Format plot
T = 0.5; %period
a = 0.15; %amplitude
width = 230.4;
height = 230.4;
fig1 = figure('Units', 'points', 'Position', [1000 1000 width height],'PaperUnits', 'points','PaperSize',[width+8 height+8]);

p = loglog(hs / T ,  T* l2_rerrs, '-ko', ...
    hs / T, T * l2_terrs / a, '-ks', ...
    [1e-7 1e-2] / T, 45 / T * [1e-7 1e-2], '--k', ...
    'MarkerSize', 4.5, 'LineWidth', 1);

set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 8)
xlim([2e-7, 1e-2]);
ylim([3e-6, 1e0]);
%xticks([1e-2, 1e-1, 1, 1e1, 1e2]);
%yticks([1e3, 1e4, 1e5, 1e6, 1e7]);
grid on;
set(gca, 'xminorgrid', 'off');
set(gca, 'yminorgrid', 'off');


lgd = legend('Rotational Velocity Error', 'Translational Velocity Error', 'Reference First-Order Line');
set(lgd, 'FontSize', 8)
lgd.Location = 'northwest';

xlabel("Normalized Step Size ({\ith/T}) [-]")
ylabel("Normalized Global Error ({\ite_\omegaT}, {\ite_vT/a}) [-]")
%title("Simple Gripper TAMSI convergence");
saveas(fig1, '/home/antequ/integratorplots/SimpleGripperTAMSIConvergence.pdf')

%figure();
%loglog(hs, l2_terrs);
%axis tight
%title("translational errors");