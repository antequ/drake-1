% normstyle = 'position_only';
 normstyle = 'velocity_only';
% normstyle = 'both';
rk2 = read_scheme('runge_kutta2', true, true, false, false, normstyle);
rk3 = read_scheme('runge_kutta3', false, true, false, false, normstyle);

rk2succeeded=rk2;
rk2succeeded{1} = rk2{1}(6:8);
rk2succeeded{2} = rk2{2}(6:8); % index 5 also failed
rk2failed=rk2;
rk2failed{1}=rk2{1}(1:6);
rk2failed{2}=rk2{2}(1:6); % index 6 succeeded

rk3succeeded=rk3;
rk3succeeded{1} = rk3{1}(4:8);
rk3succeeded{2} = rk3{2}(4:8); % index 4 succeeded
rk3failed=rk3;
rk3failed{1}=rk3{1}(1:4); % index 3 also failed
rk3failed{2}=rk3{2}(1:4);

%radau3 = read_scheme('radau', true, true, true, true, normstyle);
%iefn = read_scheme('implicit_euler', true, true, true, true, normstyle);
ieecqn = read_scheme('implicit_euler', false, true, false, true, normstyle);


%%
% combined plot
width = 230.4;
height = 230.4;
fig1 = figure('Units', 'points', 'Position', [1000 1000 width height],'PaperUnits', 'points','PaperSize',[width+8 height+8]);
v_s = 1e-4;
p = loglog(rk2succeeded{1}./v_s, rk2succeeded{2}, '-ks', ...
    rk3succeeded{1}./v_s, rk3succeeded{2}, '-kd', ...
    ieecqn{1}./v_s, ieecqn{2}, '-k^',...
    rk2failed{1}./v_s, rk2failed{2}, '-ks', ...
    rk3failed{1}./v_s, rk3failed{2}, '-kd', ...
    'MarkerSize', 6, 'LineWidth', 1);
    %radau3{1}./v_s, radau3{2}, '-k*',...
    %iefn{1}./v_s, iefn{2}, '-ko',...
p(1).MarkerFaceColor=[0.5,0.5,0.5];
p(2).MarkerFaceColor=[0.5,0.5,0.5];
p(3).MarkerFaceColor=[0.5,0.5,0.5];
%p(6).MarkerFaceColor=[0.5,0.5,0.5];
%p(7).MarkerFaceColor=[0.5,0.5,0.5];
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 8)
xlim([1e-9, 1e3]);
ylim([8e2, 1e7]);
xticks([1e-9,1e-6,1e-3,1,1e3]);
%yticks([1e3, 1e4, 1e5, 1e6, 1e7]);
grid on;
%set(gca, 'xminorgrid', 'off');
set(gca, 'yminorgrid', 'off');


%lgd = legend('RK2', 'RK2 Unstable', 'RK3+EC', 'RK3+EC Unstable', 'Radau3+FN+TALS','IE+FN+TALS','IE+EC+QN+TALS' );
lgd = legend('RK2',  'RK3+EC', 'IE+EC+QN+TALS' );
set(lgd, 'FontSize', 8)
lgd.Location = 'southwest';
if(strcmp(normstyle, 'position_only'))
    xlabel('RMS of Position Errors (m), global calculated every 1e-2s');
elseif(strcmp(normstyle, 'velocity_only'))
    xlabel('Slip Velocity Error ({\ite_v / v_s}) [-]');
else
    xlabel('RMS of L2 Norm of State Errors, global calculated every 1e-2s');
end
ylabel('Number of Function Evaluations [-]');
%title(['Work-Precision Plot of Implicit Euler Point Contact']);
saveas(fig1, '/home/antequ/integratorplots/WorkPrecisionExplicitImplicitIntegrators.pdf')
% bottom right
%%

function [] = make_the_plot(l2_errs, nders, scheme_name, vs_skipped)

figure(); 
p = loglog(l2_errs, nders, '-o');
if(any(vs_skipped))
    hold on
    q = loglog(l2_errs(vs_skipped), nders(vs_skipped), '-+', 'Color', p(1).Color);
end
xlabel('L2 norm of state errors, global calculated every 1e-2s');
ylabel('Number of Derivative evaluations');
title(['Evaluations vs Error Plot, ' scheme_name]);

end
%% read scheme
% returninfo gives l2 err, num ders, scheme name, vs skipped
function returninfo = read_scheme(scheme_fname, fixed, conv, fnr, itlim,  normstyle)


%scheme_fname = 'radau';
if(strcmp(scheme_fname, 'radau'))
    scheme_prefix = 'Radau 3';
elseif(strcmp(scheme_fname, 'radau1'))
    scheme_prefix = 'Radau 1';
elseif(strcmp(scheme_fname, 'radau3'))
    scheme_prefix = 'Radau 3';
elseif (strcmp(scheme_fname ,'runge_kutta2'))
    scheme_prefix = 'RK2';
elseif (strcmp(scheme_fname ,'semi_explicit_euler'))
    scheme_prefix = 'SEE';
elseif (strcmp(scheme_fname, 'runge_kutta3'))
    scheme_prefix = 'RK3';
elseif (strcmp(scheme_fname, 'runge_kutta3f'))
    scheme_prefix = 'RK3';
elseif (strcmp(scheme_fname, 'implicit_euler'))
    scheme_prefix = 'IE';
elseif (strcmp(scheme_fname, 'fixed_implicit_euler'))
    scheme_prefix = 'Radau 1';
elseif (strcmp(scheme_fname, 'bogacki_shampine3'))
    scheme_prefix = 'BS3';
else
    scheme_prefix = 'Unknown';
end
%friction=true;
if(fixed)
    scheme_prefix = [scheme_prefix ' Fx'];
    strf = '_fx';
else
    scheme_prefix = [scheme_prefix ' EC'];
    strf = '_ec';
end

if(conv)
    scheme_prefix = [scheme_prefix ' CC'];
    strc = '_cc';
else
    scheme_prefix = [scheme_prefix ' TR'];
    strc = '_tr';
end

if(fnr)
    scheme_prefix = [scheme_prefix ' FN'];
    strn = '_fn';
else
    scheme_prefix = [scheme_prefix ' QN'];
    strn = '_qn';
end


if(itlim)
    scheme_prefix = [scheme_prefix ' TA'];
    stri = '_ta';
else
    scheme_prefix = [scheme_prefix ' DF'];
    stri = '_df';
end
scheme_name = scheme_prefix ;

fileprefix = ['/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/finalbox20' scheme_fname strf strc strn stri ];
result1 = nan(101, 10);
result2 = nan(101, 10);
result3 = nan(101, 10);
result4 = nan(101, 10);
result5 = nan(101, 10);
result6 = nan(101, 10);
result7 = nan(101, 10);
result8 = nan(101, 10);

if(isfile([fileprefix '1.csv']))
    result1 = csvread([fileprefix '1.csv'], 1, 0);
end
if(isfile([fileprefix '2.csv']))
    result2 = csvread([fileprefix '2.csv'], 1, 0);
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

diff1 = result1(:,5:7) - result1(:,8:10);
diff2 = result2(:,5:7) - result2(:,8:10);
diff3 = result3(:,5:7) - result3(:,8:10);
diff4 = result4(:,5:7) - result4(:,8:10);
diff5 = result5(:,5:7) - result5(:,8:10);
%diff47 = result47(:,5:7) - result47(:,8:10);
%diff48 = result48(:,5:7) - result48(:,8:10);
diff6 = result6(:,5:7) - result6(:,8:10);
diff7 = result7(:,5:7) - result7(:,8:10);
diff8 = result8(:,5:7) - result8(:,8:10);

normstart = 1; %position
normend = 2; %velocity
if(strcmp(normstyle, 'position_only'))
    normend = 1;
elseif(strcmp(normstyle, 'velocity_only'))
    normstart = 2;
end
stateerr1 = vecnorm(diff1(:, normstart:normend),2, 2);
stateerr2 = vecnorm(diff2(:, normstart:normend),2, 2);
stateerr3 = vecnorm(diff3(:, normstart:normend),2, 2);
stateerr4 = vecnorm(diff4(:, normstart:normend),2, 2);
stateerr5 = vecnorm(diff5(:, normstart:normend),2, 2);
%stateerr47 = vecnorm(diff47(:, 1:2),2, 2);
%stateerr48 = vecnorm(diff48(:, 1:2),2, 2);
stateerr6 = vecnorm(diff6(:, normstart:normend),2, 2);
stateerr7 = vecnorm(diff7(:, normstart:normend),2, 2);
stateerr8 = vecnorm(diff8(:, normstart:normend),2, 2);

time = result8(:, 1);
if(strcmp(scheme_fname, 'runge_kutta2'))
figure(); plot(time, result1(:,[7 10])); title([scheme_fname ' friction comparison step size 2.5e-2 vs truth']) 
figure(); plot(time, result2(:,[7 10])); title([scheme_fname ' friction comparison step size 1e-2 vs truth']) 
figure(); plot(time, result3(:,[7 10])); title([scheme_fname ' friction comparison step size 3e-3 vs truth']) 
figure(); plot(time, result4(:,[7 10])); title([scheme_fname ' friction comparison step size 1e-3 vs truth']) 
figure(); plot(time, result5(:,[7 10])); title([scheme_fname ' friction comparison step size 3e-4 vs truth']) 
figure(); plot(time, result6(:,[7 10])); title([scheme_fname ' friction comparison step size 1e-4 vs truth']) 
figure(); plot(time, result7(:,[7 10])); title([scheme_fname ' friction comparison step size 3e-5 vs truth'])
end
if(strcmp(scheme_fname, 'runge_kutta2'))
%figure(); plot(time, result1(:,[6 9])); title([scheme_fname ' velocity comparison largest step vs truth']) 
width = 230.4;
height = 230.4;
fig1 = figure('Units', 'points', 'Position', [1000 1000 width height],'PaperUnits', 'points','PaperSize',[width+8 height+8]);
v_s = 1e-4;
p=plot(time, result1(:,9), '-k',...
    time, result1(:,6), '-k');
p(1).Color=[0.47,0.47,0.47];
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 8)
%xlim([1e-9, 1e3]);
ylim([-0.5, 0.4]);
%xticks([1e-9,1e-6,1e-3,1,1e3]);
%yticks([1e3, 1e4, 1e5, 1e6, 1e7]);
%grid on;
%set(gca, 'xminorgrid', 'off');
set(gca, 'ygrid', 'on');
set(gca, 'yminorgrid', 'off');

lgd = legend('Control', 'RK2, 25 ms');
set(lgd, 'FontSize', 8)
lgd.Location = 'south';
xlabel('Time [s]');
ylabel('Slip Velocity [m/s]');
%title(['Work-Precision Plot of Implicit Euler Point Contact']);
saveas(fig1, '/home/antequ/integratorplots/RK2VelocityComparison.pdf')





figure(); plot(time, result2(:,[6 9])); title([scheme_fname ' velocity comparison second largest step vs truth'])
figure(); plot(time, result3(:,[6 9])); title([scheme_fname ' velocity comparison third largest step vs truth'])
figure(); plot(time, result4(:,[6 9])); title([scheme_fname ' velocity comparison fourth largest step vs truth'])
figure(); plot(time, result5(:,[6 9])); title([scheme_fname ' velocity comparison fifth largest step vs truth'])
figure(); plot(time, result6(:,[6 9])); title([scheme_fname ' velocity comparison sixth largest step vs truth'])
figure(); plot(time, result7(:,[6 9])); title([scheme_fname ' velocity comparison seventh largest step vs truth'])
figure(); plot(time, result8(:,[6 9])); title([scheme_fname ' velocity comparison eighth largest step vs truth'])
end

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


l2_e1 = norm(stateerr1,2);
l2_e2 = norm(stateerr2,2);
l2_e3 = norm(stateerr3,2);
l2_e4 = norm(stateerr4,2);
l2_e5 = norm(stateerr5,2);
%l2_e47 = norm(stateerr47,2);
%l2_e48 = norm(stateerr48,2);
l2_e6 = norm(stateerr6,2);
l2_e7 = norm(stateerr7,2);
l2_e8 = norm(stateerr8,2);
l2_errs = [l2_e1 l2_e2 l2_e3 l2_e4 l2_e5 l2_e6 l2_e7 l2_e8] ./ sqrt(size(stateerr8,1));


% does sim capture stiction?
stiction_start=9; stiction_end=17;
disp(time(stiction_start));
disp(time(stiction_end));
nstiction = stiction_end - stiction_start + 1;
vsnorm1 = vecnorm(diff1(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm2 = vecnorm(diff2(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm3 = vecnorm(diff3(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm4 = vecnorm(diff4(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm5 = vecnorm(diff5(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm6 = vecnorm(diff6(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm7 = vecnorm(diff7(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm8 = vecnorm(diff8(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vs_threshold = 5e-5; %empirical
vsnorms = [vsnorm1 vsnorm2 vsnorm3 vsnorm4 vsnorm5 vsnorm6 vsnorm7 vsnorm8];
vs_skipped = vsnorms > vs_threshold;
returninfo = {l2_errs, nders, scheme_name, vs_skipped};
end
