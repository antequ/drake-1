% normstyle = 'position_only';
 normstyle = 'velocity_only';
% normstyle = 'both';
[radaufl2, radaufnd, radaufname, radaufs] = read_scheme('radaun', true, normstyle);
[radauilfl2, radauilfnd, radauilfname, radauilfs] = read_scheme('radaufil', true, normstyle);
[radaunfl2, radaunfnd, radaunfname, radaunfs] = read_scheme('radau', false, normstyle);
[radau1fl2, radau1fnd, radau1fname, radau1fs] = read_scheme('fixed_implicit_eulern', true, normstyle);
[radau1ilfl2, radau1ilfnd, radau1ilfname, radau1ilfs] = read_scheme('fixed_implicit_eulerfil', true, normstyle);
[radau1nfl2, radau1nfnd, radau1nfname, radau1nfs] = read_scheme('fixed_implicit_euler', false, normstyle);
[rk3fl2, rk3fnd, rk3fname, rk3fs] = read_scheme('runge_kutta3', true, normstyle);
[rk3nfl2, rk3nfnd, rk3nfname, rk3nfs] = read_scheme('runge_kutta3', false, normstyle);
[rk3ffl2, rk3ffnd, rk3ffname, rk3ffs] = read_scheme('runge_kutta3f', true, normstyle);
[rk3fnfl2, rk3fnfnd, rk3fnfname, rk3fnfs] = read_scheme('runge_kutta3f', false, normstyle);
[rk2fl2, rk2fnd, rk2fname, rk2fs] = read_scheme('runge_kutta2', true, normstyle);
[rk2nfl2, rk2nfnd, rk2nfname, rk2nfs] = read_scheme('runge_kutta2', false, normstyle);
[seefl2, seefnd, seefname, seefs] = read_scheme('semi_explicit_euler', true, normstyle);
[seenfl2, seenfnd, seenfname, seenfs] = read_scheme('semi_explicit_euler', false, normstyle);
[iefl2, iefnd, iefname, iefs] = read_scheme('implicit_euler', true, normstyle);
[ieilfl2, ieilfnd, ieilfname, ieilfs] = read_scheme('implicit_euleril', true, normstyle);
[ienfl2, ienfnd, ienfname, ienfs] = read_scheme('implicit_euler', false, normstyle);
[fiefl2, fiefnd, fiefname, fiefs] = read_scheme('implicit_eulerfn', true, normstyle);
[fienfl2, fienfnd, fienfname, fienfs] = read_scheme('implicit_eulerf', false, normstyle);
[fieilfl2, fieilfnd, fieilfname, fieilfs] = read_scheme('implicit_eulerfil', true, normstyle);
[bsfl2, bsfnd, bsfname, bsfs] = read_scheme('bogacki_shampine3', true, normstyle);
[bsnfl2, bsnfnd, bsnfname, bsnfs] = read_scheme('bogacki_shampine3', false, normstyle);
[bsffl2, bsffnd, bsffname, bsffs] = read_scheme('bogacki_shampine3f', true, normstyle);
[bsfnfl2, bsfnfnd, bsfnfname, bsfnfs] = read_scheme('bogacki_shampine3f', false, normstyle);

fixed_timesteps = [3e-8 1e-7 3e-7 1e-6 3e-6 1e-5 3e-5 1e-4];
if(0)
    figure();
    loglog(fixed_timesteps, radau1fl2, '-o');
    hold on;
    loglog(fixed_timesteps, radaufl2, '-o');
    loglog(fixed_timesteps, fiefl2, '-o');
    loglog(fixed_timesteps, 10^8.5 * fixed_timesteps.^2, '-');
    loglog(fixed_timesteps, 10^3.5 * fixed_timesteps, '-');
    legend('Radau 1', 'Radau 3', 'Implicit Euler Fixed', 'h^2 reference', 'h reference');
    xlabel('timestep size (h) [s]');
    ylabel('velocity RMS error [m/s]');
end
make_plots = false;
if(make_plots)
make_the_plot(radaufl2, radaufnd, radaufname, radaufs);
make_the_plot(radaunfl2, radaunfnd, radaunfname, radaunfs);
make_the_plot(radau1fl2, radau1fnd, radau1fname, radau1fs);
make_the_plot(radau1nfl2, radau1nfnd, radau1nfname, radau1nfs);
make_the_plot(rk3fl2, rk3fnd, rk3fname, rk3fs);
make_the_plot(rk3nfl2, rk3nfnd, rk3nfname, rk3nfs);
make_the_plot(rk3ffl2, rk3ffnd, rk3ffname, rk3ffs);
make_the_plot(rk3fnfl2, rk3fnfnd, rk3fnfname, rk3fnfs);
make_the_plot(rk2fl2, rk2fnd, rk2fname, rk2fs);
make_the_plot(rk2nfl2, rk2nfnd, rk2nfname, rk2nfs);
make_the_plot(seefl2, seefnd, seefname, seefs);
make_the_plot(seenfl2, seenfnd, seenfname, seenfs);
make_the_plot(iefl2, iefnd, iefname, iefs);
make_the_plot(ienfl2, ienfnd, ienfname, ienfs);
make_the_plot(fiefl2, fiefnd, fiefname, fiefs);
make_the_plot(fienfl2, fienfnd, fienfname, fienfs);
make_the_plot(fieilfl2, fieilfnd, fieilfname, fieilfs);
make_the_plot(bsfl2, bsfnd, bsfname, bsfs);
make_the_plot(bsnfl2, bsnfnd, bsnfname, bsnfs);
make_the_plot(bsffl2, bsffnd, bsffname, bsffs);
make_the_plot(bsfnfl2, bsfnfnd, bsfnfname, bsfnfs);
end

figure();
p = loglog(radaufl2, radaufnd, '-o', ...
    radauilfl2, radauilfnd, '-o', ...
    radau1fl2, radau1fnd, '-o', ...
    radau1ilfl2, radau1ilfnd, '-o', ...
    rk3fl2, rk3fnd, '-o', ...
    rk3ffl2, rk3ffnd, '-o', ...
    rk2fl2, rk2fnd, '-o', ...
    seefl2, seefnd, '-o', ...
    iefl2, iefnd, '-o', ...
    ieilfl2, ieilfnd, '-o', ...
    fiefl2, fiefnd, '-o', ...
    fieilfl2, fieilfnd, '-o', ...
    bsfl2, bsfnd, '-o', ...
    bsffl2, bsffnd, '-o');
p(8).Color=[0.32,0.32,0.32];
p(9).Color=[0.2, 0.4, 0.2];
p(10).Color=[0.5, 0.4, 0.2];
hold on;
if(any(radaufs))
    
    q = loglog(radaufl2(radaufs), radaufnd(radaufs), '-o','Color',p(1).Color);
    q(1).MarkerSize = 10;
    q(1).MarkerFaceColor = p(1).Color;
end
if(any(radau1fs))
    
    q = loglog(radau1fl2(radau1fs), radau1fnd(radau1fs), '-o','Color',p(3).Color);
    q(1).MarkerSize = 10;
    q(1).MarkerFaceColor = p(3).Color;
end
if(any(rk3fs))
    q = loglog(rk3fl2(rk3fs), rk3fnd(rk3fs), '-o','Color',p(5).Color);
    q(1).MarkerSize = 10;
    q(1).MarkerFaceColor = p(5).Color;
end
if(any(rk3ffs))
    q = loglog(rk3ffl2(rk3ffs), rk3ffnd(rk3ffs), '-o','Color',p(6).Color);
    q(1).MarkerSize = 10;
    q(1).MarkerFaceColor = p(6).Color;
end
if(any(rk2fs))
    q = loglog(rk2fl2(rk2fs), rk2fnd(rk2fs), '-o','Color',p(7).Color);
    q(1).MarkerSize = 10;
    q(1).MarkerFaceColor = p(7).Color;
end
if(any(seefs))
    q = loglog(seefl2(seefs), seefnd(seefs), '-o','Color',p(8).Color);
    q(1).MarkerSize = 10;
    q(1).MarkerFaceColor = p(8).Color;
end
if(any(iefs))
    q = loglog(iefl2(iefs), iefnd(iefs), '-o', 'Color', p(9).Color);
    q(1).MarkerSize = 10;
    q(1).MarkerFaceColor = p(9).Color;
end
if(any(fiefs))
    q = loglog(fiefl2(fiefs), fiefnd(fiefs), '-o', 'Color', p(11).Color);
    q(1).MarkerSize = 10;
    q(1).MarkerFaceColor = p(11).Color;
end
if(any(fieilfs))
    q = loglog(fieilfl2(fieilfs), fieilfnd(fieilfs), '-o', 'Color', p(12).Color);
    q(1).MarkerSize = 10;
    q(1).MarkerFaceColor = p(12).Color;
end
if(any(bsfs))
    q = loglog(bsfl2(bsfs), bsfnd(bsfs), '-o', 'Color', p(13).Color);
    q(1).MarkerSize = 10;
    q(1).MarkerFaceColor = p(13).Color;
end
if(any(bsffs))
    q = loglog(bsffl2(bsffs), bsffnd(bsffs), '-o', 'Color', p(14).Color);
    q(1).MarkerSize = 10;
    q(1).MarkerFaceColor = p(14).Color;
end
%legend('Radau 3 Fixed', 'Radau 1 Fixed', 'RK3 EC', 'RK3 Fixed', 'RK2 Fixed', 'Semi-Explicit Euler Fixed', 'Implicit Euler EC', 'Implicit Euler Fixed', 'Bogacki Shampine3 EC', 'Bogacki Shampine3 Fixed');
legend('Radau 3 Fixed', 'Radau 3 Fxd It Ltd',...
    'Radau 1 Fixed', 'Radau 1 Fxd It Ltd', ...
    'RK3 EC', 'RK3 Fixed', 'RK2 Fixed', 'Semi-Explicit Euler Fixed',...
    'Implicit Euler EC','Implicit Euler EC It Lim', ...
    'Implicit Euler Fixed', 'Implicit Euler Fxd It Lim',...
    'Bogacki Shampine3 EC', 'Bogacki Shampine3 Fixed');
if(strcmp(normstyle, 'position_only'))
    xlabel('RMS of Position Errors (m), global calculated every 1e-2s');
elseif(strcmp(normstyle, 'velocity_only'))
    xlabel('RMS of Velocity Errors (m/s), global calculated every 1e-2s');
else
    xlabel('RMS of L2 Norm of State Errors, global calculated every 1e-2s');
end
ylabel('Number of Derivative Evaluations');
title(['Evaluations vs Error Plot, with friction']);
for i = 1:size(p,1)
    p(i).MarkerSize = 10;
    
end

figure();q = loglog(radaunfl2, radaunfnd, '-o', radau1nfl2, radau1nfnd, '-o', rk3nfl2, rk3nfnd, '-o',  rk3fnfl2, rk3fnfnd, '-o', rk2nfl2, rk2nfnd, '-o', seenfl2, seenfnd, '-o', ienfl2, ienfnd, '-o', fienfl2, fienfnd, '-o', bsnfl2, bsnfnd, '-o', bsfnfl2, bsfnfnd, '-o');
q(8).Color=[0.32,0.32,0.32];
q(9).Color=[0.2, 0.4, 0.2];
q(10).Color=[0.5, 0.4, 0.2];
for i = 1:size(q,1)
    q(i).MarkerSize= 10;
end
legend('Radau 3 Fixed', 'Radau 1 Fixed', 'RK3 EC', 'RK3 Fixed', 'RK2 Fixed', 'Semi-Explicit Euler Fixed', 'Implicit Euler EC', 'Implicit Euler Fixed', 'Bogacki Shampine3 EC', 'Bogacki Shampine3 Fixed');
if(strcmp(normstyle, 'position_only'))
    xlabel('RMS of Position Errors (m), global calculated every 1e-2s');
elseif(strcmp(normstyle, 'velocity_only'))
    xlabel('RMS of Velocity Errors (m/s), global calculated every 1e-2s');
else
    xlabel('RMS of L2 Norm of State Errors, global calculated every 1e-2s');
end
ylabel('Number of Derivative Evaluations');
title(['Evaluations vs Error Plot, with no friction']);
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
%%
function [l2_errs, nders, scheme_name, vs_skipped] = read_scheme(scheme_fname, friction, normstyle)


%scheme_fname = 'radau';
if(strcmp(scheme_fname, 'radau'))
    scheme_prefix = 'Radau 3 Fixed (Robust)';
elseif(strcmp(scheme_fname, 'radaun'))
    scheme_prefix = 'Radau 3 Fixed';
elseif (strcmp(scheme_fname ,'runge_kutta2'))
    scheme_prefix = 'RK2 Fixed';
elseif (strcmp(scheme_fname ,'semi_explicit_euler'))
    scheme_prefix = 'SEE Fixed';
elseif (strcmp(scheme_fname, 'runge_kutta3'))
    scheme_prefix = 'RK3 EC';
elseif (strcmp(scheme_fname, 'runge_kutta3f'))
    scheme_prefix = 'RK3 Fixed';
elseif (strcmp(scheme_fname, 'implicit_euler'))
    scheme_prefix = 'Implicit Euler EC';
elseif (strcmp(scheme_fname, 'implicit_euleril'))
    scheme_prefix = 'Implicit Euler EC It Ltd';
elseif (strcmp(scheme_fname, 'fixed_implicit_euler'))
    scheme_prefix = 'Radau 1 Fixed (Robust)';
elseif (strcmp(scheme_fname, 'fixed_implicit_eulern'))
    scheme_prefix = 'Radau 1 Fixed';
elseif (strcmp(scheme_fname, 'implicit_eulerf'))
    scheme_prefix = 'Implicit Euler Fixed (Robust)';
elseif (strcmp(scheme_fname, 'implicit_eulerfn'))
    scheme_prefix = 'Implicit Euler Fixed';
elseif (strcmp(scheme_fname, 'implicit_eulerfil'))
    scheme_prefix = 'Implicit Euler, It Limited';
elseif (strcmp(scheme_fname, 'bogacki_shampine3'))
    scheme_prefix = 'Bogacki Shampine3 EC';
elseif (strcmp(scheme_fname, 'bogacki_shampine3f'))
    scheme_prefix = 'Bogacki Shampine3 Fixed';
else
    scheme_prefix = 'Unknown';
end
%friction=true;
if(friction)
    scheme_suffix = ', friction';
    suffix = 'err';
else
    scheme_suffix = ', no friction';
    suffix = 'nf';
end

scheme_name = [scheme_prefix scheme_suffix];

fileprefix = ['/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/box' scheme_fname 'l' suffix ];
result25 = nan(251, 10);
result3 = nan(251, 10);
result35 = nan(251, 10);
result4 = nan(251, 10);
result45 = nan(251, 10);
result5 = nan(251, 10);
result55 = nan(251, 10);
result6 = nan(251, 10);

if(isfile([fileprefix '25.csv']))
    result25 = csvread([fileprefix '25.csv'], 1, 0);
end
if(isfile([fileprefix '3.csv']))
    result3 = csvread([fileprefix '3.csv'], 1, 0);
end
if(isfile([fileprefix '35.csv']))
result35 = csvread([fileprefix '35.csv'], 1, 0);
end
if(isfile([fileprefix '4.csv']))
result4 = csvread([fileprefix '4.csv'], 1, 0);
end
if(isfile([fileprefix '45.csv']))
result45 = csvread([fileprefix '45.csv'], 1, 0);
end
%result47 = csvread([fileprefix '47.csv'], 1, 0);
%result48 = csvread([fileprefix '48.csv'], 1, 0);
if(isfile([fileprefix '5.csv']))
result5 = csvread([fileprefix '5.csv'], 1, 0);
end
if(isfile([fileprefix '55.csv']))
result55 = csvread([fileprefix '55.csv'], 1, 0);
end
if(isfile([fileprefix '6.csv']))
result6 = csvread([fileprefix '6.csv'], 1, 0);
end

diff25 = result25(:,5:7) - result25(:,8:10);
diff3 = result3(:,5:7) - result3(:,8:10);
diff35 = result35(:,5:7) - result35(:,8:10);
diff4 = result4(:,5:7) - result4(:,8:10);
diff45 = result45(:,5:7) - result45(:,8:10);
%diff47 = result47(:,5:7) - result47(:,8:10);
%diff48 = result48(:,5:7) - result48(:,8:10);
diff5 = result5(:,5:7) - result5(:,8:10);
diff55 = result55(:,5:7) - result55(:,8:10);
diff6 = result6(:,5:7) - result6(:,8:10);

normstart = 1; %position
normend = 2; %velocity
if(strcmp(normstyle, 'position_only'))
    normend = 1;
elseif(strcmp(normstyle, 'velocity_only'))
    normstart = 2;
end
stateerr25 = vecnorm(diff25(:, normstart:normend),2, 2);
stateerr3 = vecnorm(diff3(:, normstart:normend),2, 2);
stateerr35 = vecnorm(diff35(:, normstart:normend),2, 2);
stateerr4 = vecnorm(diff4(:, normstart:normend),2, 2);
stateerr45 = vecnorm(diff45(:, normstart:normend),2, 2);
%stateerr47 = vecnorm(diff47(:, 1:2),2, 2);
%stateerr48 = vecnorm(diff48(:, 1:2),2, 2);
stateerr5 = vecnorm(diff5(:, normstart:normend),2, 2);
stateerr55 = vecnorm(diff55(:, normstart:normend),2, 2);
stateerr6 = vecnorm(diff6(:, normstart:normend),2, 2);




l0_e25 = norm(stateerr25,inf);
l0_e3 = norm(stateerr3,inf);
l0_e35 = norm(stateerr35,inf);
l0_e4 = norm(stateerr4,inf);
l0_e45 = norm(stateerr45,inf);
%l0_e47 = norm(stateerr47,inf);
%l0_e48 = norm(stateerr48,inf);
l0_e5 = norm(stateerr5,inf);
l0_e55 = norm(stateerr55,inf);
l0_e6 = norm(stateerr6,inf);
%l2_errs = [l0_e6 l0_e55 l0_e5 l0_e45 l0_e4 l0_e35 l0_e3 l0_e25];

l2_e25 = norm(stateerr25,2);
l2_e3 = norm(stateerr3,2);
l2_e35 = norm(stateerr35,2);
l2_e4 = norm(stateerr4,2);
l2_e45 = norm(stateerr45,2);
%l2_e47 = norm(stateerr47,2);
%l2_e48 = norm(stateerr48,2);
l2_e5 = norm(stateerr5,2);
l2_e55 = norm(stateerr55,2);
l2_e6 = norm(stateerr6,2);
l2_errs = [l2_e6 l2_e55 l2_e5 l2_e45 l2_e4 l2_e35 l2_e3 l2_e25] ./ sqrt(size(stateerr25,1));


nder25 = result25(end, 2);
nder3 = result3(end, 2);
nder35 = result35(end, 2);
nder4 = result4(end, 2);
nder45 = result45(end, 2);
%nder47 = result47(end, 2);
%nder48 = result48(end, 2);
nder5 = result5(end, 2);
nder55 = result55(end, 2);
nder6 = result6(end, 2);
nders = [nder6 nder55 nder5 nder45 nder4 nder35 nder3 nder25];

% does sim capture stiction?
stiction_start=23; stiction_end=39;
vsnorm25 = vecnorm(diff25(stiction_start:stiction_end,2),2);
vsnorm3 = vecnorm(diff3(stiction_start:stiction_end,2),2);
vsnorm35 = vecnorm(diff35(stiction_start:stiction_end,2),2);
vsnorm4 = vecnorm(diff4(stiction_start:stiction_end,2),2);
vsnorm45 = vecnorm(diff45(stiction_start:stiction_end,2),2);
vsnorm5 = vecnorm(diff5(stiction_start:stiction_end,2),2);
vsnorm55 = vecnorm(diff55(stiction_start:stiction_end,2),2);
vsnorm6 = vecnorm(diff6(stiction_start:stiction_end,2),2);
vs_threshold = 1e-4; % 2.8e-4 fails; 1.7e-5 is fine
vsnorms = [vsnorm6 vsnorm55 vsnorm5 vsnorm45 vsnorm4 vsnorm35 vsnorm3 vsnorm25];
vs_skipped = vsnorms > vs_threshold;
end