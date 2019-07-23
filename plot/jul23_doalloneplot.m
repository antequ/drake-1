

[radaufl2, radaufnd, radaufname] = read_scheme('radau', true);
[radaunfl2, radaunfnd, radaunfname] = read_scheme('radau', false);
[rk3fl2, rk3fnd, rk3fname] = read_scheme('runge_kutta3', true);
[rk3nfl2, rk3nfnd, rk3nfname] = read_scheme('runge_kutta3', false);
[rk2fl2, rk2fnd, rk2fname] = read_scheme('runge_kutta2', true);
[rk2nfl2, rk2nfnd, rk2nfname] = read_scheme('runge_kutta2', false);
[iefl2, iefnd, iefname] = read_scheme('implicit_euler', true);
[ienfl2, ienfnd, ienfname] = read_scheme('implicit_euler', false);

make_plots = false;
if(make_plots)
make_the_plot(radaufl2, radaufnd, radaufname);
make_the_plot(radaunfl2, radaunfnd, radaunfname);
make_the_plot(rk3fl2, rk3fnd, rk3fname);
make_the_plot(rk3nfl2, rk3nfnd, rk3nfname);
make_the_plot(rk2fl2, rk2fnd, rk2fname);
make_the_plot(rk2nfl2, rk2nfnd, rk2nfname);
make_the_plot(iefl2, iefnd, iefname);
make_the_plot(ienfl2, ienfnd, ienfname);
end

figure();loglog(radaufl2, radaufnd, '-o', rk3fl2, rk3fnd, '-o', rk2fl2, rk2fnd, '-o', iefl2, iefnd, '-o');
legend('Radau 3 fixed', 'RK3 ec', 'RK2 fixed', 'Implicit Euler ec');
xlabel('L2 norm of state errors, global calculated every 1e-2s');
ylabel('Number of Derivative evaluations');
title(['Evaluations vs Error Plot, with friction']);

figure();loglog(radaunfl2, radaunfnd, '-o', rk3nfl2, rk3nfnd, '-o', rk2nfl2, rk2nfnd, '-o', ienfl2, ienfnd, '-o');
legend('Radau 3 fixed', 'RK3 ec', 'RK2 fixed', 'Implicit Euler ec');
xlabel('L2 norm of state errors, global calculated every 1e-2s');
ylabel('Number of Derivative evaluations');
title(['Evaluations vs Error Plot, with no friction']);
%%

function [] = make_the_plot(l2_errs, nders, scheme_name)

figure(); loglog(l2_errs, nders, '-o');
xlabel('L2 norm of state errors, global calculated every 1e-2s');
ylabel('Number of Derivative evaluations');
title(['Evaluations vs Error Plot, ' scheme_name]);

end
%%
function [l2_errs, nders, scheme_name] = read_scheme(scheme_fname, friction)

%scheme_fname = 'radau';
if(strcmp(scheme_fname, 'radau'))
    scheme_prefix = 'Radau 3 Fixed';
elseif (strcmp(scheme_fname ,'runge_kutta2'))
    scheme_prefix = 'RK2 Fixed';
elseif (strcmp(scheme_fname, 'runge_kutta3'))
    scheme_prefix = 'RK3 EC';
elseif (strcmp(scheme_fname, 'implicit_euler'))
    scheme_prefix = 'Implicit Euler EC';
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

fileprefix = ['/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/box' scheme_fname 'l' suffix ];
result25 = csvread([fileprefix '25.csv'], 1, 0);
result3 = csvread([fileprefix '3.csv'], 1, 0);
result35 = csvread([fileprefix '35.csv'], 1, 0);
result4 = csvread([fileprefix '4.csv'], 1, 0);
result45 = csvread([fileprefix '45.csv'], 1, 0);
%result47 = csvread([fileprefix '47.csv'], 1, 0);
%result48 = csvread([fileprefix '48.csv'], 1, 0);
result5 = csvread([fileprefix '5.csv'], 1, 0);
result55 = csvread([fileprefix '55.csv'], 1, 0);
result6 = csvread([fileprefix '6.csv'], 1, 0);

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

stateerr25 = vecnorm(diff25(:, 1:2),2, 2);
stateerr3 = vecnorm(diff3(:, 1:2),2, 2);
stateerr35 = vecnorm(diff35(:, 1:2),2, 2);
stateerr4 = vecnorm(diff4(:, 1:2),2, 2);
stateerr45 = vecnorm(diff45(:, 1:2),2, 2);
%stateerr47 = vecnorm(diff47(:, 1:2),2, 2);
%stateerr48 = vecnorm(diff48(:, 1:2),2, 2);
stateerr5 = vecnorm(diff5(:, 1:2),2, 2);
stateerr55 = vecnorm(diff55(:, 1:2),2, 2);
stateerr6 = vecnorm(diff6(:, 1:2),2, 2);




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
l2_errs = [l2_e6 l2_e55 l2_e5 l2_e45 l2_e4 l2_e35 l2_e3 l2_e25];


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
end