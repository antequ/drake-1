
% change to boxnlrk2err .. for rk2 as ground truth
% change to boxnlerr .. for rk3 as ground truth
% change to boxnlraderr .. for radau as ground truth
scheme_name = 'Radau 3, friction';
scheme_fname = 'radau';
fileprefix = ['/home/antequ/code/github/drake/bazel-bin/examples/box/integrator_benchmark.runfiles/drake/box' scheme_fname 'lerr' ];
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

time = result25(:, 1);
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

fricerr25 = abs(diff25(:, 3));
fricerr3 = abs(diff3(:, 3));
fricerr35 = abs(diff35(:, 3));
fricerr4 = abs(diff4(:, 3));
fricerr45 = abs(diff45(:, 3));
%fricerr47 = abs(diff47(:, 3));
%fricerr48 = abs(diff48(:, 3));
fricerr5 = abs(diff5(:, 3));
fricerr55 = abs(diff55(:, 3));
fricerr6 = abs(diff6(:, 3));


t = 2.1;
%change to t=2.3 for stiction. t=2.1 for sliding
q_e25 = interp1(time,stateerr25,t);
q_e3 = interp1(time,stateerr3,t);
q_e35 = interp1(time,stateerr35,t);
q_e4 = interp1(time,stateerr4,t);
q_e45 = interp1(time,stateerr45,t);
%q_e47 = interp1(time,stateerr47,t);
%q_e48 = interp1(time,stateerr48,t);
q_e5 = interp1(time,stateerr5,t);
q_e55 = interp1(time,stateerr55,t);
q_e6 = interp1(time,stateerr6,t);


q1_e25 = abs(interp1(time,diff25(:,1),t));
q1_e3 = abs(interp1(time,diff3(:,1),t));
q1_e35 = abs(interp1(time,diff35(:,1),t));
q1_e4 = abs(interp1(time,diff4(:,1),t));
q1_e45 = abs(interp1(time,diff45(:,1),t));
%q1_e47 = abs(interp1(time,diff47(:,1),t));
%q1_e48 = abs(interp1(time,diff48(:,1),t));
q1_e5 = abs(interp1(time,diff5(:,1),t));
q1_e55 = abs(interp1(time,diff55(:,1),t));
q1_e6 = abs(interp1(time,diff6(:,1),t));


q2_e25 = abs(interp1(time,diff25(:,2),t));
q2_e3 = abs(interp1(time,diff3(:,2),t));
q2_e35 = abs(interp1(time,diff35(:,2),t));
q2_e4 = abs(interp1(time,diff4(:,2),t));
q2_e45 = abs(interp1(time,diff45(:,2),t));
%q2_e47 = abs(interp1(time,diff47(:,2),t));
%q2_e48 = abs(interp1(time,diff48(:,2),t));
q2_e5 = abs(interp1(time,diff5(:,2),t));
q2_e55 = abs(interp1(time,diff55(:,2),t));
q2_e6 = abs(interp1(time,diff6(:,2),t));



fr_e25 = abs(interp1(time,diff25(:,3),t));
fr_e3 = abs(interp1(time,diff3(:,3),t));
fr_e35 = abs(interp1(time,diff35(:,3),t));
fr_e4 = abs(interp1(time,diff4(:,3),t));
fr_e45 = abs(interp1(time,diff45(:,3),t));
%fr_e47 = abs(interp1(time,diff47(:,3),t));
%fr_e48 = abs(interp1(time,diff48(:,3),t));
fr_e5 = abs(interp1(time,diff5(:,3),t));
fr_e55 = abs(interp1(time,diff55(:,3),t));
fr_e6 = abs(interp1(time,diff6(:,3),t));


stepsizes = [1e-6 3e-6 1e-5 3e-5 1e-4 3e-4 1e-3 3e-3];
q_errs = [q_e6 q_e55 q_e5 q_e45 q_e4 q_e35 q_e3 q_e25];
q1_errs = [q1_e6 q1_e55 q1_e5 q1_e45 q1_e4 q1_e35 q1_e3 q1_e25];
q2_errs = [q2_e6 q2_e55 q2_e5 q2_e45 q2_e4 q2_e35 q2_e3 q2_e25];
fr_errs = [fr_e6 fr_e55 fr_e5 fr_e45 fr_e4 fr_e35 fr_e3 fr_e25];
if(0)
figure(); loglog(stepsizes, q_errs, '-o');%, h, h.^2*1e4)
title(['state errors at t= ' num2str(t) ' s']);

figure(); loglog(stepsizes, q1_errs, '-o');%, h, h.^2*1e4)
title(['position errors at t=' num2str(t) ' s']);

figure(); loglog(stepsizes, q2_errs, '-o');%, h, h.^2*1e4)
title(['velocity errors at t=' num2str(t) ' s']);

figure(); loglog(stepsizes, fr_errs, '-o');%, h, h.^2*1e4)
title(['friction errors at t=' num2str(t) ' s']);
end


t = 2.3;
%change to t=2.3 for stiction. t=2.1 for sliding
q_e25 = interp1(time,stateerr25,t);
q_e3 = interp1(time,stateerr3,t);
q_e35 = interp1(time,stateerr35,t);
q_e4 = interp1(time,stateerr4,t);
q_e45 = interp1(time,stateerr45,t);
%q_e47 = interp1(time,stateerr47,t);
%q_e48 = interp1(time,stateerr48,t);
q_e5 = interp1(time,stateerr5,t);
q_e55 = interp1(time,stateerr55,t);
q_e6 = interp1(time,stateerr6,t);


q1_e25 = abs(interp1(time,diff25(:,1),t));
q1_e3 = abs(interp1(time,diff3(:,1),t));
q1_e35 = abs(interp1(time,diff35(:,1),t));
q1_e4 = abs(interp1(time,diff4(:,1),t));
q1_e45 = abs(interp1(time,diff45(:,1),t));
%q1_e47 = abs(interp1(time,diff47(:,1),t));
%q1_e48 = abs(interp1(time,diff48(:,1),t));
q1_e5 = abs(interp1(time,diff5(:,1),t));
q1_e55 = abs(interp1(time,diff55(:,1),t));
q1_e6 = abs(interp1(time,diff6(:,1),t));


q2_e25 = abs(interp1(time,diff25(:,2),t));
q2_e3 = abs(interp1(time,diff3(:,2),t));
q2_e35 = abs(interp1(time,diff35(:,2),t));
q2_e4 = abs(interp1(time,diff4(:,2),t));
q2_e45 = abs(interp1(time,diff45(:,2),t));
%q2_e47 = abs(interp1(time,diff47(:,2),t));
%q2_e48 = abs(interp1(time,diff48(:,2),t));
q2_e5 = abs(interp1(time,diff5(:,2),t));
q2_e55 = abs(interp1(time,diff55(:,2),t));
q2_e6 = abs(interp1(time,diff6(:,2),t));



fr_e25 = abs(interp1(time,diff25(:,3),t));
fr_e3 = abs(interp1(time,diff3(:,3),t));
fr_e35 = abs(interp1(time,diff35(:,3),t));
fr_e4 = abs(interp1(time,diff4(:,3),t));
fr_e45 = abs(interp1(time,diff45(:,3),t));
%fr_e47 = abs(interp1(time,diff47(:,3),t));
%fr_e48 = abs(interp1(time,diff48(:,3),t));
fr_e5 = abs(interp1(time,diff5(:,3),t));
fr_e55 = abs(interp1(time,diff55(:,3),t));
fr_e6 = abs(interp1(time,diff6(:,3),t));


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


l1_e25 = norm(stateerr25,1);
l1_e3 = norm(stateerr3,1);
l1_e35 = norm(stateerr35,1);
l1_e4 = norm(stateerr4,1);
l1_e45 = norm(stateerr45,1);
%l1_e47 = norm(stateerr47,1);
%l1_e48 = norm(stateerr48,1);
l1_e5 = norm(stateerr5,1);
l1_e55 = norm(stateerr55,1);
l1_e6 = norm(stateerr6,1);


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


stepsizes = [1e-6 3e-6 1e-5 3e-5 1e-4 3e-4 1e-3 3e-3];
q_errs = [q_e6 q_e55 q_e5 q_e45 q_e4 q_e35 q_e3 q_e25];
q1_errs = [q1_e6 q1_e55 q1_e5 q1_e45 q1_e4 q1_e35 q1_e3 q1_e25];
q2_errs = [q2_e6 q2_e55 q2_e5 q2_e45 q2_e4 q2_e35 q2_e3 q2_e25];
fr_errs = [fr_e6 fr_e55 fr_e5 fr_e45 fr_e4 fr_e35 fr_e3 fr_e25];
l0_errs = [l0_e6 l0_e55 l0_e5 l0_e45 l0_e4 l0_e35 l0_e3 l0_e25];
l1_errs = [l1_e6 l1_e55 l1_e5 l1_e45 l1_e4 l1_e35 l1_e3 l1_e25];
l2_errs = [l2_e6 l2_e55 l2_e5 l2_e45 l2_e4 l2_e35 l2_e3 l2_e25];

if(0)
figure(); loglog(stepsizes, q_errs, '-o');%, h, h.^2*1e4)
title(['state errors at t= ' num2str(t) ' s']);

figure(); loglog(stepsizes, q1_errs, '-o');%, h, h.^2*1e4)
title(['position errors at t=' num2str(t) ' s']);

figure(); loglog(stepsizes, q2_errs, '-o');%, h, h.^2*1e4)
title(['velocity errors at t=' num2str(t) ' s']);

figure(); loglog(stepsizes, fr_errs, '-o');%, h, h.^2*1e4)
title(['friction errors at t=' num2str(t) ' s']);

figure(); loglog(stepsizes, l0_errs, '-o');%, h, h.^2*1e4)
title('L-inf norm of state errors across all time');

figure(); loglog(stepsizes, l1_errs, '-o');%, h, h.^2*1e4)
title('L1 norm of state errors across all time');

figure(); loglog(stepsizes, l2_errs, '-o');%, h, h.^2*1e4)
title('L2 norm of state errors across all time');
end
if(1)
figure(); plot(time, result25(:,[7 10])); title('friction comparison step size 3e-3 vs truth') 
figure(); plot(time, result3(:,[7 10])); title('friction comparison step size 1e-3 vs truth') 
figure(); plot(time, result35(:,[7 10])); title('friction comparison step size 3e-4 vs truth') 
figure(); plot(time, result4(:,[7 10])); title('friction comparison step size 1e-4 vs truth') 
figure(); plot(time, result45(:,[7 10])); title('friction comparison step size 3e-5 vs truth') 
figure(); plot(time, result5(:,[7 10])); title('friction comparison step size 1e-5 vs truth') 
figure(); plot(time, result55(:,[7 10])); title('friction comparison step size 3e-6 vs truth') 
end

%%

% make plot of n_iterations vs error
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

total_time = size(result25,1) * 1.e-2;
if(1)
figure(); loglog(stepsizes, nders, '-o');%, h, h.^2*1e4)
title(['Number of derivative evaluations for ' num2str(total_time) ' s sim']);
end

figure(); loglog(l2_errs, nders, '-o');
xlabel('L2 norm of state errors, global calculated every 1e-2s');
ylabel('Number of Derivative evaluations');
title(['Evaluations vs Error Plot, ' scheme_name]);


