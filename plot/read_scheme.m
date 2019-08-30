% function read scheme
%function [l2_errs, nders, scheme_name, vs_skipped] = read_scheme(scheme_fname, fixed, conv, fnr, itlim, normstyle)


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
    scheme_prefix = 'Implicit Euler';
elseif (strcmp(scheme_fname, 'fixed_implicit_euler'))
    scheme_prefix = 'Radau 1';
elseif (strcmp(scheme_fname, 'bogacki_shampine3'))
    scheme_prefix = 'Bogacki Shampine3';
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

fileprefix = ['/home/antequ/code/github/drake/bazel-bin/examples/box2d/box2d.runfiles/drake/finalbox' scheme_fname strf strc strn stri ];
result1 = nan(251, 10);
result2 = nan(251, 10);
result3 = nan(251, 10);
result4 = nan(251, 10);
result5 = nan(251, 10);
result6 = nan(251, 10);
result7 = nan(251, 10);
result8 = nan(251, 10);

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
stiction_start=23; stiction_end=39;
vsnorm1 = vecnorm(diff1(stiction_start:stiction_end,2),2);
vsnorm2 = vecnorm(diff2(stiction_start:stiction_end,2),2);
vsnorm3 = vecnorm(diff3(stiction_start:stiction_end,2),2);
vsnorm4 = vecnorm(diff4(stiction_start:stiction_end,2),2);
vsnorm5 = vecnorm(diff5(stiction_start:stiction_end,2),2);
vsnorm6 = vecnorm(diff6(stiction_start:stiction_end,2),2);
vsnorm7 = vecnorm(diff7(stiction_start:stiction_end,2),2);
vsnorm8 = vecnorm(diff8(stiction_start:stiction_end,2),2);
vs_threshold = 1e-4; % 2.8e-4 fails; 1.7e-5 is fine
vsnorms = [vsnorm1 vsnorm2 vsnorm3 vsnorm4 vsnorm5 vsnorm6 vsnorm7 vsnorm8];
vs_skipped = vsnorms > vs_threshold;
%end
