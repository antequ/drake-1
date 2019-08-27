% normstyle = 'position_only';
 normstyle = 'velocity_only';
% normstyle = 'both';
tttt = read_scheme('implicit_euler', true, true, true, true, normstyle);
tttf = read_scheme('implicit_euler', true, true, true, false, normstyle);
ttft = read_scheme('implicit_euler', true, true, false, true, normstyle);
ttff = read_scheme('implicit_euler', true, true, false, false, normstyle);
tftt = read_scheme('implicit_euler', true, false, true, true, normstyle);
tftf = read_scheme('implicit_euler', true, false, true, false, normstyle);
tfft = read_scheme('implicit_euler', true, false, false, true, normstyle);
tfff = read_scheme('implicit_euler', true, false, false, false, normstyle);
fttt = read_scheme('implicit_euler', false, true, true, true, normstyle);
fttf = read_scheme('implicit_euler', false, true, true, false, normstyle);
ftft = read_scheme('implicit_euler', false, true, false, true, normstyle);
ftff = read_scheme('implicit_euler', false, true, false, false, normstyle);
fftt = read_scheme('implicit_euler', false, false, true, true, normstyle);
fftf = read_scheme('implicit_euler', false, false, true, false, normstyle);
ffft = read_scheme('implicit_euler', false, false, false, true, normstyle);
ffff = read_scheme('implicit_euler', false, false, false, false, normstyle);
if(0) 
figure();
p = loglog(tttt{1}, tttt{2}, '-*', ...
    tttf{1}, tttf{2}, '-x', ...
    ttft{1}, ttft{2}, '-*', ...
    ttff{1}, ttff{2}, '-x', ...
    tftt{1}, tftt{2}, '--+', ...
    tftf{1}, tftf{2}, '--o', ...
    tfft{1}, tfft{2}, '--+', ...
    tfff{1}, tfff{2}, '--o');
p(8).Color=[0.32,0.32,0.32];
%p(9).Color=[0.2, 0.4, 0.2];
%p(10).Color=[0.5, 0.4, 0.2];
hold on;
if(any(tttt{4}))
    display([tttt{3} num2str(tttt{4})])
end
if(any(tttf{4}))
    display([tttf{3} num2str(tttf{4})])
end
if(any(ttft{4}))
    display([ttft{3} num2str(ttft{4})])
end
if(any(ttff{4}))
    display([ttff{3} num2str(ttff{4})])
end
if(any(tftt{4}))
    display([tftt{3} num2str(tftt{4})])
end
if(any(tftf{4}))
    display([tftf{3} num2str(tftf{4})])
end
if(any(tfft{4}))
    display([tfft{3} num2str(tfft{4})])
end
if(any(tfff{4}))
    display([tfff{3} num2str(tfff{4})])
end
%legend('Radau 3 Fixed', 'Radau 1 Fixed', 'RK3 EC', 'RK3 Fixed', 'RK2 Fixed', 'Semi-Explicit Euler Fixed', 'Implicit Euler EC', 'Implicit Euler Fixed', 'Bogacki Shampine3 EC', 'Bogacki Shampine3 Fixed');
legend(tttt{3}, tttf{3}, ttft{3}, ttff{3}, ...
       tftt{3}, tftf{3}, tfft{3}, tfff{3});
if(strcmp(normstyle, 'position_only'))
    xlabel('RMS of Position Errors (m), global calculated every 1e-2s');
elseif(strcmp(normstyle, 'velocity_only'))
    xlabel('RMS of Velocity Errors (m/s), global calculated every 1e-2s');
else
    xlabel('RMS of L2 Norm of State Errors, global calculated every 1e-2s');
end
ylabel('Number of Derivative Evaluations');
title(['Evaluations vs Error Plot, Point Contact']);
for i = 1:size(p,1)
    p(i).MarkerSize = 10;
    
end
end

%%
 normstyle = 'velocity_only';
% normstyle = 'both';
if( 0 )
figure();
p = loglog(fttt{1}, fttt{2}, '-*', ...
    fttf{1}, fttf{2}, '-x', ...
    ftft{1}, ftft{2}, '-*', ...
    ftff{1}, ftff{2}, '-x', ...
    fftt{1}, fftt{2}, '--+', ...
    fftf{1}, fftf{2}, '--o', ...
    ffft{1}, ffft{2}, '--+', ...
    ffff{1}, ffff{2}, '--o');
p(8).Color=[0.32,0.32,0.32];
%p(9).Color=[0.2, 0.4, 0.2];
%p(10).Color=[0.5, 0.4, 0.2];
hold on;
if(any(fttt{4}))
    display([fttt{3} num2str(fttt{4})])
end
if(any(fttf{4}))
    display([fttf{3} num2str(fttf{4})])
end
if(any(ftft{4}))
    display([ftft{3} num2str(ftft{4})])
end
if(any(ftff{4}))
    display([ftff{3} num2str(ftff{4})])
end
if(any(fftt{4}))
    display([fftt{3} num2str(fftt{4})])
end
if(any(fftf{4}))
    display([fftf{3} num2str(fftf{4})])
end
if(any(ffft{4}))
    display([ffft{3} num2str(ffft{4})])
end
if(any(ffff{4}))
    display([ffff{3} num2str(ffff{4})])
end
%legend('Radau 3 Fixed', 'Radau 1 Fixed', 'RK3 EC', 'RK3 Fixed', 'RK2 Fixed', 'Semi-Explicit Euler Fixed', 'Implicit Euler EC', 'Implicit Euler Fixed', 'Bogacki Shampine3 EC', 'Bogacki Shampine3 Fixed');
legend(fttt{3}, fttf{3}, ftft{3}, ftff{3}, ...
       fftt{3}, fftf{3}, ffft{3}, ffff{3});
if(strcmp(normstyle, 'position_only'))
    xlabel('RMS of Position Errors (m), global calculated every 1e-2s');
elseif(strcmp(normstyle, 'velocity_only'))
    xlabel('RMS of Velocity Errors (m/s), global calculated every 1e-2s');
else
    xlabel('RMS of L2 Norm of State Errors, global calculated every 1e-2s');
end
ylabel('Number of Derivative Evaluations');
title(['Evaluations vs Error Plot, Point Contact']);
for i = 1:size(p,1)
    p(i).MarkerSize = 10;
    
end
end

%%
% combined plot
width = 230.4;
height = 230.4;
fig1 = figure('Units', 'points', 'Position', [1000 1000 width height],'PaperUnits', 'points','PaperSize',[width+8 height+8]);
v_s = 1e-4;
p = loglog(tttf{1}./v_s, tttf{2}, '-ko', ...
    tttt{1}./v_s, tttt{2}, '--ko', ...
    fttf{1}./v_s, fttf{2}, '-ks', ...
    fttt{1}./v_s, fttt{2}, '--ks', ...
    ttff{1}./v_s, ttff{2}, '-kv', ...
    ttft{1}./v_s, ttft{2}, '--kv', ...
    ftff{1}./v_s, ftff{2}, '-k^', ...
    ftft{1}./v_s, ftft{2}, '--k^', 'MarkerSize', 4.5, 'LineWidth', 1,'MarkerFaceColor',[0.5,0.5,0.5]);
p(2).LineWidth = 1.;
p(4).LineWidth = 1.;
p(6).LineWidth = 1.;
p(8).LineWidth = 1.;
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 8)
xlim([1e-6, 3e-2]/v_s);
ylim([7e2, 4e7]);
xticks([1e-2, 1e-1, 1, 1e1, 1e2]);
yticks([1e3, 1e4, 1e5, 1e6, 1e7]);
grid on;
set(gca, 'xminorgrid', 'off');
set(gca, 'yminorgrid', 'off');


lgd = legend('FN', 'FN+TALS', 'EC+FN', 'EC+FN+TALS',...
       'QN', 'QN+TALS', 'EC+QN', 'EC+QN+TALS');
set(lgd, 'FontSize', 8)
lgd.Location = 'northeast';
if(strcmp(normstyle, 'position_only'))
    xlabel('RMS of Position Errors (m), global calculated every 1e-2s');
elseif(strcmp(normstyle, 'velocity_only'))
    xlabel('Slip Velocity Error ({\ite_v / v_s}) [-]');
else
    xlabel('RMS of L2 Norm of State Errors, global calculated every 1e-2s');
end
ylabel('Number of Function Evaluations [-]');
%title(['Work-Precision Plot of Implicit Euler Point Contact']);
saveas(fig1, '/home/antequ/integratorplots/WorkPrecisionImplicitEulerPoint.pdf')
% bottom right
fig2 = figure('Units', 'points', 'Position',  [1000 1000 width height],'PaperUnits', 'points','PaperSize',[width+8 height+8]);
p = loglog(tttf{1}(1:8)/v_s, tttf{2}(1:8), '-ko', ...
    tttt{1}(1:8)/v_s, tttt{2}(1:8), '--ko', ...
    fttf{1}(1:8)/v_s, fttf{2}(1:8), '-ks', ...
    fttt{1}(1:8)/v_s, fttt{2}(1:8), '--ks', ...
    ttff{1}(1:8)/v_s, ttff{2}(1:8), '-kv', ...
    ttft{1}(1:8)/v_s, ttft{2}(1:8), '--kv', ...
    ftff{1}(1:8)/v_s, ftff{2}(1:8), '-k^', ...
    ftft{1}(1:8)/v_s, ftft{2}(1:8), '--k^', 'MarkerSize', 6, 'LineWidth', 1,'MarkerFaceColor',[0.5,0.5,0.5]);
p(2).LineWidth = 1.;
p(4).LineWidth = 1.;
p(6).LineWidth = 1.;
p(8).LineWidth = 1.;
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 8)
grid on;
legend(tttf{3}, tttt{3},fttf{3}, fttt{3}, ...
        ttff{3}, ttft{3}, ftff{3}, ftft{3});
lgd = legend('FN', 'FN+TALS', 'EC+FN', 'EC+FN+TALS',...
       'QN', 'QN+TALS', 'EC+QN', 'EC+QN+TALS');
   lgd.Location = 'southwest';
xlim([1.7e-5, 1.2e-2]/v_s);
ylim([3e2, 1.8e5]);
if(strcmp(normstyle, 'position_only'))
    xlabel('RMS of Position Errors (m), global calculated every 1e-2s');
elseif(strcmp(normstyle, 'velocity_only'))
    xlabel('Slip Velocity Error ({\ite_v / v_s}) [-]');
else
    xlabel('RMS of L2 Norm of State Errors, global calculated every 1e-2s');
end
ylabel('Number of Function Evaluations [-]');
%title(['Work-Precision Plot of Implicit Euler Point Contact, Bottom Right']);
saveas(fig2, '/home/antequ/integratorplots/WorkPrecisionImplicitEulerPointZoomed.pdf')
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
stiction_start=9; stiction_end=16;
nstiction = stiction_end - stiction_start + 1;
vsnorm1 = vecnorm(diff1(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm2 = vecnorm(diff2(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm3 = vecnorm(diff3(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm4 = vecnorm(diff4(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm5 = vecnorm(diff5(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm6 = vecnorm(diff6(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm7 = vecnorm(diff7(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vsnorm8 = vecnorm(diff8(stiction_start:stiction_end,2),2) ./ sqrt(nstiction);
vs_threshold = 1e-4 ./ sqrt(nstiction); % 2.8e-4 fails; 1.7e-5 is fine
vsnorms = [vsnorm1 vsnorm2 vsnorm3 vsnorm4 vsnorm5 vsnorm6 vsnorm7 vsnorm8];
vs_skipped = vsnorms > vs_threshold;
returninfo = {l2_errs, nders, scheme_name, vs_skipped};
end
