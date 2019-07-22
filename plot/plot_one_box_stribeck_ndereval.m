

build = false;
collect_data = false;
run_plot = false;
make_plot = true;
showiuplot = false;
showbxplot = false;
showbvplot = false;
showftplot = false;
showtsplot = false;

shownstepplot = false;
show2dnstepplot = false;
saveplot = true;
display = false;
one_plot = false;

ground_truth = false;
compare_one_against_truth = true;
compare_integ_scheme_against_gt = true;

exec_name = 'passive_simulation';
orig_out_folder = 'outputs/resultcsv';
orig_out_folder = 'outputs/nd_resultcsv';
out_folder = orig_out_folder;
runout_folder = 'outputs/nd_runouts';
outmeta_folder = 'outputs/nd_metacsv';
figures_folder = 'outputs/nd_figures';
pdfs_folder = 'outputs/nd_figurepdfs';
args = '--target_realtime_rate="0" --max_time_step="1e-1"'; % '--box_mu_s 0. --autodiff="true"'
integration_schemes = {'fixed_implicit_euler', 'implicit_euler', 'semi_explicit_euler', 'runge_kutta2', 'runge_kutta3', 'radau'};

v_s_opts = [1e-2, 1e-3, 1e-4, 1e-5, 1e-6];

integration_schemes = {'bogacki_shampine3', 'implicit_euler', 'runge_kutta3', 'radau'};
integration_schemes = {'runge_kutta3'};
fixed_step = {'true','false'};
v_s_opts = [1e-4];
specific_v_s_ind = 4;
%v_s_opts = [1e-3, 1e-4];
accuracy_opts = [1e-2, 1e-3, 1e-4, 1e-5, 1e-6];
specific_acc_ind = 4;

if(ground_truth)
out_folder = 'outputs/truthcsv';
runout_folder = 'outputs/truthrunouts';
figures_folder = 'outputs/truthfigures';
pdfs_folder = 'outputs/truthfigurepdfs';
args = '--target_realtime_rate="0" --max_time_step="1e-5"';
integration_schemes = {'runge_kutta3', 'implicit_euler'};
accuracy_opts=[1e-8, 1e-10, 1e-12];
v_s_opts = [1e-4];
end

%% procedures
if(build)
%% build
setenv('DRAKE_PATH','/home/antequ/code/github/drake');
runstring = [' bazel build --config snopt //examples/box:' exec_name ];
system(['cd $DRAKE_PATH; bazel build --config snopt //examples/box:' exec_name ' ']);

end

if(collect_data)
%% run
system(['cd $DRAKE_PATH; mkdir -p ' out_folder]);
system(['cd $DRAKE_PATH; mkdir -p ' runout_folder]);
for integration_scheme_cell_index = 1:length(integration_schemes)
    integration_scheme=integration_schemes{integration_scheme_cell_index};
    for v_s = v_s_opts
        for target_accuracy = accuracy_opts
            filename = getfname(integration_scheme, v_s, target_accuracy);
            outcsv = [out_folder '/' filename];
            outfile = [runout_folder '/' filename];
            pass_args = [args  ' --box_v_s="' num2str(v_s) '" --accuracy="' num2str(target_accuracy) '" --integration_scheme="' integration_scheme '" --run_filename="' outcsv '"'];
            runstring = ['cd $DRAKE_PATH; ./bazel-bin/examples/box/' exec_name ' ' pass_args];
            disp(runstring)
            system([runstring ' 1>' outfile '.out 2>' outfile '.err &']);
        end
    end
end

end

if(run_plot)
%% read and plot
system(['cd $DRAKE_PATH; mkdir -p ' figures_folder]);
system(['cd $DRAKE_PATH; mkdir -p ' pdfs_folder]);
integration_scheme=integration_schemes{1};
v_s = v_s_opts(1);
target_accuracy = accuracy_opts(1);
X_vs_acc = repmat(v_s_opts',1,length(accuracy_opts));
Y_vs_acc = repmat(accuracy_opts,length(v_s_opts), 1);
Nsteps_values = cell(1,length(integration_schemes));

for integration_scheme_cell_index = 1:length(integration_schemes)
    integration_scheme=integration_schemes{integration_scheme_cell_index};
    Nsteps_vs_acc = zeros(length(v_s_opts), length(accuracy_opts));
    for v_s_index = 1:length(v_s_opts)
        v_s = v_s_opts(v_s_index);
        for acc_index = 1:length(accuracy_opts)
            target_accuracy = accuracy_opts(acc_index);
            filename = [getenv('DRAKE_PATH') '/' out_folder '/' getfname(integration_scheme, v_s, target_accuracy) '.csv'];
            result = csvread(filename, 1, 0);
            time = result(:,1);
            input_u = result(:,2);
            box_x = result(:,3);
            box_v = result(:,4);
            fric = result(:,5);
            ts_size = result(:,6);
            filetitle = [ strrep(integration_scheme, '_',' ') ', v_s ' num2str(v_s, '%.0e') ', acc ' num2str(target_accuracy, '%.0e')];
            nsteps = size(result,1);

            Nsteps_vs_acc(v_s_index, acc_index) = nsteps;
            if(make_plot)
                savestr = [getenv('DRAKE_PATH') '/' figures_folder '/' getfname(integration_scheme, v_s, target_accuracy) 'iu.fig'];
                savepdf = [getenv('DRAKE_PATH') '/' pdfs_folder '/' getfname(integration_scheme, v_s, target_accuracy) 'iu.pdf'];
                 makeplot(time, input_u, ['Input Force ' filetitle], 'Force (N)', showiuplot, saveplot, savestr, savepdf);

                savestr = [getenv('DRAKE_PATH') '/' figures_folder '/' getfname(integration_scheme, v_s, target_accuracy) 'bx.fig'];
                savepdf = [getenv('DRAKE_PATH') '/' pdfs_folder '/' getfname(integration_scheme, v_s, target_accuracy) 'bx.pdf'];
                makeplot(time, box_x, ['Box x ' filetitle], 'x (m)', showbxplot, saveplot, savestr, savepdf);

                savestr = [getenv('DRAKE_PATH') '/' figures_folder '/' getfname(integration_scheme, v_s, target_accuracy) 'bv.fig'];
                savepdf = [getenv('DRAKE_PATH') '/' pdfs_folder '/' getfname(integration_scheme, v_s, target_accuracy) 'bv.pdf'];
                makeplot(time, box_v, ['Box v ' filetitle], 'v (m/s)', showbvplot, saveplot, savestr, savepdf);

                savestr = [getenv('DRAKE_PATH') '/' figures_folder '/' getfname(integration_scheme, v_s, target_accuracy) 'ft.fig'];
                savepdf = [getenv('DRAKE_PATH') '/' pdfs_folder '/' getfname(integration_scheme, v_s, target_accuracy) 'ft.pdf'];
                makeplot(time, fric, ['Friction ' filetitle], 'Force (N)', showftplot, saveplot, savestr, savepdf);


                savestr = [getenv('DRAKE_PATH') '/' figures_folder '/' getfname(integration_scheme, v_s, target_accuracy) 'ts.fig'];
                savepdf = [getenv('DRAKE_PATH') '/' pdfs_folder '/' getfname(integration_scheme, v_s, target_accuracy) 'ts.pdf'];
                makeplot(time, ts_size, ['Step Size ' filetitle], 'Timestep Size (s)', showtsplot, saveplot, savestr, savepdf);
            end
        end
    end
    Nsteps_values{integration_scheme_cell_index} = Nsteps_vs_acc;

end


%% plot

for integration_scheme_cell_index = 1:length(integration_schemes)
    integration_scheme=integration_schemes{integration_scheme_cell_index};
    Nsteps_vs_acc = Nsteps_values{integration_scheme_cell_index};
    
    if(shownstepplot)
        f = figure();
        surf(X_vs_acc, Y_vs_acc, Nsteps_vs_acc);
        ax = f.CurrentAxes;
        set(ax, 'XScale', 'log');
        set(ax, 'YScale', 'log');
        set(ax, 'ZScale', 'log');
        title(['Number of steps, ' strrep(integration_scheme, '_',' ')])
        zlabel('N steps')
        xlabel('v_s')
        ylabel('acc')
    end
    
    if(show2dnstepplot)
        nsteps = Nsteps_vs_acc(specific_v_s_ind,:);
        f = figure();
        plot(accuracy_opts, nsteps);
        ax = f.CurrentAxes;
        set(ax, 'XScale', 'log');
        set(ax, 'YScale', 'log');
        title(['Number of steps, ' strrep(integration_scheme, '_',' ') ', v_s = ' num2str(v_s_opts(specific_v_s_ind))])
        ylabel('N steps')
        xlabel('acc')
        nsteps = Nsteps_vs_acc(:,specific_acc_ind)';
        f = figure();
        plot(v_s_opts, nsteps);
        ax = f.CurrentAxes;
        set(ax, 'XScale', 'log');
        set(ax, 'YScale', 'log');
        title(['Number of steps, ' strrep(integration_scheme, '_',' ') ', acc = ' num2str(accuracy_opts(specific_acc_ind))])
        ylabel('N steps')
        xlabel('v_s')
    end
end
%%
if(one_plot)
nsteps_implicit_euler = Nsteps_values{2}(specific_v_s_ind, :);
nsteps_rk3 = Nsteps_values{5}(specific_v_s_ind, :);
        f = figure();
        plot(accuracy_opts, nsteps_implicit_euler);
        ax = f.CurrentAxes;
        set(ax, 'XScale', 'log');
        hold on
        plot(accuracy_opts, nsteps_rk3);
        ax = f.CurrentAxes;
        set(ax, 'XScale', 'log');
        title(['Number of steps, v_s = ' num2str(v_s_opts(specific_v_s_ind))])
        ylabel('N steps')
        xlabel('acc')
end

end
%% ground truth computation for v_s = 1e-4
% use RK3 with 1e-5 timesteps, 1e-12 error desired
if(ground_truth)
    infile = '/home/antequ/code/github/drake/outputs/truthcsv/runge_kutta3_vs40_acc120.csv'
    gtresult = csvread(infile, 1, 0);
    gttime    = gtresult(:,1);
    gtinput_u = gtresult(:,2);
    gtbox_x   = gtresult(:,3);
    gtbox_v   = gtresult(:,4);
    gtfric    = gtresult(:,5);
    gtts_size = gtresult(:,6);
    gtnsteps  = size(gtresult,1);
    
end

%% Compare one integrator against ground truth
if(compare_one_against_truth)
    infile = '/home/antequ/code/github/drake/outputs/lsresultcsv/implicit_euler_vs40_acc50.csv';
    result = csvread(infile, 1, 0);
    time    = result(:,1);
    input_u = result(:,2);
    box_x   = result(:,3);
    box_v   = result(:,4);
    fric    = result(:,5);
    ts_size = result(:,6);
    [uniontimes, our_interp, gt_interp, integ_errors] = compute_errors(gtresult, result, 2);
    our_iu = our_interp(:,2);
    our_bx = our_interp(:,3);
    our_bv = our_interp(:,4);
    our_ft = our_interp(:,5);
    our_ts = our_interp(:,6);
    gt_iu  = gt_interp(:,2);
    gt_bx  = gt_interp(:,3);
    gt_bv  = gt_interp(:,4);
    gt_ft  = gt_interp(:,5);
    gt_ts  = gt_interp(:,6);
    e_iu = (our_iu - gt_iu); %./ abs(gt_iu);
    e_bx = (our_bx - gt_bx); %./ abs(gt_bx);
    e_bv = (our_bv - gt_bv); %./ abs(gt_bv);
    e_ft = (our_ft - gt_ft); %./ abs(gt_ft);
    desc = 'implicit euler, v_s=1e-4, acc=1e-5'
    f = figure();
    plot(uniontimes, e_ft);
    title(['fric force error, ' desc]);
    ylabel('error (N)')
    xlabel('time (s)')
    hold on
    %plot(uniontimes, e_bx);
    %plot(uniontimes, e_bv);
    
    plot(uniontimes, gt_ft);
    plot(uniontimes, our_ft);
    plot(uniontimes, -gt_iu);
    
    f = figure();
    plot(uniontimes, e_bx);
    title(['box position error, ' desc]);
    ylabel('error (m)')
    xlabel('time (s)')
    hold on
    %plot(uniontimes, e_bx);
    %plot(uniontimes, e_bv);
    
    plot(uniontimes, gt_bx);
    plot(uniontimes, our_bx);
    
end
%% Aggregately compare a scheme against GT

agg_integ_errors = zeros(length(accuracy_opts), size(gtresult,2));
if(compare_integ_scheme_against_gt)
    out_folder = orig_out_folder;
    integration_scheme = 'runge_kutta3';
    v_s = 1e-4;
    for target_accuracy_ind = 1:length(accuracy_opts)
        target_accuracy = accuracy_opts(target_accuracy_ind);
        filename = [getenv('DRAKE_PATH') '/' out_folder '/' getfname(integration_scheme, v_s, target_accuracy) '.csv'];
        result = csvread(filename, 1, 0);
        [uniontimes, our_interp, gt_interp, integ_errors] = compute_errors(gtresult, result, 2);
        agg_integ_errors(target_accuracy_ind, :) = integ_errors;
    end
    
%%
    agg_iu = agg_integ_errors(:, 2); 
    agg_bx = agg_integ_errors(:, 3);
    agg_bv = agg_integ_errors(:, 4);
    agg_ft = agg_integ_errors(:, 5);
    f = figure();
    plot(accuracy_opts, agg_ft, '-o');
    title(['average friction errors, ' strrep(integration_scheme,'_',' ') ', v_s=' num2str(v_s)]);
    ylabel('friction error (N)')
    xlabel('acc tolerance')
    hold on
    set(gca, 'xscale', 'log')
    set(gca, 'yscale', 'log')
    f2 = figure();
    plot(accuracy_opts, agg_bv, '-o');
    title(['average velocity errors, ' strrep(integration_scheme,'_',' ') ', v_s=' num2str(v_s)]);
    ylabel('velocity error (m/s)')
    xlabel('acc tolerance')
    hold on
    set(gca, 'xscale', 'log')
    set(gca, 'yscale', 'log')
    
    f3 = figure();
    plot(accuracy_opts, agg_bx, '-o');
    title(['average displacement errors, ' strrep(integration_scheme,'_',' ') ', v_s=' num2str(v_s)]);
    ylabel('position error (m)')
    xlabel('acc tolerance')
    hold on
    set(gca, 'xscale', 'log')
    set(gca, 'yscale', 'log')
    
    f4 = figure();
    plot(accuracy_opts, agg_iu, '-o');
    title(['average input force errors, ' strrep(integration_scheme,'_',' ') ', v_s=' num2str(v_s)]);
    ylabel('input force error (N)')
    xlabel('acc tolerance')
    hold on
    set(gca, 'xscale', 'log')
    set(gca, 'yscale', 'log')
end

function [uniontimes, our_interp, gt_interp, integ_errors] = compute_errors(gtresult, result, p)
   time    = result(:,1);
   gttime  = gtresult(:,1);
   uniontimes = union(time, gttime);
   our_interp = interp1(time, result, uniontimes);
   gt_interp = interp1(gttime, gtresult, uniontimes);
   diff = gt_interp - our_interp;
   integ_errors = (trapz(uniontimes,abs(diff).^p) / (uniontimes(end) - uniontimes(1))).^(1./p);

end


function [f] = makeplot(time, y,  title_s, ylabel_s, show, save, savestr, savepdf)
if(show)
    f = figure(); %'visible','off'
else
    f = figure('visible', 'off');
end
plot(time, y);
title(title_s);
ylabel(ylabel_s);
xlabel('time (s)');

if(save)
   saveas(f, savestr);
   saveas(f, savepdf);
end

if(~show)
    close(f)
end
end


function [output] = getfname(integration_scheme, v_s, target_accuracy, fixed_step) 
integ = [];
if(fixed_step == true)
    integ = [integration_scheme '_fixed'];
else
    integ = [integration_scheme '_var'];
end
output = [integ '_vs' num2fname(v_s) '_acc' num2fname(target_accuracy)];
end

function [output] = num2fname(num) 
baseten = - log10(num);
output = num2str(floor(baseten * 10));
end