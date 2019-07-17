

build = false;
collect_data = false;
plot = true;
showiuplot = false;
showbxplot = false;
showbvplot = false;
showftplot = false;
showtsplot = false;
saveplot = true;
display = true;
exec_name = 'passive_simulation';
out_folder = 'outputs/resultcsv';
runout_folder = 'outputs/runouts';
figures_folder = 'outputs/figures';
pdfs_folder = 'outputs/figurepdfs';
args = '--target_realtime_rate="0"'; % '--autodiff="true"'
integration_schemes = {'fixed_implicit_euler', 'implicit_euler', 'semi_explicit_euler', 'runge_kutta2', 'runge_kutta3', 'radau'};
v_s_opts = [1e-2, 1e-3, 1e-4, 1e-5, 1e-6];
accuracy_opts = [1e-2, 1e-3, 1e-4, 1e-5, 1e-6];

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

if(plot)
%% plot
system(['cd $DRAKE_PATH; mkdir -p ' figures_folder]);
system(['cd $DRAKE_PATH; mkdir -p ' pdfs_folder]);
integration_scheme=integration_schemes{4};
v_s = v_s_opts(1);
target_accuracy = accuracy_opts(2);
for integration_scheme_cell_index = 1:length(integration_schemes)
    integration_scheme=integration_schemes{integration_scheme_cell_index};
    for v_s = v_s_opts
        for target_accuracy = accuracy_opts
filename = [getenv('DRAKE_PATH') '/' out_folder '/' getfname(integration_scheme, v_s, target_accuracy) '.csv'];
result = csvread(filename, 1, 0);
time = result(:,1);
input_u = result(:,2);
box_x = result(:,3);
box_v = result(:,4);
fric = result(:,5);
ts_size = result(:,6);
filetitle = [ strrep(integration_scheme, '_',' ') ', v_s ' num2str(v_s, '%.0e') ', acc ' num2str(target_accuracy, '%.0e')];

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


function [output] = getfname(integration_scheme, v_s, target_accuracy) 
output = [integration_scheme '_vs' num2fname(v_s) '_acc' num2fname(target_accuracy)];
end

function [output] = num2fname(num) 
baseten = - log10(num);
output = num2str(floor(baseten * 10));
end