%% booleans for turning on and off procedures
run_build = false;


%%
fixed_integration_schemes = {'implicit_euler',  'radau', 'semi_explicit_euler', 'runge_kutta2', 'runge_kutta3', 'bogacki_shampine3'};
ec_integration_schemes = {'implicit_euler', 'runge_kutta3', 'bogacki_shampine3'};
it_lim_integration_schemes = {'implicit_euler', 'radau'};
it_lim_ec_integration_schemes = {'implicit_euler'}; % (merge in Evan's Radau changes)


%% build
if(run_build)

setenv('DRAKE_PATH','/home/antequ/code/github/drake');
runstring = [' bazel build //examples/box2d:' exec_name ];
system(['cd $DRAKE_PATH; bazel build --config snopt //examples/box2d:' exec_name ' ']);

end

%% run integration schemes
% friction

% run fixed schemes

% run ec schemes

% run iteration limited schemes

% run iteration limited ec schemes

% frictionless

% run fixed schemes

% run ec schemes

% run iteration limited schemes

% run iteration limited ec schemes

