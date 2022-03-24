clear all;
initialize_environment;
% ----------------------------
%% Controls
% ----------------------------
% Set this flag to 0 to disable route planning and load a precomputed route
enable_routePlan = 0;
enable_simulation = 1;
plot_freq = 1;

 % 1 -> pure pursuit; 2 -> S. Kinematic; 3 -> S. Dynamic 4 -> clothoid-based
latContr_select = 2;

% Set this flag to 1 in order to enable online plots during the simulation
enable_onlinePlots = 0;
% Set this flag to 1 in order to enable a zoomed view in online simulation plots
enable_zoom = 0;

change_samp = true;
output_result = false;
% ----------------------------
%% Load files
% ----------------------------

load('../exercise6/results/kus');
load('../exercise6/results/clothoid_look_up');
load('../exercise6/results/p_pursuit_look_up');
output_file = 'graphs/q%d/ex-7%d%s.eps';
output_files = 'graphs/q%d/ex-7%d%s-%d.eps';
q = 1;   

% ----------------------------
%% variables to loop through
% ----------------------------

% for route planning
% c_distances = [20, 30, 35, 40, 45, 50, 55, 60 , 70];
% min_itrs = [1e5 1e4 1e3 1e2];
% max_itrs = [1e6 1e5 1e4 1e3];
c_distances = [45];
min_itrs = [1e5];
max_itrs = [1e6];

m_steers = [5];
interpolation_samples = [50];

% for simulations
const_speed = true;
speeds_req = [
%     20,
     30,
%     40,
%     50,
    60,
%      70
    ];
speeds_req_k = speeds_req;
speeds_req = speeds_req/3.6;

% to set initial speed
if const_speed
    init_speeds = speeds_req;
    init_speeds_k = speeds_req_k;
else
    init_speeds = [20,50]/3.6;
end

% route planner optimization
% c_distances = [20,30,40,50];
% min_itrs = [1e3, 1e4, 1e5];
% max_itrs = [1e4, 1e5, 1e6];
% m_steers = [4, 5, 6];
% interpolation_samples = [20, 30];

% ----------------------------
%% Load vehicle data
% ----------------------------
vehicle_data = getVehicleDataStruct();
tau_D = vehicle_data.steering_system.tau_D;
pacejkaParam = loadPacejkaParam();

% ----------------------------
%% Define initial conditions for the simulation
% ----------------------------
X0 = loadInitialConditions;

% ----------------------------
%% Longitudinal controller parameters
% ----------------------------
longCTRparam = longitController();

% ----------------------------
%% Simulation parameters
% ----------------------------
simulationPars = getSimulationParams(); 
Ts = simulationPars.times.step_size;  % integration step for the simulation (fixed step)
T0 = simulationPars.times.t0;         % starting time of the simulation
Tf = simulationPars.times.tf;         % stop time of the simulation

% ----------------------------
%% Load lateral controller parameters
% ----------------------------
LLC = load_LowLevelControlData();
LLC_sampleTime = LLC.sample_time;


init_speed = init_speeds(1);
init_speed_k = init_speeds_k(1);
X0(4) = init_speed;
speed_req = speeds_req(1);
kus = kus_table(round(kus_table(:,1))==init_speed_k,2);
clothoid_look_ahead = clothoid_look_up(clothoid_look_up(:,1)==init_speeds_k(1),2);

clothoidBasedParams = clothoidBasedControllerParams(kus,clothoid_look_ahead);
stanleyParams       = stanleyControllerParams(4,0.1);
p_pursuit_look_ahead = pure_pursuit_look_up(pure_pursuit_look_up(:,1)==init_speeds_k(1),2);
purePursuitParams   = purePursuitControllerParams(p_pursuit_look_ahead);

% ----------------------------
%% Load road scenario
% ----------------------------
scenario = load('./Scenario/scenario');
costmap = scenario.scenario_data.costmap;
mapObjects = scenario.scenario_data.mapObjects;
vehicleDims = scenario.scenario_data.vehicleDims;

% ----------------------------
%% graphical interface 
% ----------------------------

if (enable_onlinePlots)
    % Initialize figure for online plots
    figure('Name','Road Scenario','NumberTitle','off')
    grid on
    axis equal
    xlabel('x [m]')
    ylabel('y [m]')
    title('Road Scenario')
    hold on
end
% ----------------------------
%% Start Simulation
% ----------------------------
fprintf('Starting Simulation\n')
tic;
if (enable_routePlan)
    % Perform route planning
    route_results = {};
    % loop through all test speeds
    total = 0;
    for ind1=1:length(c_distances)
        c_distance = c_distances(ind1);
        for ind2=1:length(min_itrs)
            min_itr = min_itrs(ind2);
            max_itr = max_itrs(ind2);
            if (c_distance == 40 || c_distance == 50) && min_itr <= 1e4
            else
                for ind3=1:length(m_steers)
                    m_steer = m_steers(ind3);
                    for ind4=1:length(interpolation_samples)
                        total = total +1;
                        interpolation_sample = interpolation_samples(ind4);
                        comb = append(string(c_distance),',1e',...
                            string(log10(min_itr)),',1e', string(log10(max_itr)),',',...
                            string(m_steer),',', string(interpolation_sample));
                        fprintf('starting test #%d [%s]\n', total ,comb);
                        routePlanner;
                        fprintf('finished\n');
                        route_results{total} = route_data;
                    end
                end
            end
        end
    end
    
    route_results_table = zeros(length(route_results),8);
    for i = 1:length(route_results)
        route_results_table(i,:) = ...
        [route_results{i}.c_distance
        route_results{i}.min_itr
        route_results{i}.elapsed_time_routePlan
        route_results{i}.max_itr
        route_results{i}.m_steer
        route_results{i}.interpolation_sample
        route_results{i}.refPath.Length
        length(route_results{i}.refPath_poses_fewPoints)]';
    end
    % to output results to csv
    T = array2table(route_results_table);
    T.Properties.VariableNames(1:8) = {'c_dist','min_itr','elapsed_time','max_itr','m_steer','inter_sample','path_length','sampled_path_length'};
    output = 'results/route_results_table_%d.csv';
    writetable(T,sprintf(output,latContr_select));
    
    elapsed_time_table = zeros(length(c_distances),length(min_itr)+1);
    route_len_table = zeros(length(c_distances),length(min_itr)+1);
    for k =1:length(c_distances)
        elapsed_time_table(k+1,1) = c_distances(k);
        route_len_table(k+1,1) = c_distances(k);
        for j =1:length(min_itrs)
            elapsed_time_table(1,j+1) = min_itrs(j);
            route_len_table(1,j+1) = min_itrs(j);
            for i = 1:length(route_results)
                if route_results{i}.c_distance == c_distances(k)
                    if route_results{i}.min_itr == min_itrs(j)
                        elapsed_time_table(k+1,j+1) = route_results{i}.elapsed_time_routePlan;
                        route_len_table(k+1,j+1) = route_results{i}.refPath.Length;
                    end
                end
            end
        end
    end
    
else
    % Load a precomputed route to decrease simulation time
    referencePath_points_original = load('refRoute_poses_RRT'); % RRT* solution
    referencePath_fewPoints = load('refPath_poses_fewPoints');  % interpolated RRT* solution with few points
    cpuTime_routePlan = load('cpuTime_routePlan');  
    refPath = load('refPath');  
    refPath = refPath.refPath;
    refRoute_points_orig = referencePath_points_original.refRoute_points_orig;
    refRoute_fewPoints   = referencePath_fewPoints.refRoute_fewPoints;
    elapsed_time_routePlan = cpuTime_routePlan.elapsed_time_routePlan; 
    route_data = struct();
    route_data = struct();
    route_data.c_distance = c_distances(1);
    route_data.min_itr = min_itrs(1);
    route_data.max_itr = max_itrs(1);
    route_data.m_steer = m_steers(1);
    route_data.interpolation_sample = interpolation_samples(1);
    route_data.costmap = costmap;
    route_data.refPath = refPath;
    route_data.refRoute_points_orig = refRoute_points_orig;
    route_data.refRoute_fewPoints = refRoute_fewPoints;
%     route_data.refPath_poses_fewPoints = refPath_poses_fewPoints;
    route_data.elapsed_time_routePlan = elapsed_time_routePlan;
    route_results = route_data;    
end



if(enable_simulation)
% Simulink simulation
if(change_samp)
    interp_sampling = interpolation_samples;
    interp_vector_fewPoints = 0:interp_sampling:refPath.Length;
    refPath_poses_fewPoints = interpolate(refPath,interp_vector_fewPoints);
    isPathValid = checkPathValidity(refPath,costmap);
    path_fewPoints = ClothoidList();
    numOfClothoids_fewPoints = size(refPath_poses_fewPoints,1);
    for jj = 1:numOfClothoids_fewPoints-1
        try
            path_fewPoints.push_back_G1(refPath_poses_fewPoints(jj,1),refPath_poses_fewPoints(jj,2),deg2rad(refPath_poses_fewPoints(jj,3)), refPath_poses_fewPoints(jj+1,1),refPath_poses_fewPoints(jj+1,2),deg2rad(refPath_poses_fewPoints(jj+1,3))); 
        catch
        warning('Problem using function.  Assigning a value of 0.');
        end
    end
    % Compute the local curvature for the reference route
    interp_vector_fewPoints = [interp_vector_fewPoints, path_fewPoints.length];    % add also the final route point
    [x_cloth_refPath_fewPoints,y_cloth_refPath_fewPoints,theta_cloth_refPath_fewPoints,curv_refPath_fewPoints] = path_fewPoints.evaluate(interp_vector_fewPoints);
    refRoute_fewPoints = [x_cloth_refPath_fewPoints',y_cloth_refPath_fewPoints',theta_cloth_refPath_fewPoints',curv_refPath_fewPoints'];
end

model_sims = {};
error_data = {};
for ind=1:length(speeds_req)
    init_speed = init_speeds(ind);
    init_speed_k = init_speeds_k(ind);
    X0(4) = init_speed;
    speed_req = speeds_req(ind);
    speed_req_k = speeds_req_k(ind);
    kus = kus_table(round(kus_table(:,1))==init_speeds_k(ind),2);
    
    clothoid_look_ahead = clothoid_look_up(clothoid_look_up(:,1)==init_speeds_k(ind),2);
    clothoidBasedParams = clothoidBasedControllerParams(kus,clothoid_look_ahead);
    p_pursuit_look_ahead = pure_pursuit_look_up(pure_pursuit_look_up(:,1)==init_speeds_k(ind),2);

    purePursuitParams   = purePursuitControllerParams(p_pursuit_look_ahead);

    fprintf('Starting Simulation # %d [%d km/h]\n', ind ,speeds_req_k(ind));
    model_sim = sim('framework_sim.slx');
    elapsed_time_simulation = toc; 
    fprintf('Simulation completed\n')
    total_simul_time = elapsed_time_simulation + elapsed_time_routePlan;
    fprintf('It took %.1f seconds to compute the route with RRT*\n',elapsed_time_routePlan)
    fprintf('The total simulation time is %.1f seconds\n',total_simul_time)
    model_sims{ind} = model_sim;
    error_central = struct();
    dataAnalysis;
    error_data{ind} = error_central;
end
% --------------------------------------
%% speed / look-up combination vs. error
% --------------------------------------

    format shortG
    error_speed_table = zeros(length(error_data),4);
    for i = 1:length(error_data)
        error_speed_table(i,:) = [
            error_data{i}.speed
            error_data{i}.e_max
            error_data{i}.e_mean
            error_data{i}.e_std];
    end
    
    % to output results to csv
    T = array2table(error_speed_table);
    T.Properties.VariableNames(1:4) = {'u','max_error','mean_error','std_error'};
    output = 'results/error_results_table_%d.csv';
    writetable(T,sprintf(output,latContr_select));
    
% ----------
%% Bar graph
% ----------

pivot_table = error_speed_table(:,[1 2]);
% flipud(winter(7)
if latContr_select ==1 || latContr_select == 4
    leg_name = 'look ahead [m]';
else
    leg_name = 'gain value';
end
figure('Name','Tracking error','NumberTitle','off'); clf;

% set(0,'DefaultAxesColorOrder',parula(length(la_to_plot)))

bar(pivot_table(:,1),pivot_table(:,2:end));
grid on
xlabel('vehicle speed (u) [$km/h$]')
ylabel('tracking error [m]')
% yticks((0:2:14))
%     ylim([0 52])
% xlim([10 110])
set(gca,'fontsize',26)
% hleg = legend(string(la_to_plot),'location','NW');
% htitle = get(hleg,'Title');
% set(htitle,'String',leg_name)

pbaspect([1 1 1])
exportgraphics(gcf,sprintf(output_files,q,q,'ss',latContr_select),'ContentType','vector')
end

%% for saving specified results

if output_result
    sw = 3;
    if sw ==1
        what_for1 = 'm_steer_test_';
        what_for2 = m_steers(1);
    end
    if sw == 2
        what_for1 = 'cdist_test';
        what_for2 = c_distances(1);
    end
    
    if sw == 3
        what_for1 = 'samp_test';
        what_for2 = interpolation_samples(1);
    end
    
    temp_var = strcat( what_for1 ,num2str(what_for2));
    if ~enable_routePlan
        route_data = struct();
        route_data.c_distance = c_distances(1);
        route_data.min_itr = min_itrs(1);
        route_data.max_itr = max_itrs(1);
        route_data.m_steer = m_steers(1);
        route_data.interpolation_sample = interpolation_samples(1);
        route_data.costmap = costmap;
        route_data.refPath = refPath;
        route_data.refRoute_points_orig = refRoute_points_orig;
        route_data.refRoute_fewPoints = refRoute_fewPoints;
        route_data.refPath_poses_fewPoints = refPath_poses_fewPoints;
        route_data.elapsed_time_routePlan = elapsed_time_routePlan;
        route_results = route_data;
    end
    
    S.(temp_var) = {route_results, model_sims, error_data};

    output_name = 'results/%s_%d.mat';
    save(sprintf(output_name,what_for1,what_for2),'-struct', 'S');
end
