% ----------------------------------------------------------------
%% Main script for self-driving vehicle control framework 
%  authors: Mattia Piccinini & Gastone Pietro Papini Rosati
%  emails:  mattia.piccinini@unitn.it, gastone.rosatipapini@unitn.it
%  date:    26/11/2020
% ----------------------------------------------------------------

% ----------------------------
%% Initialization
% ----------------------------
initialize_environment;
load('kus');
load('speed_look_result');
output_file = './graphs/q%d/ex-6%d%s-%d.eps';
q = 1;   

% ----------------------------
%% variables to loop through
% ----------------------------

const_speed = true;
freq = 0.001; % 1/s
req_speeds = [20,30,40,50,60,70,80,90,100];
req_speeds_k = req_speeds;
req_speeds = req_speeds/3.6;
look_ahead_list = [1,2,3,4,5,10,15,20,25,30]; %[5,10,15,20,25,30];
if const_speed
    init_speeds = req_speeds;
    init_speeds_k = req_speeds_k;
else
    init_speeds = [20,50]/3.6;
end

% speed_req = 70/3.6;
% init_speed = speed_req;
% kus = kus_table(round(kus_table(:,1))==init_speed_k,2);
% look_ahead = 30;
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
clothoidBasedParams = clothoidBasedControllerParams(0,0);
stanleyParams       = stanleyControllerParams();


% ----------------------------------------------------------------
%% Select the desired lateral controller
% ----------------------------------------------------------------

% -------------------
% Selection logic
%   o latContr_select = 1 --> arc path following
%   o latContr_select = 2 --> Stanley kinematic
%   o latContr_select = 3 --> Stanley dynamic
%   o latContr_select = 4 --> clothoid-based
% -------------------
latContr_select = 1;

% ----------------------------
%% Load road scenario
% ----------------------------
road_path = load('./Scenario/scenario_test');
road_data = [road_path.path.x, road_path.path.y, road_path.path.theta];
road_data_sampled = [road_path.path.x_sampled', road_path.path.y_sampled'];

% ----------------------------
%% Define graphical interface settings
% ----------------------------
% Set this flag to 1 in order to enable online plots during the simulation
enable_onlinePlots = 0;
% Set this flag to 1 in order to enable a zoomed view in online simulation plots
enable_zoom = 1;

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
model_sims = {};
error_data = {};
total = 0;
for ind=1:length(req_speeds)
    speed_req = req_speeds(ind);
    speed_req_k = req_speeds_k(ind);
    init_speed = init_speeds(ind);
    init_speed_k = init_speeds_k(ind);
    if latContr_select == 4
        kus = kus_table(round(kus_table(:,1))==init_speed_k,2);
    end
    X0(4) = init_speed;
    
    for ind2 = 1: length(look_ahead_list)

        look_ahead = look_ahead_list(ind2);
        if (speed_req_k ==70 &&  look_ahead < 15) || (speed_req_k ==80 &&  look_ahead < 15) || (speed_req_k >=90 &&  look_ahead < 20)
            int = 1
        else
            total = total +1;
            if latContr_select == 4
                clothoidBasedParams = clothoidBasedControllerParams(kus,look_ahead);
            end
            purePursuitParams   = purePursuitControllerParams(look_ahead);
            fprintf('Starting Simulation # %d [%d km/h] w/ look_ahead %d\n', total, round(speed_req*3.6),look_ahead);
            tic;
            % Simulink simulation
            model_sim = sim('framework_sim.slx');
            model_sims{total} = model_sim;
            total_simul_time = toc; 
            fprintf('Simulation completed\n')
            fprintf('The total simulation time was %.1f seconds\n',total_simul_time)
            error_data{total} = dataAnalysis(model_sim,vehicle_data,Ts,road_data_sampled,speed_req_k,look_ahead);
        end
    end
end
% ----------------------------
%% Post-Processing
% ----------------------------

%% speed / look-up combination vs. error

    format shortG
    error_speed_lookahead_table = zeros(length(error_data),7);
    for i = 1:length(error_data)
%         fprintf("speed: %d lookup: %d", error_data{i}.speed, error_data{i}.look_ahead);
        error_speed_lookahead_table(i,:) = [
            error_data{i}.speed
            error_data{i}.look_ahead
            error_data{i}.e_max
            error_data{i}.reached_flag
            error_data{i}.steps_taken
            error_data{i}.e_mean
            error_data{i}.e_std];
    end
    
    T = array2table(error_speed_lookahead_table);
    T.Properties.VariableNames(1:7) = {'u','look_ahead','max_error','reached_flg','steps_to_finish','mean_error','std_error'};
    writetable(T,'error_speed_lookahead_table_pure.csv');
    
    %% bar graph for tracking error [clothoid]
    la_to_plot = [3,10,15,20,25,30];
%     la_to_plot = look_ahead_list;
    s_to_plot = [20,40,60,80,100];
%     s_to_plot = req_speeds_k;
    pivot_table = zeros(length(s_to_plot),length(la_to_plot)+1);
    for k =1:length(s_to_plot)
        pivot_table(k,1) = s_to_plot(k);
        for j =1:length(la_to_plot)
            for i = 1:length(error_data)
                if error_data{i}.reached_flag
                    if error_data{i}.speed == s_to_plot(k)
                        if error_data{i}.look_ahead == la_to_plot(j)
                            pivot_table(k,j+1) = error_data{i}.e_max;
                        end
                    end
                end
            end
        end
    end
    %% bar graph
    % flipud(winter(7)
    set(0,'DefaultAxesColorOrder',parula(7))
    bar(pivot_table(:,1),pivot_table(:,2:end));
    grid on
    xlabel('vehicle speed (u) [$km/h$]')
    ylabel('tracking error [m]')
    yticks((0:2:14))
%     ylim([0 52])
    xlim([10 110])
    set(gca,'fontsize',26)
    hleg = legend(string(la_to_plot),'location','NW');
    htitle = get(hleg,'Title');
    set(htitle,'String','look ahead [m]')
    pbaspect([1 1 1])
    exportgraphics(gcf,sprintf(output_file,q,q,'a',latContr_select),'ContentType','vector')
