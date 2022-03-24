% ----------------------------
%% Initialization
% ----------------------------
initialize_environment;
load('results/kus');
load('results/speed_look_result');
output_file = 'graphs/q%d/ex-6%d%s-%d.eps';
q = 1;   

% ----------------------------
%% variables to loop through
% ----------------------------

const_speed = true;
freq = 0.001; % 1/s
% speeds to test
req_speeds = [
%     20,
    30,
%     40,
%     50,
%     60,
    70,
%     80,
%     90,
%     100
    ];
req_speeds_k = req_speeds;
req_speeds = req_speeds/3.6;
% look ahead to test
% look_ahead_list = [3,10,15,20,25,30];
look_ahead_list = [3,20];
% combination testing for kinematic [steer angle, gain]
s_k_tests = {
    {[6],[0.1,0.5,1]}, % 20
    {[6],[0.1,0.5,1]}, % 30
    {[6],[0.1,0.5,1]}, % 40
    {[6],[0.1,0.5,1]}, % 50
    {[6],[0.1,0.5,1]}, % 60
    {[5],[0.1,0.5]}, % 70
    {[4],[0.1]}, % 80
    {[3],[0.1,0.5]}, % 90
    {[2],[0.1,0.5,1]}, % 100
};

% to set initial speed
if const_speed
    init_speeds = req_speeds;
    init_speeds_k = req_speeds_k;
else
    init_speeds = [20,50]/3.6;
end

% for quick trials
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
stanleyParams       = stanleyControllerParams(0.5,0.5);
purePursuitParams   = purePursuitControllerParams(0);

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
% to save simulation models and their error data
model_sims = {};
error_data = {};
total = 0;
% loop through all test speeds
for ind=1:length(req_speeds)
    speed_req = req_speeds(ind);
    speed_req_k = req_speeds_k(ind);
    init_speed = init_speeds(ind);
    init_speed_k = init_speeds_k(ind);
    X0(4) = init_speed;
    % loop for stanly controllers
    if latContr_select ==2 || latContr_select ==3
        for ind2 = 1: length(s_k_tests{ind}{2}) %at specific gain
            total = total +1;
            stanleyParams = stanleyControllerParams(s_k_tests{ind}{1},s_k_tests{ind}{2}(ind2));          
            fprintf('Starting Simulation # %d [%d km/h] w/ max steer %d , gain %f \n', total ,speed_req_k, s_k_tests{ind}{1},s_k_tests{ind}{2}(ind2));
            tic;
            % Simulink simulation
            model_sim = sim('framework_sim.slx');
            model_sims{total} = model_sim;
            total_simul_time = toc; 
            fprintf('Simulation completed\n')
            fprintf('The total simulation time was %.1f seconds\n',total_simul_time)
            error_data{total} = dataAnalysis(model_sim,vehicle_data,Ts,road_data_sampled,speed_req_k,s_k_tests{ind}{2}(ind2));
        end
    end    
    % loop for clothoid and pure pursuit
    if latContr_select ==1 || latContr_select == 4
        % set kus if clothoid controller
        if latContr_select == 4
            kus = kus_table(round(kus_table(:,1))==init_speed_k,2);
        end
        for ind2 = 1: length(look_ahead_list) % loop through look ahead values
            look_ahead = look_ahead_list(ind2);
            % if else statement to skip known tests that throws errors
            if latContr_select == 1 && ((speed_req_k ==70 &&  look_ahead < 15) || (speed_req_k ==80 &&  look_ahead <= 15) || (speed_req_k ==90 &&  look_ahead <= 20)|| (speed_req_k ==100 &&  look_ahead <= 25))
            elseif latContr_select == 4 && ((speed_req_k ==70 &&  look_ahead == 15) || (speed_req_k ==80 &&  look_ahead < 20) || (speed_req_k >=90 &&  look_ahead < 20) )
            else
                total = total +1;
                % re-set clothoid params only if its chosen
                if latContr_select == 4
                    clothoidBasedParams = clothoidBasedControllerParams(kus,look_ahead);
                end    
                % re-set pure pursuit params
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
end
% --------------------------------------
%% speed / look-up combination vs. error
% --------------------------------------

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
    
    % to output results to csv
    T = array2table(error_speed_lookahead_table);
    T.Properties.VariableNames(1:7) = {'u','look_ahead','max_error','reached_flg','steps_to_finish','mean_error','std_error'};
    output = 'results/error_results_table_%d.csv';
    writetable(T,sprintf(output,latContr_select));
    
% ------------------------------
%% Data restructure for plotting
% ------------------------------
    % choose what to plot
    la_to_plot = look_ahead_list;
    s_to_plot = req_speeds_k;
    
%     la_to_plot = [3,10,15,20,25,30];
%     la_to_plot = [5,10,15,20,25,30];
%     la_to_plot = [0.1,0.5,1];
    s_to_plot = [30,70];

    % create a pivot table of the error data to plot based on speed and
    % [look ahead or gain]
    pivot_table = zeros(length(s_to_plot),length(la_to_plot)+1);
    check_reached = false;
    for k =1:length(s_to_plot)
        pivot_table(k,1) = s_to_plot(k);
        for j =1:length(la_to_plot)
            for i = 1:length(error_data)
                if error_data{i}.reached_flag || ~check_reached
                    if error_data{i}.speed == s_to_plot(k)
                        if error_data{i}.look_ahead == la_to_plot(j)
                            pivot_table(k,j+1) = error_data{i}.e_max;
                        end
                    end
                end
            end
        end
    end
% ----------
%% Bar graph
% ----------
    % flipud(winter(7)
    if latContr_select ==1 || latContr_select == 4
        leg_name = 'look ahead [m]';
    else
        leg_name = 'gain value';
    end
    figure('Name','Tracking error','NumberTitle','off'); clf;
    
    set(0,'DefaultAxesColorOrder',parula(length(la_to_plot)+1))
    
    bar(pivot_table(:,1),pivot_table(:,2:end));
    grid on
    xlabel('vehicle speed (u) [$km/h$]')
    ylabel('tracking error [m]')
    yticks((0:2:16))     
    ylim([0 16])
    xlim([10 110])
    set(gca,'fontsize',26)
    hleg = legend(string(la_to_plot),'location','NW');
    htitle = get(hleg,'Title');
    set(htitle,'String',leg_name)

    pbaspect([1 1 1])
%     % box to display max steer angle
%     string_t1 = {'20-70 km/h','80-90 km/h','100   km/h'};
%     string_t2 = {'6','4','3'};
%     t1 = annotation('textbox', [0.2915, 0.57, 0.1, 0.1],'String', 'Max Steer');
%     t1.BackgroundColor = 'w';
%     t1.HorizontalAlignment = 'center';
%     t1.FontName = 'FixedWidth';
%     t1.FontSize = 17;
%     t2 = annotation('textbox', [0.2737, 0.5159, 0.1, 0.1],'String', string_t1);
%     t2.BackgroundColor = 'w';
%     t2.FontName = 'FixedWidth';
%     t2.FontSize = 12;
%     t3 = annotation('textbox', [0.3440, 0.5159, 0.1, 0.1],'String', string_t2);
%     t3.BackgroundColor = 'w';
%     t3.HorizontalAlignment = 'center';
%     t3.FontName = 'FixedWidth';
%     t3.FontSize = 12;
    exportgraphics(gcf,sprintf(output_file,q,q,'a',latContr_select),'ContentType','vector')
    
%% to extract best look aheads based on speed

p = pivot_table;
p(p==0) = nan;
[MN,I] = min(p,[],2);
p_f = look_ahead_list(I-1);
lookahead_lookup = [pivot_table(:,1) p_f'];
%% pure_pursuit_look_up - ../exercise6/results/p_pursuit_look_up
