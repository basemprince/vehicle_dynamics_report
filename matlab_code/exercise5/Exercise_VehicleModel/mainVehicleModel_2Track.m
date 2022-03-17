% ----------------------------------------------------------------
%% Main script for a basic simulation framework with a Formula SAE vehicle model 
%  authors: Mattia Piccinini & Gastone Pietro Papini Rosati
%  email:   mattia.piccinini@unitn.it & gastone.rosatipapini@unitn.it
%  date:    13/10/2020
% ----------------------------------------------------------------

% ----------------------------
%% Initialization
% ----------------------------
initialize_environment;

% ----------------------------
%% Load vehicle data
% ----------------------------
vehicle_data = getVehicleDataStruct();
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


%% Variables to loop through
freq = 0.001; % 1/s
req_steer_angles = [10,24]; % degrees
req_speeds = [40,80]/3.6;
init_speeds = [20,50]/3.6;

% freq = 0.001; % 1/s
% req_steer_angles = [70,24,12]; % degrees
% req_speeds = [50,80,100]/3.6;
% 
% req_speed = 50/3.6;
% req_steer_angle = 70;

% ----------------------------
%% Start Simulation
% ----------------------------
model_sims = {};
handling_datas = {};
for ind=1:1
%     req_speed_in = req_speeds(ind);
%     X0(4) = init_speeds(ind);
%     req_steer_angle = req_steer_angles(ind);
%     fprintf('Starting Simulation # %d [%d km/h] and steer angle %d\n', ind, req_speed_in*3.6, req_steer_angle)
    tic;
    model_sim = sim('Vehicle_Model_2Track.slx');
    elapsed_time_simulation = toc;
    fprintf('Simulation completed\n')
    fprintf('The total simulation time was %.2f seconds\n',elapsed_time_simulation)
    handling_data = handling_diagram(model_sim,vehicle_data,Ts,ind);
    model_sims{ind} = model_sim;
    handling_datas{ind} = handling_data;
    dataAnalysis(model_sim,vehicle_data,Ts);
    % ----------------------------
    %% Post-Processing
    % ----------------------------
    
end


% graphing(model_sims,handling_datas,vehicle_data,Ts);

