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
steer_angle = 5; % degrees
freq = 0.0062831853; % rad/s = [0.001 hz]
req_speeds = [50,80,100]/3.6;  
% ----------------------------
%% Start Simulation
% ----------------------------
model_sims = {};
handling_datas = {};
for ind=1:length(req_speeds)
    req_speed = req_speeds(ind);
    fprintf('Starting Simulation # %d [%d km/h]\n', ind, req_speed*3.6)
    tic;
    model_sim = sim('Vehicle_Model_2Track');
    elapsed_time_simulation = toc;
    fprintf('Simulation completed\n')
    fprintf('The total simulation time was %.2f seconds\n',elapsed_time_simulation)
    handling_data = handling_diagram(model_sim,vehicle_data,Ts);
    model_sims{ind} = model_sim;
    handling_datas{ind} = handling_data;
    dataAnalysis(model_sim,vehicle_data,Ts);
    % ----------------------------
    %% Post-Processing
    % ----------------------------
    
end


graphing(model_sims,handling_datas,vehicle_data,Ts);

