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
%% Simulation parameters
% ----------------------------
simulationPars = getSimulationParams(); 
Ts = simulationPars.times.step_size;  % integration step for the simulation (fixed step)
T0 = simulationPars.times.t0;         % starting time of the simulation
Tf = simulationPars.times.tf;         % stop time of the simulation

% ----------------------------
%% Start Simulation
% ----------------------------
fprintf('Starting Simulation\n')
tic;
model_sim = sim('Vehicle_Model_2Track');
elapsed_time_simulation = toc;
fprintf('Simulation completed\n')
fprintf('The total simulation time was %.2f seconds\n',elapsed_time_simulation)

% ----------------------------
%% Post-Processing
% ----------------------------
dataAnalysis(model_sim,vehicle_data,Ts);
