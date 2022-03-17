function handling_data = handling_diagram(model_sim,vehicle_data,Ts)

    % ---------------------------------
    %% Extract data from simulink model
    % ---------------------------------
    u          = model_sim.states.u.data;
    v          = model_sim.states.v.data;
    tau_D = vehicle_data.steering_system.tau_D;  % [-] steering system ratio (pinion-rack)
    L  = vehicle_data.vehicle.L;   % [m] Vehicle length
    delta_D    = model_sim.inputs.delta_D.data;
    Omega      = model_sim.states.Omega.data;
    % Desired sinusoidal steering angle for the equivalent single track front wheel
    desired_steer_atWheel = delta_D/tau_D;

    % -----------------
    % Accelerations
    % -----------------
    % Derivatives of u, v [m/s^2]
    dot_u = diff(u)/Ts;
    dot_v = diff(v)/Ts;
    % Total longitudinal and lateral accelerations
    Ax = dot_u(1:end) - Omega(2:end).*v(2:end);
    Ay = dot_v(1:end) + Omega(2:end).*u(2:end);
    
    %% ----------------------------------------------------------------
    % Handling diagram fitting
    % ----------------------------------------------------------------
    % Y = delta - (Omega*L./u);
    delta_D_r = deg2rad(delta_D);
    delta_t = deg2rad(desired_steer_atWheel);
    Y = delta_t - (Omega*L./u);

    % define the fitting law
    fitting_law = @(c,x)(c(1)*x + c(2)*x.^2);  
    % define the initial conditions
    c0 = [0,0];  
    % define the fitting algorithm and fitting options
    options = optimoptions('lsqcurvefit','MaxFunctionEvaluations',50000,'MaxIterations',50000,'FunctionTolerance',1e-9,...
        'StepTolerance',1e-9,'Display','final-detailed');
    % fit the data
    [optim_coeffs,resnorm,~,exitflag,output] = lsqcurvefit(fitting_law,c0,[0;Ay],Y,[],[],options); 
    % extract the optimal fitting coefficients
    c_1 = optim_coeffs(1);
    c_2 = optim_coeffs(2);
    fitted_Y = c_1*[0;Ay] + c_2*[0;Ay].^2;
    
    handling_data = struct();
    handling_data.Y = Y;
    handling_data.fitted_Y= fitted_Y;
    
    
  end
    
