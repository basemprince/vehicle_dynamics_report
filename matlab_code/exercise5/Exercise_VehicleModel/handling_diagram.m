function handling_data = handling_diagram(model_sim,vehicle_data,Ts,counter)

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
    
    %% for saving 
    output_file = '../graphs/q%d/ex-5%d%s.eps';
    output_files = '../graphs/q%d/ex-5%d%s%d.eps';
    q = 2;
    %% ----------------------------------------------------------------
    % Handling diagram fitting
    % ----------------------------------------------------------------
    % Y = delta - (Omega*L./u);
    delta_D_r = deg2rad(delta_D);
    delta_t = deg2rad(desired_steer_atWheel);
    Y = delta_t - (Omega*L./u);

    % define the fitting law
%     fitting_law = @(c,x)(c(1)*x + c(2)*x.^2 + c(3).x.^3 + c(4));  
    % define the initial conditions
%     c0 = [0,0];  
    % define the fitting algorithm and fitting options
%     options = optimoptions('lsqcurvefit','MaxFunctionEvaluations',50000,'MaxIterations',50000,'FunctionTolerance',1e-9,...
%         'StepTolerance',1e-9,'Display','final-detailed');
    % fit the data
%     [optim_coeffs,resnorm,~,exitflag,output] = lsqcurvefit(fitting_law,c0,[0;Ay],Y,[],[],options); 
    % extract the optimal fitting coefficients
%     c_1 = optim_coeffs(1);
%     c_2 = optim_coeffs(2);
%     c_3 = optim_coeffs(3);
%     c_4 = optim_coeffs(3);
%     fitted_Y = c_1*[0;Ay] + c_2*[0;Ay].^2 + c_3*[0;Ay].^3 + c_4;
    color = ['k', 'g', 'r', 'c', 'y', 'k'];
    c_counter = 1;
    figure('Name',append('Fitting results ', string(counter)),'NumberTitle','off'), clf
    plot([0;Ay],Y,'-b','LineWidth',2,'displayName','real')
    hold on
    first = 1; last = 5;
    threshold_min = 9*10^-4;
    threshold_max = 1*10^-6;
    for ind = first:last
        [coeffs,S,mu] = polyfit([0;Ay],Y,ind);
        [fitted_Y, error] = polyval(coeffs,[0;Ay],S,mu);
        error = mean(error);
%         disp(error);
        if error < threshold_min && error > threshold_max && c_counter <= 3
            saved_coeffs = coeffs;
            plot([0;Ay],fitted_Y,append(':',color(c_counter)),'LineWidth',2,'displayName',append('fitted ',iptnum2ordinal(ind), ' order'));
            c_counter = c_counter +1;
        end
    end
    
    hold off
    grid on
    legend('location','southeast');
    xlabel('$a_y [m^2 / s]$')
    ylabel('Y')
    legend('location','northeast','fontSize', 26);
    hold off
    pbaspect([1 1 1])
    set(gca,'FontSize',26)
    exportgraphics(gcf,sprintf(output_files,q,q,'e',counter),'ContentType','vector')
    
    handling_data = struct();
    handling_data.Y = Y;
    handling_data.fitted_Y= fitted_Y;
    handling_data.coeffs = saved_coeffs;
    
    
  end
    
