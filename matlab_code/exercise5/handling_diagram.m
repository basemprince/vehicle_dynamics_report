function handling_data = handling_diagram(model_sim,vehicle_data,Ts,counter,speed,req_steer_angle)
    
    fit = true;
    stead_state = false;
    % ----------------------------------------------------------------
    %% Find steady state
    % ----------------------------------------------------------------
    limit = 0.003;
    if stead_state
        first_ind  = find(diff(model_sim.states.u.Data) >= limit, 1, 'last');
    else
        first_ind  = 1;
    end

    % ---------------------------------
    %% Extract data from simulink model
    % ---------------------------------
    u          = model_sim.states.u.data(first_ind:end);
    v          = model_sim.states.v.data(first_ind:end);
    tau_D      = vehicle_data.steering_system.tau_D;  % [-] steering system ratio (pinion-rack)
    L          = vehicle_data.vehicle.L;   % [m] Vehicle length
    delta_D    = model_sim.inputs.delta_D.data(first_ind:end);
    Omega      = model_sim.states.Omega.data(first_ind:end);
    % Desired sinusoidal steering angle for the equivalent single track front wheel
    desired_steer_atWheel = delta_D/tau_D;

    % -----------------
    %% Accelerations
    % -----------------
    % Derivatives of u, v [m/s^2]
    dot_u = diff(u)/Ts;
    dot_v = diff(v)/Ts;
    % Total longitudinal and lateral accelerations
    Ax = dot_u(1:end) - Omega(2:end).*v(2:end);
    Ay = dot_v(1:end) + Omega(2:end).*u(2:end);
    Ay = [Ay(1);Ay];
    % -----------
    %% For saving
    % -----------
    output_file = '../graphs/q%d/ex-5%d%s.eps';
    output_files = '../graphs/q%d/ex-5%d%s%d.eps';
    q = 4;
    % -------------------------
    %% Handling diagram fitting
    % -------------------------
    delta_D_r = deg2rad(delta_D);
    delta_t = deg2rad(desired_steer_atWheel);
    Y = delta_t - (Omega*L./u);
    fitted_Y = Y;
    saved_coeffs = 0;
    color = ['k', 'g', 'r', 'c', 'y', 'm'];
    c_counter = 1;
    
    figure('Name',append('Fitting results ', string(counter)),'NumberTitle','off'), clf
    plot(Ay,Y,'-b','LineWidth',2,'displayName','real')
    hold on
    if (fit)
        first = 1; last = 1;
        threshold_min = 9*10^-4;
        threshold_max = 1*10^-6;
        for ind = first:last
            [coeffs,S,mu] = polyfit(Ay,Y,ind);
            [fitted_Y, error] = polyval(coeffs,Ay,S,mu);
            error = mean(error);
            disp(error);
%             if error < threshold_min && error > threshold_max && c_counter <= 3
                saved_coeffs = coeffs;
                plot(Ay,fitted_Y,append(':',color(c_counter)),'LineWidth',2,'displayName',append('fitted ',iptnum2ordinal(ind), ' order'));
                c_counter = c_counter +1;
%             end
        end

        legend('location','northeast','fontSize', 26);
    end
    hold off
    grid on
    xlabel('$a_y [m^2 / s]$')
    ylabel('Y')
%     xlim([10 17]);
%     xticks([10 11 12 13 14 15 16 17 18])
    pbaspect([1 1 1])
    set(gca,'FontSize',26)
%     exportgraphics(gcf,sprintf(output_files,q,q,'e',counter),'ContentType','vector')
    
    handling_data = struct();
    handling_data.speed = speed;
    handling_data.Ay = Ay;
    handling_data.steer_angle = req_steer_angle;
    handling_data.Y = Y;
    handling_data.fitted_Y= fitted_Y;
    handling_data.coeffs = saved_coeffs;
    handling_data.kus = saved_coeffs(1);
    
    
  end
    
