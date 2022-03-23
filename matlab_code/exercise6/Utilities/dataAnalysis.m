function [error_central] = dataAnalysis(model_sim,vehicle_data,Ts,road_data_sampled,speed_req_k,look_ahead)

    plot_path = true;
    % ----------------------------------------------------------------
    %% Post-Processing and Data Analysis
    % ----------------------------------------------------------------

    % ---------------------------------
    %% Load vehicle data
    % ---------------------------------
    Lf = vehicle_data.vehicle.Lf;  % [m] Distance between vehicle CoG and front wheels axle
    Lr = vehicle_data.vehicle.Lr;  % [m] Distance between vehicle CoG and front wheels axle
    L  = vehicle_data.vehicle.L;   % [m] Vehicle length
    Wf = vehicle_data.vehicle.Wf;  % [m] Width of front wheels axle 
    Wr = vehicle_data.vehicle.Wr;  % [m] Width of rear wheels axle                   
    m  = vehicle_data.vehicle.m;   % [kg] Vehicle Mass
    g  = vehicle_data.vehicle.g;   % [m/s^2] Gravitational acceleration
    tau_D = vehicle_data.steering_system.tau_D;  % [-] steering system ratio (pinion-rack)

    % ---------------------------------
    %% Extract data from simulink model
    % ---------------------------------
    time_sim = model_sim.states.u.time;
    dt = time_sim(2)-time_sim(1);

    % -----------------
    % Inputs
    % -----------------
    ped_0      = model_sim.inputs.ped_0.data;
    delta_D    = squeeze(model_sim.inputs.delta_D.data);

    % -----------------
    % States
    % -----------------
    x_CoM      = model_sim.states.x.data;
    y_CoM      = model_sim.states.y.data;
    psi        = model_sim.states.psi.data;
    u          = model_sim.states.u.data;
    v          = model_sim.states.v.data;
    Omega      = squeeze(model_sim.states.Omega.data);
    Fz_rr      = model_sim.states.Fz_rr.data;
    Fz_rl      = model_sim.states.Fz_rl.data;
    Fz_fr      = model_sim.states.Fz_fr.data;
    Fz_fl      = model_sim.states.Fz_fl.data;
    delta      = squeeze(model_sim.states.delta.data);
    omega_rr   = model_sim.states.omega_rr.data;
    omega_rl   = model_sim.states.omega_rl.data;
    omega_fr   = model_sim.states.omega_fr.data;
    omega_fl   = model_sim.states.omega_fl.data;
    alpha_rr   = model_sim.states.alpha_rr.data;
    alpha_rl   = model_sim.states.alpha_rl.data;
    alpha_fr   = model_sim.states.alpha_fr.data;
    alpha_fl   = model_sim.states.alpha_fl.data;
    kappa_rr   = model_sim.states.kappa_rr.data;
    kappa_rl   = model_sim.states.kappa_rl.data;
    kappa_fr   = model_sim.states.kappa_fr.data;
    kappa_fl   = model_sim.states.kappa_fl.data;
    ped        = model_sim.states.pedal.data;

    % -----------------
    % Extra Parameters
    % -----------------
    Tw_rr      = model_sim.extra_params.Tw_rr.data;
    Tw_rl      = model_sim.extra_params.Tw_rl.data;
    Tw_fr      = model_sim.extra_params.Tw_fr.data;
    Tw_fl      = model_sim.extra_params.Tw_fl.data;
    Fx_rr      = model_sim.extra_params.Fx_rr.data;
    Fx_rl      = model_sim.extra_params.Fx_rl.data;
    Fx_fr      = model_sim.extra_params.Fx_fr.data;
    Fx_fl      = model_sim.extra_params.Fx_fl.data;
    Fy_rr      = model_sim.extra_params.Fy_rr.data;
    Fy_rl      = model_sim.extra_params.Fy_rl.data;
    Fy_fr      = model_sim.extra_params.Fy_fr.data;
    Fy_fl      = model_sim.extra_params.Fy_fl.data;
    Mz_rr      = model_sim.extra_params.Mz_rr.data;
    Mz_rl      = model_sim.extra_params.Mz_rl.data;
    Mz_fr      = model_sim.extra_params.Mz_fr.data;
    Mz_fl      = model_sim.extra_params.Mz_fl.data;
    gamma_rr   = model_sim.extra_params.gamma_rr.data;
    gamma_rl   = model_sim.extra_params.gamma_rl.data;
    gamma_fr   = model_sim.extra_params.gamma_fr.data;
    gamma_fl   = model_sim.extra_params.gamma_fl.data;
    delta_fr   = model_sim.extra_params.delta_fr.data;
    delta_fl   = model_sim.extra_params.delta_fl.data;

    % Chassis side slip angle beta [rad]
    beta = atan(v./u);

    % -----------------
    % Accelerations
    % -----------------
    % Derivatives of u, v [m/s^2]
    dot_u = diff(u)/Ts;
    dot_v = diff(v)/Ts;
    % Total longitudinal and lateral accelerations
    Ax = dot_u(1:end) - Omega(2:end).*v(2:end);
    Ay = dot_v(1:end) + Omega(2:end).*u(2:end);
    % Ax low-pass filtered signal (zero-phase digital low-pass filtering)
    Wn_filter = 0.01;
    [b_butt,a_butt] = butter(4,Wn_filter,'low');
    Ax_filt = filtfilt(b_butt,a_butt,Ax);  
    dot_u_filt = filtfilt(b_butt,a_butt,dot_u);  
    % Steady state lateral acceleration
    Ay_ss = Omega.*u;
    % Longitudinal jerk [m/s^3]
    jerk_x = diff(dot_u)/Ts;

    % -----------------
    % Other parameters
    % -----------------
    % Total CoM speed [m/s]
    vG = sqrt(u.^2 + v.^2);
    % Steady state and transient curvature [m]
    rho_ss   = Omega./vG;
    rho_tran = ((dot_v.*u(1:end-1) - dot_u.*v(1:end-1)) ./ ((vG(1:end-1)).^3)) + rho_ss(1:end-1);
    % Desired sinusoidal steering angle for the equivalent single track front wheel
    desired_steer_atWheel = delta_D/tau_D;

    % -----------------
    %% For saving Plots
    % -----------------
    output_file = './graphs/q%d/ex-6%d%s.eps';
    q = 1;              

    % --------------------
    %% Plot vehicle inputs
    % --------------------
%     figure('Name','Inputs','NumberTitle','off'), clf   
%     % --- pedal --- %
%     ax(1) = subplot(121);
%     hold on
%     plot(time_sim,ped_0,'b','LineWidth',2)
%     plot(time_sim,ped,'r','LineWidth',2)
%     grid on
%     ylabel('pedal $p_0$ [-]')
%     legend('control','state (1st order dyn)')
%     xlabel('Time (s)')
%     xlim([0 time_sim(end)])
%     pbaspect([1 1 1]);
%     % --- delta_0 --- %
%     ax(2) = subplot(122);
%     plot(time_sim,delta_D,'b','LineWidth',2)
%     grid on
%     ylabel('steering angle $\delta_D$ [deg]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1]);
% %     exportgraphics(gcf,sprintf(output_file,q,q,'a'),'ContentType','vector')
%     % --------------------
%     %% Plot vehicle motion
%     % --------------------
%     limiter = time_sim(end);
%     figure('Name','veh motion','NumberTitle','off'), clf   
%     % --- u --- %
%     ax(1) = subplot(141);
%     hold on;
%     plot(time_sim,u*3.6,'b','LineWidth',2,'displayName', 'real')
%     plot(time_sim,squeeze(model_sim.req_speed.data)*3.6,'--','LineWidth',2,'displayName','target')
%     grid on
%     legend('Location','southeast')
%     ylabel('$u$ [km/h]')
%     xlim([0 limiter])
%     xlabel('Time (s)')
%     xticks([2 4 6 8 10])
%     pbaspect([1 1 1]);
%     hold off
%     % --- v --- %
%     ax(2) = subplot(142);
%     plot(time_sim,v,'b','LineWidth',2)
%     grid on
%     ylabel('$v$ [m/s]')
%     xlim([0 limiter])
%     xlabel('Time (s)')
%     xticks([2 4 6 8 10])
%     pbaspect([1 1 1]);
%     % --- Omega --- %
%     ax(3) = subplot(143);
%     plot(time_sim,Omega,'b','LineWidth',2)
%     grid on
%     ylabel('$\Omega$ [rad/s]')
%     xlim([0 limiter])
%     xticks([2 4 6 8 10])
%     xlabel('Time (s)')
%     pbaspect([1 1 1]);
%     % --- ax --- %
%     ax(4) = subplot(144);
% %     plot(time_sim(2:end),dot_u - Omega(2:end).*v(2:end),'m','LineWidth',2,'displayName','$\dot{u}-\Omega v$')
% %     hold on
% %     plot(time_sim(2:end),diff(u)/Ts,'--g','LineWidth',2,'displayName','$\dot{u}$')
% %     plot(time_sim(2:end),Ax_filt,'-.b','LineWidth',1,'displayName','filt $\dot{u}-\Omega v$')
%     plot(time_sim(2:end),dot_u_filt,'b','LineWidth',2,'displayName','filt $\dot{u}$')
% %     hold off
%     grid on
%     ylabel('$a_{x}$ $[m/s^2]$')
% %     legend('Location','northeast')
%     xlim([0 limiter])
%     xticks([2 4 6 8 10])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     exportgraphics(gcf,sprintf(output_file,q,q,'b'),'ContentType','vector')
%     % ---------------------
%     %% Plot steering angles
%     % ---------------------
    figure('Name','steer','NumberTitle','off'), clf   
    % --- delta_0 --- %
    ax(1) = subplot(221);
    plot(time_sim,delta_D,'b','LineWidth',2)
    grid on
    ylabel('$\delta_0$ [deg]')
    xlim([0 time_sim(end)])
    xlabel('Time (s)')
    pbaspect([1 1 1]);
    % --- delta_fr --- %
    ax(2) = subplot(222);
    plot(time_sim,delta_fr,'b','LineWidth',2)
    grid on
    ylabel('$\delta_{fr}$ [rad]')
    xlim([0 time_sim(end)])
    xlabel('Time (s)')
    pbaspect([1 1 1]);
    % --- delta_fl --- %
    ax(3) = subplot(223);
    plot(time_sim,delta_fl,'b','LineWidth',2)
    grid on
    ylabel('$\delta_{fl}$ [rad]')
    xlim([0 time_sim(end)])
    xlabel('Time (s)')
    pbaspect([1 1 1]);
    % --- comparison --- %
    ax(4) = subplot(224);
    plot(time_sim,rad2deg(delta_D)/tau_D,'b','LineWidth',2,'displayName','$\delta_D/\tau_D$')
    hold on
    plot(time_sim,delta_fr,'r','LineWidth',2,'displayName','$\delta_{fr}$')
    plot(time_sim,delta_fl,'m','LineWidth',2,'displayName','$\delta_{fl}$')
    hold off
    grid on
    legend('location','northeast')
    xlim([0 time_sim(end)])
    ylabel('$\delta$ [rad]');
    xlabel('Time (s)')
    pbaspect([1 1 1]);
    set(ax(1),'position',[.0 .55 .35 .35])
    set(ax(2),'position',[.0 .1 .35 .35])
    set(ax(3),'position',[.25 .55 .35 .35])
    set(ax(4),'position',[.25 .1 .35 .35])
% %     exportgraphics(gcf,sprintf(output_file,q,q,'c'),'ContentType','vector')
%     
%     
%     % -------------------------------------------
%     %% Plot lateral tire slips and lateral forces
%     % -------------------------------------------
%     figure('Name','Lateral slips & forces','NumberTitle','off'), clf
%     % --- alpha_rr --- %
%     ax(1) = subplot(241);
%     plot(time_sim,alpha_rr,'b','LineWidth',2)
%     grid on
%     ylabel('$\alpha_{rr}$ [deg]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1]);
%     % --- alpha_rl --- %
%     ax(2) = subplot(242);
%     plot(time_sim,alpha_rl,'m','LineWidth',2)
%     grid on
%     ylabel('$\alpha_{rl}$ [deg]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1]);
%     % --- alpha_fr --- %
%     ax(3) = subplot(243);
%     plot(time_sim,alpha_fr,'b','LineWidth',2)
%     grid on
%     ylabel('$\alpha_{fr}$ [deg]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1]);
%     % --- alpha_fl --- %
%     ax(4) = subplot(244);
%     plot(time_sim,alpha_fl,'m','LineWidth',2)
%     grid on
%     ylabel('$\alpha_{fl}$ [deg]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1]);
%     % --- Fy_rr --- %
%     ax(5) = subplot(245);
%     plot(time_sim,Fy_rr,'b','LineWidth',2)
%     grid on
%     ylabel('$Fy_{rr}$ [N]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1]);
%     % --- Fy_rl --- %
%     ax(6) = subplot(246);
%     plot(time_sim,Fy_rl,'m','LineWidth',2)
%     grid on
%     ylabel('$Fy_{rl}$ [N]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1]);
%     % --- Fy_fr --- %
%     ax(7) = subplot(247);
%     plot(time_sim,Fy_fr,'b','LineWidth',2)
%     grid on
%     ylabel('$Fy_{fr}$ [N]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1]);
%     % --- Fy_fl --- %
%     ax(8) = subplot(248);
%     plot(time_sim,Fy_fl,'m','LineWidth',2)
%     grid on
%     ylabel('$Fy_{fl}$ [N]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1]);
%     linkaxes([ax(1) ax(2) ax(3) ax(4)],'xy');
%     linkaxes([ax(5) ax(6) ax(7) ax(8)],'xy');
% %     exportgraphics(gcf,sprintf(output_file,q,q,'d'),'ContentType','vector')
%     % linkaxes(ax,'x')
%     clear ax
% 
%     % -----------------------------------------------------
%     %% Plot longitudinal tire slips and longitudinal forces
%     % -----------------------------------------------------
%     figure('Name','Long slips & forces','NumberTitle','off'), clf
%     % --- kappa_rr --- %
%     ax(1) = subplot(241);
%     plot(time_sim,kappa_rr,'b','LineWidth',2)
%     grid on
%     ylabel('$\kappa_{rr}$ [-]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- kappa_rl --- %
%     ax(2) = subplot(242);
%     plot(time_sim,kappa_rl,'m','LineWidth',2)
%     grid on
%     ylabel('$\kappa_{rl}$ [-]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- kappa_fr --- %
%     ax(3) = subplot(243);
%     plot(time_sim,kappa_fr,'b','LineWidth',2)
%     grid on
%     ylabel('$\kappa_{fr}$ [-]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- kappa_fl --- %
%     ax(4) = subplot(244);
%     plot(time_sim,kappa_fl,'m','LineWidth',2)
%     grid on
%     ylabel('$\kappa_{fl}$ [-]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     
%     linkaxes([ax(1) ax(2) ax(3) ax(4)],'xy');
%     % --- Fx_rr --- %
%     ax(5) = subplot(245);
%     plot(time_sim,Fx_rr,'b','LineWidth',2)
%     grid on
%     ylabel('$Fx_{rr}$ [N]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- Fx_rl --- %
%     ax(6) = subplot(246);
%     plot(time_sim,Fx_rl,'m','LineWidth',2)
%     grid on
%     ylabel('$Fx_{rl}$ [N]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- Fx_fr --- %
%     ax(7) = subplot(247);
%     plot(time_sim,Fx_fr,'b','LineWidth',2)
%     grid on
%     ylabel('$Fx_{fr}$ [N]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- Fx_fl --- %
%     ax(8) = subplot(248);
%     plot(time_sim,Fx_fl,'m','LineWidth',2)
%     grid on
%     ylabel('$Fx_{fl}$ [N]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     linkaxes([ax(5) ax(6) ax(7) ax(8)],'xy');
%     
% %     exportgraphics(gcf,sprintf(output_file,q,q,'f'),'ContentType','vector')
%     % linkaxes(ax,'x')
%     clear ax
% 
%     % -----------------------------------
%     %% Plot wheel torques and wheel rates
%     % -----------------------------------
%     figure('Name','Wheel rates & torques','NumberTitle','off'), clf
%     % --- omega_rr --- %
%     ax(1) = subplot(241);
%     plot(time_sim,omega_rr,'b','LineWidth',2)
%     grid on
%     ylabel('$\omega_{rr}$ [rad/s]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- omega_rl --- %
%     ax(2) = subplot(242);
%     plot(time_sim,omega_rl,'m','LineWidth',2)
%     grid on
%     ylabel('$\omega_{rl}$ [rad/s]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- omega_fr --- %
%     ax(3) = subplot(243);
%     plot(time_sim,omega_fr,'b','LineWidth',2)
%     grid on
%     ylabel('$\omega_{fr}$ [rad/s]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- omega_fl --- %
%     ax(4) = subplot(244);
%     plot(time_sim,omega_fl,'m','LineWidth',2)
%     grid on
%     ylabel('$\omega_{fl}$ [rad/s]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     linkaxes([ax(1) ax(2) ax(3) ax(4)],'xy');
%     % --- Tw_rr --- %
%     ax(5) = subplot(245);
%     plot(time_sim,Tw_rr,'b','LineWidth',2)
%     grid on
%     ylabel('$Tw_{rr}$ [Nm]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- Tw_rl --- %
%     ax(6) = subplot(246);
%     plot(time_sim,Tw_rl,'m','LineWidth',2)
%     grid on
%     ylabel('$Tw_{rl}$ [Nm]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- Tw_fr --- %
%     ax(7) = subplot(247);
%     plot(time_sim,Tw_fr,'b','LineWidth',2)
%     grid on
%     ylabel('$Tw_{fr}$ [Nm]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- Tw_fl --- %
%     ax(8) = subplot(248);
%     plot(time_sim,Tw_fl,'m','LineWidth',2)
%     grid on
%     ylabel('$Tw_{fl}$ [Nm]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     linkaxes([ax(5) ax(6) ax(7) ax(8)],'xy');
% %     exportgraphics(gcf,sprintf(output_file,q,q,'g'),'ContentType','vector')
%     clear ax
% 
%     % ---------------------------------------------------
%     %% Plot vertical tire loads and self-aligning torques
%     % ---------------------------------------------------
%     figure('Name','Vert loads & aligning torques','NumberTitle','off'), clf
%     % --- Fz_rr --- %
%     ax(1) = subplot(241);
%     plot(time_sim,Fz_rr,'b','LineWidth',2)
%     grid on
%     ylabel('$Fz_{rr}$ [N]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- Fz_rl --- %
%     ax(2) = subplot(242);
%     plot(time_sim,Fz_rl,'m','LineWidth',2)
%     grid on
%     ylabel('$Fz_{rl}$ [N]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- Fz_fr --- %
%     ax(3) = subplot(243);
%     plot(time_sim,Fz_fr,'b','LineWidth',2)
%     grid on
%     ylabel('$Fz_{fr}$ [N]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- Fz_fl --- %
%     ax(4) = subplot(244);
%     plot(time_sim,Fz_fl,'m','LineWidth',2)
%     grid on
%     ylabel('$Fz_{fl}$ [N]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     linkaxes([ax(1) ax(2) ax(3) ax(4)],'xy');
%     % --- Mz_rr --- %
%     ax(5) = subplot(245);
%     plot(time_sim,Mz_rr,'b','LineWidth',2)
%     grid on
%     ylabel('$Mz_{rr}$ [Nm]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- Mz_rl --- %
%     ax(6) = subplot(246);
%     plot(time_sim,Mz_rl,'m','LineWidth',2)
%     grid on
%     ylabel('$Mz_{rl}$ [Nm]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- Mz_fr --- %
%     ax(7) = subplot(247);
%     plot(time_sim,Mz_fr,'b','LineWidth',2)
%     grid on
%     ylabel('$Mz_{fr}$ [Nm]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- Mz_fl --- %
%     ax(8) = subplot(248);
%     plot(time_sim,Mz_fl,'m','LineWidth',2)
%     grid on
%     ylabel('$Mz_{fl}$ [Nm]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     linkaxes([ax(5) ax(6) ax(7) ax(8)],'xy'); 
% %     exportgraphics(gcf,sprintf(output_file,q,q,'h'),'ContentType','vector')
%     % linkaxes(ax,'x')
%     clear ax
%     
%     % ------------------
%     %% Plot wheel camber
%     % ------------------
%     figure('Name','Wheel camber','NumberTitle','off'), clf
%     % --- gamma_rr --- %
%     ax(1) = subplot(221);
%     plot(time_sim,gamma_rr,'b','LineWidth',2)
%     grid on
%     ylabel('$\gamma_{rr}$ [deg]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- gamma_rl --- %
%     ax(2) = subplot(222);
%     plot(time_sim,gamma_rl,'b','LineWidth',2)
%     grid on
%     ylabel('$\gamma_{rl}$ [deg]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- gamma_fr --- %
%     ax(3) = subplot(223);
%     plot(time_sim,gamma_fr,'m','LineWidth',2)
%     grid on
%     ylabel('$\gamma_{fr}$ [deg]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- gamma_fl --- %
%     ax(4) = subplot(224);
%     plot(time_sim,gamma_fl,'m','LineWidth',2)
%     grid on
%     ylabel('$\gamma_{fl}$ [deg]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     linkaxes([ax(1) ax(2) ax(3) ax(4)],'xy');
%     set(ax(1),'position',[.0 .55 .35 .35])
%     set(ax(2),'position',[.0 .1 .35 .35])
%     set(ax(3),'position',[.25 .55 .35 .35])
%     set(ax(4),'position',[.25 .1 .35 .35])
%     
% %     exportgraphics(gcf,sprintf(output_file,q,q,'i'),'ContentType','vector')
%     % linkaxes(ax,'x')
%     clear ax
%     
%     % ---------------------------------
%     %% Plot accelerations, chassis side slip angle and curvature
%     % ---------------------------------
%     figure('Name','Pars extra','NumberTitle','off'), clf
%     % --- ax --- %
%     ax(1) = subplot(221);
%     plot(time_sim(2:end),dot_u - Omega(2:end).*v(2:end),'m','LineWidth',2,'displayName','$\dot{u}-\Omega v$')
%     hold on
%     plot(time_sim(2:end),diff(u)/Ts,'--g','LineWidth',2,'displayName','$\dot{u}$')
%     plot(time_sim(2:end),Ax_filt,'-.b','LineWidth',1,'displayName','filt $\dot{u}-\Omega v$')
%     plot(time_sim(2:end),dot_u_filt,'-.r','LineWidth',1,'displayName','filt $\dot{u}$')
%     hold off
%     grid on
%     ylabel('$a_{x}$ $[m/s^2]$')
%     legend('Location','northeast')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- ay --- %
%     ax(2) = subplot(222);
%     plot(time_sim(2:end),dot_v + Omega(2:end).*u(2:end),'b','LineWidth',2,'displayName','$\dot{v}+\Omega u$')
%     hold on
%     plot(time_sim(2:end),Omega(2:end).*u(2:end),'--g','LineWidth',1,'displayName','$\Omega u$')
%     hold off
%     grid on
%     ylabel('$a_{y}$ $[m/s^2]$')
%     legend('Location','southeast')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
% 
%     % --- beta --- %
%     ax(3) = subplot(223);
%     plot(time_sim,rad2deg(beta),'b','LineWidth',2)
%     grid on
%     ylabel('$\beta$ [deg]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
% 
%     % --- rho --- %
%     ax(4) = subplot(224);
%     plot(time_sim,rho_ss,'b','LineWidth',2,'displayName','$\rho_{ss}$')
%     hold on
%     plot(time_sim(1:end-1),rho_tran,'--g','LineWidth',1,'displayName','$\rho_{transient}$')
%     hold off
%     grid on
%     ylabel('$\rho$ [$m^{-1}$]')
%     legend('Location','southeast')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
% 
%     set(ax(1),'position',[.0 .55 .35 .35])
%     set(ax(2),'position',[.0 .1 .35 .35])
%     set(ax(3),'position',[.25 .55 .35 .35])
%     set(ax(4),'position',[.25 .1 .35 .35])
%     
% %     exportgraphics(gcf,sprintf(output_file,q,q,'j'),'ContentType','vector')
%     % linkaxes(ax,'x')
%     clear ax
% 
%     % --------------------------
%     %% Plot vehicle pose x,y,psi
%     % --------------------------
%     figure('Name','Pose','NumberTitle','off'), clf 
%     % --- x --- %
%     ax(1) = subplot(131);
%     plot(time_sim,x_CoM,'b','LineWidth',2)
%     grid on
%     ylabel('$x$ [m]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- y --- %
%     ax(2) = subplot(132);
%     plot(time_sim,y_CoM,'b','LineWidth',2)
%     grid on
%     ylabel('$y$ [m]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
%     % --- psi --- %
%     ax(3) = subplot(133);
%     plot(time_sim,rad2deg(psi),'b','LineWidth',2)
%     grid on
%     ylabel('$\psi$ [deg]')
%     xlim([0 time_sim(end)])
%     xlabel('Time (s)')
%     pbaspect([1 1 1])
% %     exportgraphics(gcf,sprintf(output_file,q,q,'k'),'ContentType','vector')
%     % linkaxes(ax,'x')
%     clear ax
% 
%     %---------------------------------------
%     %% Plot G-G diagram from simulation data
%     % --------------------------------------
%     figure('Name','G-G plot','NumberTitle','off'), clf
%     axis equal
%     hold on
%     plot3(Ay,Ax_filt,u(1:end-1),'Color',color('purple'),'LineWidth',3)
%     xlabel('$a_y$ [m/s$^2$]')
%     ylabel('$a_x$ [m/s$^2$]')
%     zlabel('$u$ [m/s]')
%     title('G-G diagram from simulation data','FontSize',18)
%     grid on
%     pbaspect([1 1 1])
% %     exportgraphics(gcf,sprintf(output_file,q,q,'l'),'ContentType','vector')
    % ------------------
    %% Plot vehicle path
    % ------------------
    if plot_path
        name = append('speed: ', string(speed_req_k),', look ahead: ', string(look_ahead));
        cut_off = find(x_CoM>=240,1,'first');
        if ~isempty(cut_off)
            x_CoM = x_CoM(1:cut_off);
            y_CoM = y_CoM(1:cut_off);        
        end
        real_path = [x_CoM,y_CoM];
        N = length(x_CoM);
        hh = figure('Name',name,'NumberTitle','off'); clf;
    %     set(hh, 'Visible', 'off');
        h1 = plot(x_CoM,y_CoM,'Color',color('gold'),'LineWidth',2,'displayName','real path');
        hold on
        title(name);
        h2 = plot(road_data_sampled(:,1),road_data_sampled(:,2),'-b','displayName','target path');
        for i = 1:floor(N/20):N
            rot_mat = [cos(psi(i)) -sin(psi(i)) ; sin(psi(i)) cos(psi(i))];
            pos_rr = rot_mat*[-Lr -Wr/2]';
            pos_rl = rot_mat*[-Lr +Wr/2]';
            pos_fr = rot_mat*[+Lf -Wf/2]';
            pos_fl = rot_mat*[+Lf +Wf/2]';
            pos = [pos_rr pos_rl pos_fl pos_fr];
            p = patch(x_CoM(i) + pos(1,:),y_CoM(i) + pos(2,:),'blue');
            quiver(x_CoM(i), y_CoM(i), u(i)*cos(psi(i)), u(i)*sin(psi(i)), 'color', [1,0,0]);
            quiver(x_CoM(i), y_CoM(i), -v(i)*sin(psi(i)), v(i)*cos(psi(i)), 'color', [0.23,0.37,0.17]);
        end
        grid on
        hold off
        set(gca,'fontsize',26)
        axis equal
        legend([h1,h2],'location','northeast');
        xlabel('x-coord [m]')
        ylabel('y-coord [m]')
        pbaspect([1 1 1])
    %     exportgraphics(gcf,sprintf(output_file,q,q,'m'),'ContentType','vector')
    end

    % -------------------
    %% Error Calculations
    % -------------------
    reached_threshold = 5;
    reached_target = false;
    desired_path = road_data_sampled;
    cut_off = find(x_CoM>=240,1,'first');
    if ~isempty(cut_off)
        x_CoM = x_CoM(1:cut_off);
        y_CoM = y_CoM(1:cut_off);        
    end
    real_path = [x_CoM,y_CoM];
    k = dsearchn(desired_path,real_path);
    close_path = desired_path(k,:);
    finish_line = 632;
    at_fin = find(k==finish_line);
    seconds_till_fin = 0;
    at_finish = real_path(at_fin,:);
    lowest = Inf;
    for i = 1: length(at_finish)
        cur = norm(at_finish(i,:)-desired_path(finish_line,:));
        if (cur < lowest)
            lowest = cur;
        end
    end
    
    if ( lowest < reached_threshold && ~isempty(at_finish))
        reached_target = true;
        seconds_till_fin = min(at_fin);
    end

    error_central = struct();
    e_cum = zeros(length(real_path),1);
    for ind = 1: length(real_path)
        e_cum(ind) = norm(real_path(ind,:)-close_path(ind,:));
    end
    error_central.speed = speed_req_k;
    error_central.steps_taken = seconds_till_fin;
    error_central.look_ahead = look_ahead;
    error_central.reached_flag = reached_target;
    error_central.e_cum = e_cum;
    error_central.e_max = max(e_cum);
    error_central.e_min = min(e_cum);
    error_central.e_mean = mean(e_cum);
    error_central.e_std = std(e_cum);
    error_central.delta_fr = delta_fr;
    error_central.delta_fl = delta_fl;
    error_central.time_sim = time_sim;
    
    
