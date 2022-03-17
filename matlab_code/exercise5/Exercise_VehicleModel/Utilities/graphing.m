function graphing(model_sims,handling_datas,vehicle_data,Ts)

    % ----------------------------------------------------------------
    %% Post-Processing and Data Analysis
    % ----------------------------------------------------------------
    time_sim = [model_sims{1}.states.u.time, model_sims{2}.states.u.time,model_sims{3}.states.u.time];
    % ---------------------------------
    %% Load vehicle data
    % ---------------------------------
    tau_D = vehicle_data.steering_system.tau_D;  % [-] steering system ratio (pinion-rack)
    Lf = vehicle_data.vehicle.Lf;  % [m] Distance between vehicle CoG and front wheels axle
    Lr = vehicle_data.vehicle.Lr;  % [m] Distance between vehicle CoG and front wheels axle
    L  = vehicle_data.vehicle.L;   % [m] Vehicle length
    Wf = vehicle_data.vehicle.Wf;  % [m] Width of front wheels axle 
    Wr = vehicle_data.vehicle.Wr;  % [m] Width of rear wheels axle                   
    m  = vehicle_data.vehicle.m;   % [kg] Vehicle Mass
    g  = vehicle_data.vehicle.g;   % [m/s^2] Gravitational acceleration
    % ---------------------------------
    %% Extract data from simulink model
    % ---------------------------------
    u          = [model_sims{1}.states.u.data, model_sims{2}.states.u.data,model_sims{3}.states.u.data];
    v          = [model_sims{1}.states.v.data, model_sims{2}.states.v.data,model_sims{3}.states.v.data];
    psi        = [model_sims{1}.states.psi.data, model_sims{2}.states.psi.data,model_sims{3}.states.psi.data];
    Omega      = [model_sims{1}.states.Omega.data, model_sims{2}.states.Omega.data,model_sims{3}.states.Omega.data];
    delta      = [model_sims{1}.states.delta.data, model_sims{2}.states.delta.data,model_sims{3}.states.delta.data];
    delta_D    = [model_sims{1}.inputs.delta_D.data, model_sims{2}.inputs.delta_D.data,model_sims{3}.inputs.delta_D.data];
    x_CoM      = [model_sims{1}.states.x.data, model_sims{2}.states.x.data,model_sims{3}.states.x.data];
    y_CoM      = [model_sims{1}.states.y.data, model_sims{2}.states.y.data,model_sims{3}.states.y.data];
    desired_steer_atWheel = delta_D/tau_D;
    delta_D_r = deg2rad(delta_D);
    % -----------------
    % Accelerations
    % -----------------
    % Derivatives of u, v [m/s^2]
    dot_u = diff(u)/Ts;
    dot_v = diff(v)/Ts;
    % Total longitudinal and lateral accelerations
    Ax = dot_u(1:end,:) - Omega(2:end,:).*v(2:end,:);
    Ay = dot_v(1:end,:) + Omega(2:end,:).*u(2:end,:);

    %%for saving 
    output_file = '../graphs/q%d/ex-5%d%s.eps';
    output_files = '../graphs/q%d/ex-5%d%s%d.eps';
    q = 1;
    
    % -------------------------------
    %% Plot vehicle paths
    % -------------------------------
    speeds = {'50 $km/h$','80 $km/h$','100 $km/h$'};
    color = {'-b','-r','-g'};
    figure('Name','Real Vehicle Path','NumberTitle','off'), clf
    set(gca,'FontSize',23)

    for ind = 1:length(handling_datas)
        plot(x_CoM(:,ind),y_CoM(:,ind),color{ind},'LineWidth',2,'displayName',speeds{ind})
        hold on
        N = length(time_sim(ind));
        for i = 1:floor(N/20):N
            rot_mat = [cos(psi(i,ind)) -sin(psi(i,ind)) ; sin(psi(i,ind)) cos(psi(i,ind))];
            pos_rr = rot_mat*[-Lr -Wr/2]';
            pos_rl = rot_mat*[-Lr +Wr/2]';
            pos_fr = rot_mat*[+Lf -Wf/2]';
            pos_fl = rot_mat*[+Lf +Wf/2]';
            pos = [pos_rr pos_rl pos_fl pos_fr];
            p = patch(x_CoM(i,ind) + pos(1,:),y_CoM(i) + pos(2,:),'blue');
            quiver(x_CoM(i,ind), y_CoM(i,ind), u(i,ind)*cos(psi(i,ind)), u(i,ind)*sin(psi(i,ind)), 'color', [1,0,0]);
            quiver(x_CoM(i,ind), y_CoM(i,ind), -v(i,ind)*sin(psi(i,ind)), v(i,ind)*cos(psi(i,ind)), 'color', [0.23,0.37,0.17]);
        end
        grid on
        pbaspect([1 1 1])
    end
    hold off
    xlabel('x-coord [m]')
    ylabel('y-coord [m]')
    legend('location','northeast','fontsize',23);
    exportgraphics(gcf,sprintf(output_file,q,q,'a'),'ContentType','vector')    
    clear ax;
    
    % ---------------------------------
    %% handling diagrams
    % ---------------------------------
    delta_t = deg2rad(desired_steer_atWheel);       
    for ind = 1:length(handling_datas)
        figure('Name','Handling Diagram','NumberTitle','on'), clf
        hold on
        plot([0;Ay(:,ind)],handling_datas{ind}.Y,'-b','LineWidth',2,'displayName','real')

        plot([0;Ay(:,ind)],handling_datas{ind}.fitted_Y,'--r','LineWidth',2,'displayName','fitted')
        set(gca,'fontsize',16);%, 'XAxisLocation', 'origin','YAxisLocation', 'origin');
        grid on
        xlabel('$a_y [m^2 / s]$')
        ylabel('Y')
        legend('location','northeast','fontSize', 26);
        hold off
        pbaspect([1 1 1])
        set(gca,'FontSize',26)
        exportgraphics(gcf,sprintf(output_files,q,q,'b',ind),'ContentType','vector')
    end
    
    % ---------------------------------
    %% handling diagrams [superimposed]
    % ---------------------------------   
    figure('Name','Handling Diagram','NumberTitle','off'), clf
    set(gca,'fontsize',16);%, 'XAxisLocation', 'origin','YAxisLocation', 'origin');
    for ind = 1:length(handling_datas)
        plot([0;Ay(:,ind)],handling_datas{ind}.fitted_Y,color{ind},'LineWidth',2,'displayName',speeds{ind})
        hold on
    end
    grid on
    xlabel('$a_y [m^2 / s]$')
    ylabel('Y')
    hold off
    legend('location','northeast','fontsize',23);
    set(gca,'FontSize',26)
    pbaspect([1 1 1])
    exportgraphics(gcf,sprintf(output_file,q,q,'d'),'ContentType','vector')
    %% Omega vs delta
%     for ind = 1:length(handling_datas)
%         time_step = 0:0.00100:1000;    
%         figure('Name','Omega vs Delta','NumberTitle','on'), clf
%         hold on
%         plot(time_step,Omega(:,ind),'-b','LineWidth',2,'displayName','$\Omega$')
%         plot(time_step,delta(:,ind)*10,'-r','LineWidth',2,'displayName','$\delta_{D}$')    
%     %     plot(time_step,delta_D_r,'-r','LineWidth',2,'displayName','$\delta$')
%         grid on
%         xlabel('Time [s]')
%         ylabel('rad/s')
%         legend('location','northeast','fontSize', 20);
%         pbaspect([1 1 1])      
%         exportgraphics(gcf,sprintf(output_files,q,q,'c',ind),'ContentType','vector')
%         clear ax
%     end
    %% for triple graph

    figure('Name','Tripple Graph','NumberTitle','off'), clf
    time_step = 0:0.00100:1000;
    % -- 50km/h
    ax(1) = subplot(231);
    plot([0;Ay(:,1)],handling_datas{1}.Y,'-b','LineWidth',2,'displayName','real')
    hold on
    plot([0;Ay(:,1)],handling_datas{1}.fitted_Y,'--r','LineWidth',1,'displayName','fitted')
    hold off
    title('50 km\h');
    grid on
    xlabel('$a_y [m^2 / s]$')
    ylabel('Y')
    legend('location','northeast','fontSize', 20);
    pbaspect([1 1 1])  

    % -- 80km/h
    ax(2) = subplot(232);
    plot([0;Ay(:,2)],handling_datas{2}.Y,'-b','LineWidth',2,'displayName','real')
    hold on
    plot([0;Ay(:,2)],handling_datas{2}.fitted_Y,'--r','LineWidth',1,'displayName','fitted')
    hold off
    title('80 km\h');
    grid on
    xlabel('$a_y [m^2 / s]$')
    ylabel('Y')
    legend('location','northeast','fontSize', 20);
    pbaspect([1 1 1])  
    % -- 100km/h
    ax(3) = subplot(233);
    plot([0;Ay(:,3)],handling_datas{3}.Y,'-b','LineWidth',2,'displayName','real')
    hold on
    plot([0;Ay(:,3)],handling_datas{3}.fitted_Y,'--r','LineWidth',1,'displayName','fitted')
    hold off
    title('100 km\h');
    grid on
    xlabel('$a_y [m^2 / s]$')
    ylabel('Y')
    legend('location','northeast','fontSize', 20);
    pbaspect([1 1 1])  
    ylim([-6e-3 6e-3])
    xlim([-10 10])
    set([ax(1),ax(2),ax(3)],'XTick',-10:4:10)
    linkaxes([ax(1),ax(2),ax(3)],'xy')
    hold off
  
    % -- 50km/h
    ax(4) = subplot(234);
    plot(time_step,Omega(:,1),'-b','LineWidth',2,'displayName','$\Omega$')
    hold on
    plot(time_step,delta_D_r(:,1),'-r','LineWidth',2,'displayName','$\delta_{D}$')
    hold off
    title('50 km\h');
    grid on
    xlabel('Time [s]')
    ylabel('rad/s')
    ax(4).YAxis.Exponent=-1;
    legend('location','northeast','fontSize', 20);
    pbaspect([1 1 1])  

    % -- 80km/h
    ax(5) = subplot(235);
    plot(time_step,Omega(:,2),'-b','LineWidth',2,'displayName','$\Omega$')
    hold on
    plot(time_step,delta_D_r(:,2),'-r','LineWidth',2,'displayName','$\delta_{D}$')
    hold off
    title('80 km\h');
    grid on
    xlabel('Time [s]')
    ylabel('rad/s')
    ax(5).YAxis.Exponent=-1;
    legend('location','northeast','fontSize', 20);
    pbaspect([1 1 1])  

    % -- 100km/h
    ax(6) = subplot(236);
    plot(time_step,Omega(:,3),'-b','LineWidth',2,'displayName','$\Omega$')
    hold on
    plot(time_step,delta_D_r(:,3),'-r','LineWidth',2,'displayName','$\delta_{D}$')
    hold off
    title('100 km\h');
    grid on
    xlabel('Time [s]')
    ylabel('rad/s')
    legend('location','northeast','fontSize', 20);
    pbaspect([1 1 1]) 
    ax(6).YAxis.Exponent=-1;
    set([ax(4),ax(5),ax(6)],'XTick',0:200:1000)
    set(ax,'FontSize',15)
    linkaxes([ax(4),ax(5),ax(6)],'xy')    

    set(ax(1),'position',[.0 .55 .35 .35])
    set(ax(2),'position',[.2 .55 .35 .35])
    set(ax(3),'position',[0.4 .55 .35 .35])
    set(ax(4),'position',[.0 .1 .35 .35])
    set(ax(5),'position',[.2 .1 .35 .35])
    set(ax(6),'position',[.4 .1 .35 .35])
    exportgraphics(gcf,sprintf(output_file,q,q,'c'),'ContentType','vector')
    clear ax
end
    
