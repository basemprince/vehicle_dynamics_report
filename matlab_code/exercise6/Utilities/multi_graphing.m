function multi_graphing(model_sims,handling_datas,vehicle_data,Ts)

    % ---------------------------------
    %% plotting switches
    % ---------------------------------
    print_handling_diag = false;
    superimpose = true;

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
    [time_sim, u,v,psi,Omega,delta,delta_D,x_CoM,y_CoM] = deal(zeros(length(model_sims{1}.states.u.time),length(model_sims)));
    for ind = 1:length(model_sims)
        time_sim(:,ind) = model_sims{ind}.states.u.time;
        u(:,ind)        = model_sims{ind}.states.u.data;
        v(:,ind) = model_sims{ind}.states.v.data;
        psi(:,ind) = model_sims{ind}.states.psi.data;
        Omega(:,ind) = model_sims{ind}.states.Omega.data;
        delta(:,ind) = model_sims{ind}.states.delta.data;
        delta_D(:,ind) = model_sims{ind}.inputs.delta_D.data;
        x_CoM(:,ind) = model_sims{ind}.states.x.data;
        y_CoM(:,ind) = model_sims{ind}.states.y.data;                                                    
    end
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
    Ay = [Ay(1,:);Ay];
    % ----------------
    %% For plot saving
    % ----------------
    output_file = 'graphs/q%d/ex-6%d%s.eps';
    output_files = 'graphs/q%d/ex-6%d%s%d.eps';
    q = 1;
    speeds = {};
    deltas_fig = {};
    for ind = 1:length(model_sims)
        speeds{ind} = append(string(error_data{ind}.speed), ' km/hr');
        lookahead_figs{ind} = string(error_data{ind}.look_ahead);
    end
    color = {'-b','-r','-g','-m','-k','-k','-y'};   
    
    % -------------------
    %% Plot vehicle paths
    % -------------------
    figure('Name','Real Vehicle Path','NumberTitle','off'), clf
    set(gca,'FontSize',23)
    color_counter = 0;
    speed_to_plot = 70;
    l_ahed_to_plot = [5,20,30];
    for ind = 1:length(error_data)
        if error_data{ind}.speed == speed_to_plot && ismember(error_data{ind}.look_ahead,l_ahed_to_plot)
            color_counter = color_counter +1;
            plot(x_CoM(1:11000,ind),y_CoM(1:11000,ind),color{color_counter},'LineWidth',2,'displayName',append(speeds{ind},' - ', lookahead_figs{ind}))
            hold on           
        end
    end
    grid on
    pbaspect([1 1 1])
    hold off
    xlim([0 220])
    xlabel('x-coord [m]')
    ylabel('y-coord [m]')
    legend('location','SE','fontsize',23);
    set(gca,'FontSize',26)
    exportgraphics(gcf,sprintf(output_files,q,q,'x',speed_to_plot),'ContentType','vector') 
    clear ax;
    
    % ------------------
    %% handling diagrams
    % ------------------
    if (print_handling_diag)
        delta_t = deg2rad(desired_steer_atWheel);       
        for ind = 1:length(handling_datas)
            figure('Name','Handling Diagram','NumberTitle','on'), clf
            hold on
            plot(Ay(:,ind),handling_datas{ind}.Y,'-b','LineWidth',2,'displayName','real')
            plot(Ay(:,ind),handling_datas{ind}.fitted_Y,'--r','LineWidth',2,'displayName','fitted')
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
    end
    % ---------------------------------
    %% handling diagrams [superimposed]
    % ---------------------------------  
    if (superimpose)
        figure('Name','Fitted Handling Diagram','NumberTitle','off'), clf
        col = 0;
    %     set(gca,'fontsize',16);%, 'XAxisLocation', 'origin','YAxisLocation', 'origin');
        for ind = 1:2:length(handling_datas)-1
    %         col = col +1;
    %         if ind == 1
    %             sup = 31;
    %         elseif ind == 3
    %             sup = 3.5;
    %         else 
    %             sup =0;
    %         end
    %         plot(Ay(:,ind)*(length(handling_datas)-ind+sup),handling_datas{ind}.fitted_Y*(length(handling_datas)-ind+sup),color{col},'LineWidth',2,'displayName',append(speeds{ind},' - ', deltas_fig{ind}));
            plot(Ay(:,ind),handling_datas{ind}.fitted_Y,color{col},'LineWidth',2,'displayName',append(speeds{ind},' - ', deltas_fig{ind}));      
            hold on
        end
        grid on
        xlabel('$a_y [m^2 / s]$')
        ylabel('Y')
        hold off
        legend('location','northeast','fontsize',23);
        set(gca,'FontSize',26);
    %     set(gca,'FontSize',26,'xtick',([-50 -40 -30 -20 -10 0 10 20 30 40 50]));
        ax = gca();
    %     ax.YAxis.Exponent=-1;
        ax.YRuler.Exponent = -3;
        xlim([-16 16])
        ylim([-11e-3 11e-3])
        pbaspect([1 1 1])
        exportgraphics(gcf,sprintf(output_file,q,q,'d'),'ContentType','vector')
    end
    % ----------------
    %% Omega vs. Delta
    % ----------------
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
    % ----------------
    %% Multi-graph
    % ----------------
% 
%     figure('Name','Tripple Graph','NumberTitle','off'), clf
%     time_step = 0:0.00100:1000;
%     % -- 50km/h
%     ax(1) = subplot(231);
%     plot([0;Ay(:,1)],handling_datas{1}.Y,'-b','LineWidth',2,'displayName','real')
%     hold on
%     plot([0;Ay(:,1)],handling_datas{1}.fitted_Y,'--r','LineWidth',1,'displayName','fitted')
%     hold off
%     title('50 km\h');
%     grid on
%     xlabel('$a_y [m^2 / s]$')
%     ylabel('Y')
%     legend('location','northeast','fontSize', 20);
%     pbaspect([1 1 1])  
% 
%     % -- 80km/h
%     ax(2) = subplot(232);
%     plot([0;Ay(:,2)],handling_datas{2}.Y,'-b','LineWidth',2,'displayName','real')
%     hold on
%     plot([0;Ay(:,2)],handling_datas{2}.fitted_Y,'--r','LineWidth',1,'displayName','fitted')
%     hold off
%     title('80 km\h');
%     grid on
%     xlabel('$a_y [m^2 / s]$')
%     ylabel('Y')
%     legend('location','northeast','fontSize', 20);
%     pbaspect([1 1 1])  
%     % -- 100km/h
%     ax(3) = subplot(233);
%     plot([0;Ay(:,3)],handling_datas{3}.Y,'-b','LineWidth',2,'displayName','real')
%     hold on
%     plot([0;Ay(:,3)],handling_datas{3}.fitted_Y,'--r','LineWidth',1,'displayName','fitted')
%     hold off
%     title('100 km\h');
%     grid on
%     xlabel('$a_y [m^2 / s]$')
%     ylabel('Y')
%     legend('location','northeast','fontSize', 20);
%     pbaspect([1 1 1])  
%     ylim([-6e-3 6e-3])
%     xlim([-10 10])
%     set([ax(1),ax(2),ax(3)],'XTick',-10:4:10)
%     linkaxes([ax(1),ax(2),ax(3)],'xy')
%     hold off
%   
%     % -- 50km/h
%     ax(4) = subplot(234);
%     plot(time_step,Omega(:,1),'-b','LineWidth',2,'displayName','$\Omega$')
%     hold on
%     plot(time_step,delta_D_r(:,1),'-r','LineWidth',2,'displayName','$\delta_{D}$')
%     hold off
%     title('50 km\h');
%     grid on
%     xlabel('Time [s]')
%     ylabel('rad/s')
%     ax(4).YAxis.Exponent=-1;
%     legend('location','northeast','fontSize', 20);
%     pbaspect([1 1 1])  
% 
%     % -- 80km/h
%     ax(5) = subplot(235);
%     plot(time_step,Omega(:,2),'-b','LineWidth',2,'displayName','$\Omega$')
%     hold on
%     plot(time_step,delta_D_r(:,2),'-r','LineWidth',2,'displayName','$\delta_{D}$')
%     hold off
%     title('80 km\h');
%     grid on
%     xlabel('Time [s]')
%     ylabel('rad/s')
%     ax(5).YAxis.Exponent=-1;
%     legend('location','northeast','fontSize', 20);
%     pbaspect([1 1 1])  
% 
%     % -- 100km/h
%     ax(6) = subplot(236);
%     plot(time_step,Omega(:,3),'-b','LineWidth',2,'displayName','$\Omega$')
%     hold on
%     plot(time_step,delta_D_r(:,3),'-r','LineWidth',2,'displayName','$\delta_{D}$')
%     hold off
%     title('100 km\h');
%     grid on
%     xlabel('Time [s]')
%     ylabel('rad/s')
%     legend('location','northeast','fontSize', 20);
%     pbaspect([1 1 1]) 
%     ax(6).YAxis.Exponent=-1;
%     set([ax(4),ax(5),ax(6)],'XTick',0:200:1000)
%     set(ax,'FontSize',15)
%     linkaxes([ax(4),ax(5),ax(6)],'xy')    
% 
%     set(ax(1),'position',[.0 .55 .35 .35])
%     set(ax(2),'position',[.2 .55 .35 .35])
%     set(ax(3),'position',[0.4 .55 .35 .35])
%     set(ax(4),'position',[.0 .1 .35 .35])
%     set(ax(5),'position',[.2 .1 .35 .35])
%     set(ax(6),'position',[.4 .1 .35 .35])
%     exportgraphics(gcf,sprintf(output_file,q,q,'c'),'ContentType','vector')
%     clear ax
%     
    % ----------------
    %% print kus results out
    % ----------------
    format shortG
    m = zeros(length(handling_datas),2);
    for ind = 1:length(handling_datas)
        fprintf("run# %d coefficients", ind);
        disp(handling_datas{ind}.coeffs);
        m(ind,:) = [handling_datas{ind}.speed*3.6 handling_datas{ind}.kus];
    end
    writematrix(m,'results/kus.csv') 
end
    
