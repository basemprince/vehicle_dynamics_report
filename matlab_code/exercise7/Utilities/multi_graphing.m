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
    data_central = struct();
    
%     [time_sim, u,v,psi,Omega,delta,delta_D,x_CoM,y_CoM] = deal(zeros(length(model_sims{1}.states.u.time),length(model_sims)));
    for ind = 1:length(model_sims)
        data_central.time_sim{ind} = model_sims{ind}.states.u.time;
        data_central.u{ind}       = model_sims{ind}.states.u.data;
        data_central.v{ind} = model_sims{ind}.states.v.data;
        data_central.speed_req{ind} = squeeze(model_sims{ind}.req_speed.data);
        
        data_central.psi{ind} = model_sims{ind}.states.psi.data;
        data_central.Omega{ind} = squeeze(model_sims{ind}.states.Omega.data);
        data_central.delta{ind} = model_sims{ind}.states.delta.data;
        data_central.delta_D{ind} = model_sims{ind}.inputs.delta_D.data;
        data_central.x_CoM{ind} = model_sims{ind}.states.x.data;
        data_central.y_CoM{ind} = model_sims{ind}.states.y.data;  
        data_central.desired_steer_atWheel{ind} = data_central.delta_D{ind}/tau_D;
        data_central.delta_D_r{ind} = deg2rad(data_central.delta_D{ind});
        % -----------------
        % Accelerations
        % -----------------
        % Derivatives of u, v [m/s^2]
        data_central.dot_u{ind} = diff(data_central.u{ind})/Ts;
        data_central.dot_v{ind} = diff(data_central.v{ind})/Ts;
        % Total longitudinal and lateral accelerations
        data_central.Ax{ind} = data_central.dot_u{ind}(1:end,:) - data_central.Omega{ind}(2:end,:).*data_central.v{ind}(2:end,:);
        data_central.Ay{ind} = data_central.dot_v{ind}(1:end,:) + data_central.Omega{ind}(2:end,:).*data_central.u{ind}(2:end,:);
        data_central.Ay{ind} = [data_central.Ay{ind}(1,:);data_central.Ay{ind}];
    end
    
    % ----------------
    %% For plot saving
    % ----------------
    output_file = 'graphs/q%d/ex-7%d%s.eps';
    output_files = 'graphs/q%d/ex-7%d%s%d.eps';
    q = 1;
    speeds = {};
    deltas_fig = {};
    for ind = 1:length(model_sims)
        speeds{ind} = append(string(error_data{ind}.speed), ' km/hr');
    end
    colors = {'-b','-r','-g','-m','-k','-k','-y'};   
    
    % -------------------
    %% Plot vehicle paths
    % -------------------
    vehRoute = ClothoidList();
    numOfClothoids_route = size(refRoute_fewPoints,1);
    for i = 1:numOfClothoids_route-1
        vehRoute.push_back_G1(refRoute_fewPoints(i,1),refRoute_fewPoints(i,2),refRoute_fewPoints(i,3), refRoute_fewPoints(i+1,1),refRoute_fewPoints(i+1,2),refRoute_fewPoints(i+1,3)); 
    end
    [x_route,y_route] = vehRoute.evaluate(0:0.1:vehRoute.length);
    figure('Name','Real Vehicle Paths','NumberTitle','off'), clf       
    for ind = 1: length(model_sims)
        N = length(data_central.time_sim{ind});
        set(gca,'fontsize',16)
        ax(ind) = subplot(1,3,ind);
        title(speeds{ind})  %Real Vehicle Path
        % Plot road scenario
    %     plot(costmap,'Inflation','off')
        hold on
        for ii=1:size(mapObjects,1)
            rectangle('Position',mapObjects(ii,:),'FaceColor',color('purple'),'EdgeColor','k','LineWidth',2)
        end
        % Plot interpolated reference route 

        plot(refRoute_fewPoints(:,1),refRoute_fewPoints(:,2),'go','MarkerFaceColor','g','MarkerSize',8) %,'DisplayName','Route'

        % Plot original (not interpolated) reference route 
        plot(refRoute_points_orig(:,1),refRoute_points_orig(:,2),'o','Color',color('orange'),'MarkerFaceColor',color('orange'),'MarkerSize',8) %,'DisplayName','Interpolated Route'
        % Plot vehicle CoM trajectory
        plot(data_central.x_CoM{ind},data_central.y_CoM{ind},'Color',color('gold'),'LineWidth',4)
        vehRoute.plot;
        for i = 1:floor(N/20):N
            rot_mat = [cos(data_central.psi{ind}(i)) -sin(data_central.psi{ind}(i)) ; sin(data_central.psi{ind}(i)) cos(data_central.psi{ind}(i))];
            pos_rr = rot_mat*[-Lr -Wr/2]';
            pos_rl = rot_mat*[-Lr +Wr/2]';
            pos_fr = rot_mat*[+Lf -Wf/2]';
            pos_fl = rot_mat*[+Lf +Wf/2]';
            pos = [pos_rr pos_rl pos_fl pos_fr];
            p = patch(data_central.x_CoM{ind}(i) + pos(1,:),data_central.y_CoM{ind}(i) + pos(2,:),'blue');
            quiver(data_central.x_CoM{ind}(i), data_central.y_CoM{ind}(i), data_central.u{ind}(i)*cos(data_central.psi{ind}(i)), data_central.u{ind}(i)*sin(data_central.psi{ind}(i)), 'color', [1,0,0]);
            quiver(data_central.x_CoM{ind}(i), data_central.y_CoM{ind}(i), - data_central.v{ind}(i)*sin(data_central.psi{ind}(i)), data_central.v{ind}(i)*cos(data_central.psi{ind}(i)), 'color', [0.23,0.37,0.17]);
        end
        plot(5,5,'mo','MarkerSize',6,'MarkerFaceColor','m')
        plot(196,134.5,'go','MarkerSize',6,'MarkerFaceColor','g')
        text(2,11,'A','FontSize',16);
        text(193,145,'B','FontSize',16);
%         hold off
        box on
        xlabel('x-coord [m]')
        ylabel('y-coord [m]')
        xlim([0 200])
        ylim([0 200])
        xticks([0:20:200])
        yticks([0:20:200])
        legend('interpolating points','route','vehicle path','clothoid fitting','clothoid fitting','location','NW','fontSize',11.5)
        set(gca,'color',[0.97 0.97 0.97])
        pbaspect([1 1 1])  
    end
    exportgraphics(gcf,sprintf(output_file,q,q,'vpt'),'ContentType','vector')   
%% speed
    figure('Name','speeds','NumberTitle','off'), clf  
   for ind = 1: length(model_sims)

        set(gca,'fontsize',16)
        ax(ind) = subplot(1,3,ind);

        plot(data_central.time_sim{ind},data_central.u{ind}*3.6,'-b','LineWidth',2)
        title(speeds{ind}) 
        hold on
        plot(data_central.time_sim{ind},data_central.speed_req{ind}*3.6,'--r','LineWidth',2)
        legend('real','desired','Location','SW')
        grid on
        ylabel('$u$ [km/h]')  
        xlabel('Time (s)')
        xlim([0 data_central.time_sim{ind}(end)])

        pbaspect([1 1 1])   
   end
    linkaxes(ax,'y')
    exportgraphics(gcf,sprintf(output_file,q,q,'sp'),'ContentType','vector')   