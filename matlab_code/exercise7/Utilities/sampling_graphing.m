clear all

vehicle_data = getVehicleDataStruct();
Lf = vehicle_data.vehicle.Lf;  % [m] Distance between vehicle CoG and front wheels axle
Lr = vehicle_data.vehicle.Lr;  % [m] Distance between vehicle CoG and front wheels axle
L  = vehicle_data.vehicle.L;   % [m] Vehicle length
Wf = vehicle_data.vehicle.Wf;  % [m] Width of front wheels axle 
Wr = vehicle_data.vehicle.Wr;  % [m] Width of rear wheels axle                   
m  = vehicle_data.vehicle.m;   % [kg] Vehicle Mass
g  = vehicle_data.vehicle.g;   % [m/s^2] Gravitational acceleration
tau_D = vehicle_data.steering_system.tau_D;  % [-] steering system ratio (pinion-rack)


output_file = 'graphs/q%d/ex-7%d%s.eps';
output_files = 'graphs/q%d/ex-7%d%s-%d.eps';
q = 1;   

load('results/samp_test_10.mat');

figure('Name','camp_effect','NumberTitle','off'), clf


S = samp_test10{1};

c =0;
for i = 10:20:50
    c= c+1;
    interp_sampling = i;
    interp_vector_fewPoints = 0:interp_sampling:S.refPath.Length;
    refPath_poses_fewPoints = interpolate(S.refPath,interp_vector_fewPoints);
    path_fewPoints = ClothoidList();
    numOfClothoids_fewPoints = size(refPath_poses_fewPoints,1);
    for jj = 1:numOfClothoids_fewPoints-1
        try
            path_fewPoints.push_back_G1(refPath_poses_fewPoints(jj,1),refPath_poses_fewPoints(jj,2),deg2rad(refPath_poses_fewPoints(jj,3)), refPath_poses_fewPoints(jj+1,1),refPath_poses_fewPoints(jj+1,2),deg2rad(refPath_poses_fewPoints(jj+1,3))); 
        catch
        warning('Problem using function.  Assigning a value of 0.');
        end
    end
    % Compute the local curvature for the reference route
    interp_vector_fewPoints = [interp_vector_fewPoints, path_fewPoints.length];    % add also the final route point
    [x_cloth_refPath_fewPoints,y_cloth_refPath_fewPoints,theta_cloth_refPath_fewPoints,curv_refPath_fewPoints] = path_fewPoints.evaluate(interp_vector_fewPoints);
    refRoute_fewPoints = [x_cloth_refPath_fewPoints',y_cloth_refPath_fewPoints',theta_cloth_refPath_fewPoints',curv_refPath_fewPoints'];

    ax(c) = subplot(1,3,c);
    fn = sprintf('samp_test_%d',i);
    fn1 = sprintf('samp_test%d',i);
    hold on
    %plot(planner)
    plot(S.costmap)
%     plot(S.refPath,'DisplayName','Planned Path')
%         scatter(S.refRoute_points_orig(:,1),S.refRoute_points_orig(:,2),[],'filled', ...
%             'DisplayName','Transition Points')
%         scatter(refPath_poses_fewPoints(:,1),refPath_poses_fewPoints(:,2),'MarkerFaceColor','r','DisplayName','Interpolated Points')
        scatter(refRoute_fewPoints(:,1),refRoute_fewPoints(:,2),'MarkerFaceColor','r','DisplayName','Interpolated Points')
    xlim([0 200])
    ylim([0 200])
    xlabel('x [m]')
    ylabel('y [m]')
%     xticks([0:20:200])
%     yticks([0:20:200])

    set(gca,'color',[0.9 0.9 0.9])
    legend('location','NW','fontSize',11);
    title(append('sampling - ', string(i)),'Interpreter','tex')
    pbaspect([1 1 1])      
    box on;
end
% set(ax(1),'position',[.0 .60 .35 .35])
% set(ax(3),'position',[.0 .1 .35 .35])
% set(ax(2),'position',[.20 .60 .35 .35])
% set(ax(4),'position',[.20 .1 .35 .35])
exportgraphics(gcf,sprintf(output_file,q,q,'samp1'),'ContentType','vector')

%% sampling vs vertix count
iters = 150;
vert_count = zeros(iters,2);
for j=1:iters
    interp_sampling = j;
    interp_vector_fewPoints = 0:interp_sampling:S.refPath.Length;
    refPath_poses_fewPoints = interpolate(S.refPath,interp_vector_fewPoints);
    vert_count(j,:) = [j length(refPath_poses_fewPoints)];
end
figure('Name','sample count path length','NumberTitle','off'), clf
plot(vert_count(:,2),'b','LineWidth',2);
xlabel('interpolation samples') 
ylabel('path vertix count') 
% xlim([0 150])
grid on
pbaspect ([1 1 1]);
exportgraphics(gcf,sprintf(output_file,q,q,'vrtc'),'ContentType','vector')

%% tracking
F.samp_test_10 = load('results/samp_test_10.mat');
F.samp_test_30 = load('results/samp_test_30.mat');
F.samp_test_50 = load('results/samp_test_50.mat');
F.samp_test_70 = load('results/samp_test_70.mat');
F.samp_test_90 = load('results/samp_test_90.mat');
F.samp_test_100 = load('results/samp_test_110.mat');

scenario = load('./Scenario/scenario');
mapObjects = scenario.scenario_data.mapObjects;
vehicleDims = scenario.scenario_data.vehicleDims;
clear ax;

figure('Name','tracking','NumberTitle','off'), clf
c =0;
for i = [30 70 90]
    c= c+1;

    fn = sprintf('samp_test_%d',i);
    fn1 = sprintf('samp_test%d',i);

    time_sim =   F.(fn).(fn1){2}{1}.states.u.time;
    x_CoM      = F.(fn).(fn1){2}{1}.states.x.data;
    y_CoM      = F.(fn).(fn1){2}{1}.states.y.data;
    psi        = F.(fn).(fn1){2}{1}.states.psi.data;
    u        = F.(fn).(fn1){2}{1}.states.u.data;
    v        = F.(fn).(fn1){2}{1}.states.v.data;

    N = length(time_sim);
    vehRoute = ClothoidList();
    numOfClothoids_route = size(F.(fn).(fn1){1}.refRoute_fewPoints,1);
    for j = 1:numOfClothoids_route-1
        vehRoute.push_back_G1(F.(fn).(fn1){1}.refRoute_fewPoints(j,1),...
            F.(fn).(fn1){1}.refRoute_fewPoints(j,2),...
            F.(fn).(fn1){1}.refRoute_fewPoints(j,3),...
            F.(fn).(fn1){1}.refRoute_fewPoints(j+1,1),...
            F.(fn).(fn1){1}.refRoute_fewPoints(j+1,2),...
            F.(fn).(fn1){1}.refRoute_fewPoints(j+1,3)); 
    end
    [x_route,y_route] = vehRoute.evaluate(0:0.1:vehRoute.length);

    ax(c) = subplot(1,3,c);
    title(append('sampling - ', string(i),' [m]'),'Interpreter','tex')
    hold on
    for ii=1:size(mapObjects,1)
        rectangle('Position',mapObjects(ii,:),'FaceColor',color('purple'),'EdgeColor','k','LineWidth',2)

    end
    % Plot interpolated reference route 

    plot(F.(fn).(fn1){1}.refPath_poses_fewPoints(:,1),F.(fn).(fn1){1}.refPath_poses_fewPoints(:,2),'go','MarkerFaceColor','g','MarkerSize',8) %,'DisplayName','Route'

    % Plot original (not interpolated) reference route 
    plot(F.(fn).(fn1){1}.refRoute_points_orig(:,1),F.(fn).(fn1){1}.refRoute_points_orig(:,2),'o','Color',color('orange'),'MarkerFaceColor',color('orange'),'MarkerSize',8) %,'DisplayName','Interpolated Route'
    % Plot vehicle CoM trajectory
    plot(x_CoM,y_CoM,'Color',color('gold'),'LineWidth',4)
    vehRoute.plot;
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
    plot(5,5,'mo','MarkerSize',6,'MarkerFaceColor','m')
    plot(196,134.5,'go','MarkerSize',6,'MarkerFaceColor','g')
    text(2,11,'A','FontSize',16);
    text(193,145,'B','FontSize',16);
    hold off
    box on
    xlabel('x [m]')
    ylabel('y [m]')
    xlim([0 200])
    ylim([0 200])
%     xticks([0:20:200])
%     yticks([0:20:200])
    legend('interp. pnts','route','path','location','NW','fontSize',18)
    set(gca,'color',[0.97 0.97 0.97])
    pbaspect([1 1 1])  
end
% set(ax(1),'position',[.0 .60 .35 .35])
% set(ax(3),'position',[.0 .1 .35 .35])
% set(ax(2),'position',[.20 .60 .35 .35])
% set(ax(4),'position',[.20 .1 .35 .35])

exportgraphics(gcf,sprintf(output_file,q,q,'samp2'),'ContentType','vector')  

% ----------
%% Bar graph
% ----------
figure('Name','Tracking error','NumberTitle','off'); clf;
c =0;
errors = zeros(3,2);
for i = 10:20:90
    c= c+1;
    errors(c,1)= i;
    fn = sprintf('samp_test_%d',i);
    fn1 = sprintf('samp_test%d',i);
    errors(c,2) = F.(fn).(fn1){3}{1}.e_max;
end

bar(errors(:,1),errors(:,2:end));
grid on
xlabel('sampling distance')
ylabel('tracking error [m]')
% yticks((0:2:14))
ylim([0 1.1])
% xlim([10 110])
set(gca,'fontsize',26)
% hleg = legend(string(la_to_plot),'location','NW');
% htitle = get(hleg,'Title');
% set(htitle,'String',leg_name)

pbaspect([1 1 1])
exportgraphics(gcf,sprintf(output_file,q,q,'samp3'),'ContentType','vector')

