% ----------------------------------------------------
%% Route Planner
% ----------------------------------------------------

% ----------------------
% initial and final conditions
% ----------------------
x0 = X0(1);       % [m]
y0 = X0(2);       % [m]
theta0 = X0(3);   % [rad]
startPose = [x0, y0, theta0]; 
goalPose  = loadFinalPose;

% ----------------------
% compute minimum turning radius
% ----------------------
L = vehicleDims.Wheelbase;
max_steer_angle = m_steer;  % [deg] max steering angle at the front wheel 
min_turn_radius = L/tan(deg2rad(max_steer_angle));

% ----------------------
% Solve the problem
% ----------------------
planner = pathPlannerRRT(costmap,'ConnectionDistance',c_distance,...
        'GoalBias',0.1,'MinIterations',min_itr,'MaxIterations',1e6,...
        'MinTurningRadius',min_turn_radius);
tic;
refPath = plan(planner,startPose,goalPose);
elapsed_time_routePlan = round(toc,1); 

% interpolate the calculated reference path
refRoute_points_orig = interpolate(refPath);
interp_sampling = interpolation_sample;
interp_vector_fewPoints = 0:interp_sampling:refPath.Length;
refPath_poses_fewPoints = interpolate(refPath,interp_vector_fewPoints);
disp(length(refPath_poses_fewPoints))
isPathValid = checkPathValidity(refPath,costmap);

% ----------------------
% Fit the interpolated path with clothoids
% ----------------------
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

% ----------------------------------------------------
%% Save the resulting path
% ----------------------------------------------------
save('./Scenario/refPath_poses_fewPoints','refRoute_fewPoints');
save('./Scenario/refRoute_poses_RRT','refRoute_points_orig');
save('./Scenario/cpuTime_routePlan','elapsed_time_routePlan');
save('./Scenario/refPath','refPath');
% ----------------------------------------------------
%% Plot the resulting path
% ----------------------------------------------------
enable_plotRoute = 1;
if (enable_plotRoute)
    figure('Name',comb,'NumberTitle','off'), clf
    hold on
    %plot(planner)
    plot(costmap)
    plot(refPath,'DisplayName','Planned Path')
%     scatter(refRoute_points_orig(:,1),refRoute_points_orig(:,2),[],'filled', ...
%         'DisplayName','Transition Points')
%     scatter(refPath_poses_fewPoints(:,1),refPath_poses_fewPoints(:,2),'DisplayName','Interpolated Points')
    xlim([0 200])
    ylim([0 200])
    xlabel('x [m]')
    ylabel('y [m]')
    xticks([0:20:200])
    yticks([0:20:200])

    set(gca,'color',[0.9 0.9 0.9])
    legend('location','NW');
    title('Path RRT*')
    pbaspect([1 1 1])      
    box on;
    exportgraphics(gcf,sprintf(output_file,q,q,'r'),'ContentType','vector')
end

route_data = struct();
route_data.c_distance = c_distance;
route_data.min_itr = min_itr;
route_data.max_itr = max_itr;
route_data.m_steer = m_steer;
route_data.interpolation_sample = interpolation_sample;
route_data.costmap = costmap;
route_data.refPath = refPath;
route_data.refRoute_points_orig = refRoute_points_orig;
route_data.refRoute_fewPoints = refRoute_fewPoints;
route_data.refPath_poses_fewPoints = refPath_poses_fewPoints;
route_data.elapsed_time_routePlan = elapsed_time_routePlan;
