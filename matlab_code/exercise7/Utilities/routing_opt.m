clear all
set(0,'DefaultAxesColorOrder',lines(4))
c_distances = [20, 30, 35, 40, 45, 50, 55, 60 , 70];
min_itrs = [1e5 1e4 1e3 1e2];
max_itrs = [1e6 1e5 1e4 1e3];

output_file = 'graphs/q%d/ex-7%d%s.eps';
output_files = 'graphs/q%d/ex-7%d%s-%d.eps';
q = 1;   

load('results/c_dis_itr.mat');

elapsed_time_table = zeros(length(c_distances),length(min_itrs)+1);
route_len_table = zeros(length(c_distances),length(min_itrs)+1);
for k =1:length(c_distances)
    elapsed_time_table(k+1,1) = c_distances(k);
    route_len_table(k+1,1) = c_distances(k);
    for j =1:length(min_itrs)
        elapsed_time_table(1,j+1) = min_itrs(length(min_itrs)+1-j);
        route_len_table(1,j+1) = min_itrs(length(min_itrs)+1-j);
        for i = 1:length(route_results)
            if route_results{i}.c_distance == c_distances(k)
                if route_results{i}.min_itr == min_itrs(length(min_itrs)+1-j)
                    elapsed_time_table(k+1,j+1) = route_results{i}.elapsed_time_routePlan;
                    route_len_table(k+1,j+1) = route_results{i}.refPath.Length;
                end
            end
        end
    end
end
%% connection dist effect
figure('Name','connecition dist','NumberTitle','off'), clf

c =0;
dist_to_plt = [30 50 70];
itrs_to_plt = [1e5];
for i =  1:length(route_results)
    curr_c  = route_results{i}.c_distance;
    curr_itr  = route_results{i}.min_itr;
    if ismember(curr_c,dist_to_plt) && ismember(curr_itr,itrs_to_plt)
        c = c+1;
        ax(c) = subplot(1,length(dist_to_plt),c);
        hold on
        %plot(planner)
        plot(route_results{i}.costmap)
        plot(route_results{i}.refPath,'color','b','DisplayName','Planned Path')
%         scatter(route_results{i}.refRoute_points_orig(:,1),route_results{i}.refRoute_points_orig(:,2),[],'filled', ...
%             'DisplayName','Transition Points')
%         scatter(route_results{i}.refPath_poses_fewPoints(:,1),route_results{i}.refPath_poses_fewPoints(:,2),'DisplayName','Interpolated Points')
        xlim([0 200])
        ylim([0 200])
        xlabel('x [m]')
        ylabel('y [m]')
    %     xticks([0:20:200])
    %     yticks([0:20:200])

        set(gca,'color',[0.9 0.9 0.9])
        legend('location','NW','fontSize',11);
        title(append('connection distance - ', string(curr_c)),'Interpreter','tex')
        pbaspect([1 1 1])      
        box on;
    end
end
% set(ax(1),'position',[.0 .60 .35 .35])
% set(ax(3),'position',[.0 .1 .35 .35])
% set(ax(2),'position',[.20 .60 .35 .35])
% set(ax(4),'position',[.20 .1 .35 .35])
exportgraphics(gcf,sprintf(output_file,q,q,'cds'),'ContentType','vector')

%% itrs effect
figure('Name','iter effect','NumberTitle','off'), clf

c =0;
dist_to_plt = [60];
itrs_to_plt = [1e3 1e4 1e5];
for i =  length(route_results):-1:1
    curr_c  = route_results{i}.c_distance;
    curr_itr  = route_results{i}.min_itr;
    if ismember(curr_c,dist_to_plt) && ismember(curr_itr,itrs_to_plt)
        c = c+1;
        ax(c) = subplot(length(dist_to_plt),length(itrs_to_plt),c);
        hold on
        %plot(planner)
        plot(route_results{i}.costmap)
        plot(route_results{i}.refPath,'color','b','DisplayName','Planned Path')
%         scatter(route_results{i}.refRoute_points_orig(:,1),route_results{i}.refRoute_points_orig(:,2),[],'filled', ...
%             'DisplayName','Transition Points')
%         scatter(route_results{i}.refPath_poses_fewPoints(:,1),route_results{i}.refPath_poses_fewPoints(:,2),'DisplayName','Interpolated Points')
        xlim([0 200])
        ylim([0 200])
        xlabel('x [m]')
        ylabel('y [m]')
    %     xticks([0:20:200])
    %     yticks([0:20:200])

        set(gca,'color',[0.9 0.9 0.9])
        legend('location','NW','fontSize',11);
        title(append('min/max itr - 1e', string(log10(route_results{i}.min_itr)), '/ 1e', string(log10(route_results{i}.max_itr))),'Interpreter','tex')
        pbaspect([1 1 1])      
        box on;
    end
end
% set(ax(1),'position',[.0 .60 .35 .35])
% set(ax(3),'position',[.0 .1 .35 .35])
% set(ax(2),'position',[.20 .60 .35 .35])
% set(ax(4),'position',[.20 .1 .35 .35])
exportgraphics(gcf,sprintf(output_file,q,q,'itr'),'ContentType','vector')

% ----------
%% Bar graph
% ----------
% elapsed time
figure('Name','elapsed time','NumberTitle','off'); clf;

% set(0,'DefaultAxesColorOrder',flipud(parula(length(min_itrs))))
dist = 3 ;
y_plt = elapsed_time_table(2:end,2:end);
y_plt(y_plt==0)=nan;
x = 1:dist:(1+(length(y_plt)-1)*dist) ;
bar(x,y_plt);
xticklabels(elapsed_time_table(2:end,1))
grid on
xlabel('connection distance [m]')
ylabel('elapsed time [s]')
% yticks((0:2:14))
ylim([0 30])
% set(gca, 'YScale', 'log')
xlim([x(1)-4 x(end)]+2)
set(gca,'fontsize',20)
hleg = legend(append('1e',string(log10(min_itrs)), ' / 1e',string(log10(min_itrs))),'location','NW');
htitle = get(hleg,'Title');
set(htitle,'String','min/max itrs')

pbaspect([1.2 1 1])
exportgraphics(gcf,sprintf(output_file,q,q,'eltm'),'ContentType','vector')


%% length of route
avg_dists = zeros(length(min_itrs),2);
for x=1:length(min_itrs)
    avg = sum(route_len_table(2:end,x+1)) ./ sum(route_len_table(2:end,x+1)~=0);
    avg_dists(x,:) = [min_itrs(length(min_itrs)+1-x) avg];
end

figure('Name','path distance','NumberTitle','off'); clf;
set(0,'DefaultAxesColorOrder',lines(4))
% set(0,'DefaultAxesColorOrder',flipud(parula(length(min_itrs))))
dist = 3 ;
x = 1:dist:(1+(length(avg_dists(:,1))-1)*dist) ;
bar(x,avg_dists(:,2));
x_tick_ = append('1e', string(log10(avg_dists(:,1))), '/ 1e', string(log10(avg_dists(:,1))+1));
xticklabels(x_tick_)
grid on
xlabel('min/max iterations')
ylabel('average path total distance [m]')
% yticks((0:2:14))
ylim([450 580])
xlim([x(1)-4 x(end)]+2)
set(gca,'fontsize',20)
% hleg = legend(append('1e',string(log10(min_itrs)), ' / 1e',string(log10(min_itrs))),'location','NW');
% htitle = get(hleg,'Title');
% set(htitle,'String','min/max itrs')

pbaspect([1.2 1 1])
exportgraphics(gcf,sprintf(output_file,q,q,'rl'),'ContentType','vector')