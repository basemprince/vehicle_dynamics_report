% ----------------------------
%% For total error graphing
% ----------------------------

clear all;
load('./results/pure_purs');
load('./results/s_dynamic');
load('./results/s_kinematic');
load('./results/clothoid');

output_file = './graphs/q%d/ex-7%d%s.eps';
q = 2;      

data_central = struct();
test_count = 2;

for ind = 1:test_count
    
data_central.c.x{ind} = clothoid{ind}.time_sim;
data_central.p.x{ind} = pure{ind}.time_sim;
data_central.k.x{ind} = s_kinematic{ind}.time_sim;
data_central.d.x{ind} = s_dynamic{ind}.time_sim;

data_central.c.delta_fr{ind} = clothoid{ind}.delta_fr;
data_central.p.delta_fr{ind} = pure{ind}.delta_fr;
data_central.k.delta_fr{ind} = s_kinematic{ind}.delta_fr;
data_central.d.delta_fr{ind} = s_dynamic{ind}.delta_fr;

end
color = '-b';
color2 = '-m';
title_size = 21;


%% combined 

figure('Name','steering behaviour-all','NumberTitle','off'), clf   
% clothoid
ax(1) = subplot(241);
plot(data_central.c.x{1},data_central.c.delta_fr{1},color,'LineWidth',2)
title("Clothoid-Based - 30",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
grid on
pbaspect([1 1 1]);
% pure 
ax(2) = subplot(242);
plot(data_central.p.x{1},data_central.p.delta_fr{1},color,'LineWidth',2)
title("Pure Pursuit - 30",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
grid on
% ax(3).YAxis.Exponent=1;
pbaspect([1 1 1]);
% s_kinematic
ax(3) = subplot(243);
plot(data_central.k.x{1},data_central.k.delta_fr{1},color,'LineWidth',2)
title("Stanley Kinematic - 30",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
ylim([-7 7])
grid on
pbaspect([1 1 1]);
% s_dynamic
ax(4) = subplot(244);
plot(data_central.d.x{1},data_central.d.delta_fr{1},color,'LineWidth',2)
title("Stanley Dynamic - 30",'fontweight','bold','FontSize', title_size)
grid on
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
ylim([-7 7])
pbaspect([1 1 1]);

% clothoid
ax(5) = subplot(245);
plot(data_central.c.x{2},data_central.c.delta_fr{2},color2,'LineWidth',2)
title("Clothoid-Based - 60",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
grid on
pbaspect([1 1 1]);
% pure 
ax(6) = subplot(246);
plot(data_central.p.x{2},data_central.p.delta_fr{2},color2,'LineWidth',2)
title("Pure Pursuit - 60",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
grid on
% ax(3).YAxis.Exponent=1;
pbaspect([1 1 1]);
% s_kinematic
ax(7) = subplot(247);
plot(data_central.k.x{2},data_central.k.delta_fr{2},color2,'LineWidth',2)
title("Stanley Kinematic - 60",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
ylim([-7 7])
grid on
pbaspect([1 1 1]);
% s_dynamic
ax(8) = subplot(248);
plot(data_central.d.x{2},data_central.d.delta_fr{2},color2,'LineWidth',2)
title("Stanley Dynamic - 60",'fontweight','bold','FontSize', title_size)
grid on
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
ylim([-7 7])
pbaspect([1 1 1]);
linkaxes(ax,'y')
exportgraphics(gcf,sprintf(output_file,q,q,'ff'),'ContentType','vector')
%% error tracking

controlers = {'Clothoid-Based','Pure Pursuit','Stanley Kinematic','Stanley Dynamic'};
error = zeros(test_count,5);
error(:,1) = [30, 60];
for ind =1: test_count
error(ind,2) = clothoid{ind}.e_max;
error(ind,3) = pure{ind}.e_max;
error(ind,4) = s_kinematic{ind}.e_max;
error(ind,5) = s_dynamic{ind}.e_max;
end
figure('Name','max error','NumberTitle','off'), clf  
set(0,'DefaultAxesColorOrder',parula(5))
bar(error(:,1),error(:,2:end));
grid on

xlabel('vehicle speed (u) [$km/h$]')
ylabel('tracking error [m]')
% yticks((0:2:14))
%     ylim([0 52])
% xlim([10 110])
set(gca,'fontsize',26)
hleg = legend(controlers,'location','NW');
htitle = get(hleg,'Title');
set(htitle,'String','Lateral Controller')

pbaspect([1 1 1])
exportgraphics(gcf,sprintf(output_file,q,q,'er'),'ContentType','vector')

%% times

for ind = 1: test_count
disp(data_central.c.x{ind}(end));
disp(data_central.p.x{ind}(end));
disp(data_central.k.x{ind}(end));
disp(data_central.d.x{ind}(end));
disp('----');
end

%% derivations
for ind = 1: test_count
disp(sum(abs(diff(data_central.c.delta_fr{ind}))));
disp(sum(abs(diff(data_central.p.delta_fr{ind}))));
disp(sum(abs(diff(data_central.k.delta_fr{ind}))));
disp(sum(abs(diff(data_central.d.delta_fr{ind}))));
disp('----');
end
%% max change
for ind = 1: test_count
disp(max(abs(diff(data_central.c.delta_fr{ind}))));
disp(max(abs(diff(data_central.p.delta_fr{ind}))));
disp(max(abs(diff(data_central.k.delta_fr{ind}))));
disp(max(abs(diff(data_central.d.delta_fr{ind}))));
disp('----');
end