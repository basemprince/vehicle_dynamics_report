% ----------------------------
%% For total error graphing
% ----------------------------

clear all;
load('./results/pure_purs');
load('./results/stanly_dynamic');
load('./results/stanly_kenimatic');
load('./results/clothid');

output_file = './graphs/q%d/ex-6%d%s.eps';
q = 1;      

x = clothid{1}.time_sim;
time_sim = length(clothid{1}.time_sim);
test_count = length(clothid);
delta_30 = zeros(time_sim,4);
delta_70 = zeros(time_sim,4);

delta_30(:,1) = clothid{1}.delta_fr;
delta_30(:,2) = pure{1}.delta_fr;
delta_30(:,3) = s_kinematic{1}.delta_fr;
delta_30(:,4) = s_dynamic{1}.delta_fr;
delta_70(:,1) = clothid{2}.delta_fr;
delta_70(:,2) = pure{2}.delta_fr;
delta_70(:,3) = s_kinematic{2}.delta_fr;
delta_70(:,4) = s_dynamic{2}.delta_fr;

color = '-b';
color2 = '-m';
title_size = 21;

%% for speed 30
figure('Name','steering behaviour-30','NumberTitle','off'), clf   
% clothid
ax(1) = subplot(221);
plot(x,delta_30(:,1),color,'LineWidth',2)
title("Clothoid-Based",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
grid on
pbaspect([1 1 1]);
% pure 
ax(3) = subplot(222);
plot(x,delta_30(:,2),color,'LineWidth',2)
title("Pure Pursuit",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
grid on
% ax(3).YAxis.Exponent=1;
pbaspect([1 1 1]);
% s_kinematic
ax(2) = subplot(223);
plot(x,delta_30(:,3),color,'LineWidth',2)
title("Stanley Kinematic",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
ylim([-7 7])
grid on
pbaspect([1 1 1]);
% s_dynamic
ax(4) = subplot(224);
plot(x,delta_30(:,4),color,'LineWidth',2)
title("Stanley Dynamic",'fontweight','bold','FontSize', title_size)
grid on
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
ylim([-7 7])
pbaspect([1 1 1]);
set(ax(1),'position',[.0 .6 .35 .35])
set(ax(2),'position',[.0 .1 .35 .35])
set(ax(3),'position',[.2 .6 .35 .35])
set(ax(4),'position',[.2 .1 .35 .35])


%% for speed 70
figure('Name','steering behaviour-70','NumberTitle','off'), clf   
% clothid
ax(1) = subplot(221);
plot(x,delta_70(:,1),color,'LineWidth',2)
title("Clothoid-Based",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
grid on
pbaspect([1 1 1]);
% pure 
ax(3) = subplot(222);
plot(x,delta_70(:,2),color,'LineWidth',2)
title("Pure Pursuit",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
grid on
% ax(3).YAxis.Exponent=1;
pbaspect([1 1 1]);
% s_kinematic
ax(2) = subplot(223);
plot(x,delta_70(:,3),color,'LineWidth',2)
title("Stanley Kinematic",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
ylim([-7 7])
grid on
pbaspect([1 1 1]);
% s_dynamic
ax(4) = subplot(224);
plot(x,delta_70(:,4),color,'LineWidth',2)
title("Stanley Dynamic",'fontweight','bold','FontSize', title_size)
grid on
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
ylim([-7 7])
pbaspect([1 1 1]);
set(ax(1),'position',[.0 .6 .35 .35])
set(ax(2),'position',[.0 .1 .35 .35])
set(ax(3),'position',[.2 .6 .35 .35])
set(ax(4),'position',[.2 .1 .35 .35])

%% combined 

figure('Name','steering behaviour-all','NumberTitle','off'), clf   
% clothid
ax(1) = subplot(241);
plot(x,delta_30(:,1),color,'LineWidth',2)
title("Clothoid-Based - 30",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
grid on
pbaspect([1 1 1]);
% pure 
ax(2) = subplot(242);
plot(x,delta_30(:,2),color,'LineWidth',2)
title("Pure Pursuit - 30",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
grid on
% ax(3).YAxis.Exponent=1;
pbaspect([1 1 1]);
% s_kinematic
ax(3) = subplot(243);
plot(x,delta_30(:,3),color,'LineWidth',2)
title("Stanley Kinematic - 30",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
ylim([-7 7])
grid on
pbaspect([1 1 1]);
% s_dynamic
ax(4) = subplot(244);
plot(x,delta_30(:,4),color,'LineWidth',2)
title("Stanley Dynamic - 30",'fontweight','bold','FontSize', title_size)
grid on
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
ylim([-7 7])
pbaspect([1 1 1]);

% clothid
ax(5) = subplot(245);
plot(x,delta_70(:,1),color2,'LineWidth',2)
title("Clothoid-Based - 70",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
grid on
pbaspect([1 1 1]);
% pure 
ax(6) = subplot(246);
plot(x,delta_70(:,2),color2,'LineWidth',2)
title("Pure Pursuit - 70",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
grid on
% ax(3).YAxis.Exponent=1;
pbaspect([1 1 1]);
% s_kinematic
ax(7) = subplot(247);
plot(x,delta_70(:,3),color2,'LineWidth',2)
title("Stanley Kinematic - 70",'fontweight','bold','FontSize', title_size)
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
ylim([-7 7])
grid on
pbaspect([1 1 1]);
% s_dynamic
ax(8) = subplot(248);
plot(x,delta_70(:,4),color2,'LineWidth',2)
title("Stanley Dynamic - 70",'fontweight','bold','FontSize', title_size)
grid on
ylabel('$\delta_0$ [deg]')
xlabel('Time (s)')
ylim([-7 7])
pbaspect([1 1 1]);
linkaxes(ax,'xy')
exportgraphics(gcf,sprintf(output_file,q,q,'ff'),'ContentType','vector')
%% error tracking

controlers = {'Clothoid-Based','Pure Pursuit','Stanley Kinematic','Stanley Dynamic'};
error = zeros(test_count,5);
error(:,1) = [30, 70];
for ind =1: test_count
error(ind,2) = clothid{ind}.e_max;
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