clc
clearvars 
close all   
% Set LaTeX as default interpreter for axis labels, ticks and legends
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

set(0,'DefaultFigureWindowStyle','docked');
set(0,'defaultAxesFontSize',20)
set(0,'DefaultLegendFontSize',20)

load('20130222_01_02_03_grandsport.mat');
%% fig1
figure(1);

plot(insData.vxCG.time,insData.vxCG.value,'-b','markerSize',5,'lineWidth',2,'DisplayName','longitudinal [$u$]');
hold on
plot(insData.vyCG.time,insData.vyCG.value,'-r','lineWidth',2,'DisplayName','lateral [$v]$');
legend('Location','northeast','FontSize',20);
xlabel('Time (s)')
ylabel('Veclocity $(^{m}/_{s})$');
pbaspect([1 1 1]);
% xlim([0 700]);
grid on

hold off
print -depsc graphs/ex-31.eps
%% fig2

trial_RL = tireData.wheelTicksRL.value - tireData.wheelTicksRL.value(1);
trial_RL(trial_RL ~= 0) = 1;
trial_RL = [0;diff(trial_RL)];
trial_RL(trial_RL == -1) = 0;
non_zero_tick_RL = trial_RL .* tireData.wheelTicksRL.time;
non_zero_tick_RL = non_zero_tick_RL( non_zero_tick_RL ~= 0 );
non_zero_tick_RL = [non_zero_tick_RL,[0;diff(non_zero_tick_RL)]];
calc = params.rollingCircumferenceFL.value ./ non_zero_tick_RL(:,2);
non_zero_tick_RL = [non_zero_tick_RL, calc];

trial = tireData.wheelTicksRR.value - tireData.wheelTicksRR.value(1);
trial(trial ~= 0) = 1;
trial = [0;diff(trial)];
trial(trial == -1) = 0;
non_zero_tick = trial .* tireData.wheelTicksRR.time;
non_zero_tick = non_zero_tick( non_zero_tick ~= 0 );
non_zero_tick = [non_zero_tick,[0;diff(non_zero_tick)]];
calc = params.rollingCircumferenceFL.value ./ non_zero_tick(:,2);
non_zero_tick = [non_zero_tick, calc];

figure(2);

plot(tireData.wheelTicksRL.time,tireData.wheelTicksRL.value,'-b','markerSize',5,'lineWidth',2,'DisplayName','longitudinal velocity');
xlabel('Time (s)');
ylabel('$v_{x}$ ($^{m}/_{s}$)');
pbaspect([1 1 1]);
xlim([0 30]);
% ylim([-2 6]);
grid on

% print -depsc ex-32.eps
%% fig3

figure(3);

plot(insData.vxCG.time,insData.vxCG.value,'-b','markerSize',5,'lineWidth',2,'DisplayName','INS');
hold on
plot(non_zero_tick_RL(:,1),non_zero_tick_RL(:,3),'-r','markerSize',5,'lineWidth',2,'DisplayName','RL');
plot(non_zero_tick(:,1),non_zero_tick(:,3),'-g','markerSize',5,'lineWidth',2,'DisplayName','RR');
legend('Location','northeast','FontSize',20);
xlabel('Time (s)')
ylabel('longitudinal [$v_{x}$] ($^{m}/_{s}$)');
grid on
pbaspect([1 1 1]);

hold off
print -depsc graphs/ex-32.eps
%% fig 4
figure(4);

plot(insData.yawRate.time,deg2rad(insData.yawRate.value),'-b','markerSize',5,'lineWidth',4,'DisplayName','yaw rate');
hold on
plot(insData.yawRate.time,deg2rad(insData.yawRate.value),'-g','markerSize',5,'lineWidth',1,'DisplayName','yaw rate [filter]');
legend('Location','northeast','FontSize',20);
xlabel('Time (s)')
ylabel('$\Omega$ ($^{rad}/_{s}$)');
pbaspect([1 1 1]);

grid on

hold off
print -depsc graphs/ex-33.eps
%% fig5
figure(5);

laterial_acc = deg2rad(insData.yawRate.value).* insData.vxCG.value;
laterial_acc_filtered = deg2rad(insData.yawRateFilt.value).* insData.vxCG.value;

plot(insData.yawRate.time,laterial_acc,'-g','markerSize',5,'lineWidth',3,'DisplayName','INS [cal]');
hold on
plot(insData.yawRate.time,insData.ayCGFilt.value,'-b','markerSize',5,'lineWidth',2,'DisplayName','INS [filter]');

legend('Location','northeast','FontSize',20);
xlabel('Time (s)')
ylabel('laterial [$a_{y}$] ($^{m}/_{s^{2}}$)');
grid on
pbaspect([1 1 1]);
hold off
print -depsc graphs/ex-34.eps

%% fig6

acc_calc = gradient(non_zero_tick(:,3)) ./non_zero_tick(:,2);
acc_calc_mean = movmean(acc_calc,6);
acc_calc_RL = gradient(non_zero_tick_RL(:,3)) ./non_zero_tick_RL(:,2);

% acc_calc = [0;diff(non_zero_tick(:,3))];
% acc_calc_RL = [0;diff(non_zero_tick_RL(:,3))];

figure(6);
plot(insData.axCG.time,insData.axCG.value,'-m','lineWidth',2,'displayName','INS');
hold on
% plot(non_zero_tick_RL(:,1),acc_calc_RL,'lineWidth',1);
plot(non_zero_tick(:,1),acc_calc_mean,'-b','lineWidth',2,'displayName','RR hall sensor [cal]');
legend('Location','southeast','FontSize',25);
xlabel('Time (s)')
ylabel('logitudinal [$a_{x}$] ($^{m}/_{s^{2}}$)');
grid on
pbaspect([1 1 1]);

slip_ang = 50 * atan(insData.vyCG.value ./  insData.vxCG.value);

hold off
print -depsc graphs/ex-35.eps
%% fig7
figure(7);
plot(insData.vxCG.time,slip_ang,'-b','lineWidth',4, 'displayName','side slip [cal]');
hold on
plot(insData.sideSlip.time,insData.sideSlip.value,'-g','lineWidth',2,'displayName','INS side slip');
legend('Location','northwest','FontSize',25);
xlabel('Time (s)')
ylabel('$\beta$ (deg)');
ylim ([-4 6]);
% xlim([0 700]);
grid on
pbaspect([1 1 1]);
hold off
print -depsc graphs/ex-36.eps