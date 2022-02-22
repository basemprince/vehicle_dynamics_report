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

plot(insData.vxCG.time,insData.vxCG.value,'lineWidth',2);
hold on
plot(insData.vyCG.time,insData.vyCG.value,'lineWidth',2);
legend({'logitudinal velocity','lateral velocity'},'Location','northeast','FontSize',30);
xlabel('Time(s)')
ylabel('Velocity (m/s)');
% xlim([0 700]);
grid on

hold off
print -depsc graphs/ex-31.eps
%% fig2

% FLDelta = [tireData.wheelTicksFL.value(1); diff(tireData.wheelTicksFL.value)];
% FLChangeCount = FLDelta;
% % v1ChangeCount( v1Delta ~= 0 ) = 1:nnz( v1Delta );
% rowNum = transpose(1: size(FLChangeCount));
% FLChangeCount = [FLChangeCount rowNum];
% FLChangeCount( FLDelta ~= 0 ) = 1;

% firstColumn = FLChangeCount(:,1);
% T_ind = FLChangeCount(firstColumn==1, :);
% T_ind = T_ind(:,2);
% T_FL = tireData.wheelTicksFL.time(T_ind);
% T_FL_diff = diff(T_FL);

trial_RL = tireData.wheelTicksRL.value - tireData.wheelTicksRL.value(1);
% trial_RL = [0;diff(tireData.wheelTicksRL.value)];
trial_RL(trial_RL ~= 0) = 1;
trial_RL = [0;diff(trial_RL)];
trial_RL(trial_RL == -1) = 0;
non_zero_tick_RL = trial_RL .* tireData.wheelTicksRL.time;
non_zero_tick_RL = non_zero_tick_RL( non_zero_tick_RL ~= 0 );
non_zero_tick_RL = [non_zero_tick_RL,[0;diff(non_zero_tick_RL)]];
calc = params.rollingCircumferenceFL.value ./ non_zero_tick_RL(:,2);
non_zero_tick_RL = [non_zero_tick_RL, calc];
% ind11 = non_zero_tick_RL(:,3)<100 & non_zero_tick_RL(:,2)< 1;
% non_zero_tick_RL = non_zero_tick_RL(ind11,:,:);

trial = tireData.wheelTicksRR.value - tireData.wheelTicksRR.value(1);
% trial = [0;diff(tireData.wheelTicksRR.value)];
trial(trial ~= 0) = 1;
trial = [0;diff(trial)];
trial(trial == -1) = 0;
non_zero_tick = trial .* tireData.wheelTicksRR.time;
non_zero_tick = non_zero_tick( non_zero_tick ~= 0 );
non_zero_tick = [non_zero_tick,[0;diff(non_zero_tick)]];
calc = params.rollingCircumferenceFL.value ./ non_zero_tick(:,2);
non_zero_tick = [non_zero_tick, calc];
% ind11 = non_zero_tick(:,3)<100 & non_zero_tick(:,2)< 1;
% non_zero_tick = non_zero_tick(ind11,:,:);

% last_num = 0;
% prev_result = 0;
% result = 0;
% size_ = size(tireData.wheelTicksFL.time,1);
% FLvel = zeros(size_,1);

% for i = 1: size(non_zero_tick)
%     ind = round((non_zero_tick(i,1)*1000)-1,0);
%     disp(ind);
%     FLvel(ind) = params.rollingCircumferenceFL.value / non_zero_tick(i,2);
% end

% for i = 1:size(tireData.wheelTicksFL.time)
%     try
%         ind = round((i /1000)+1);
%         ind2 = find(non_zero_tick(:,1) == ind);
%         last_num = non_zero_tick(ind2,2);
%         result = params.rollingCircumferenceFL.value / (last_num);
% %     disp(last_num);
%     catch
%         if result == inf
%             result = prev_result;
%         end
%     end
%     FLvel(i) = result;
%     prev_result = result;
% end

% T_trial = tireData.wheelTicksFL.time(non_zero_tick);
% time_diff = [0;diff(tireData.wheelTicksFL.time)];
% product = trial .* time_diff;

% FLvel = 2 * pi * params.rollingCircumferenceFL.value ./ product;
% FLvel(FLvel == inf) = 0;

% rps = [];
% index = 1;
% sum = 0;

% for i= 1:size(FLDelta)
%     if index <= 100
%         sum = sum + FLChangeCount(i);
%         index = index +1;
%     else
%         index = 1;
%         rps = [rps;sum];
%         sum = 0;
%     end
% end
% 
% FLvel = 2 * pi * params.rollingCircumferenceFL.value * rps;


figure(2);

plot(tireData.wheelTicksRL.time,tireData.wheelTicksRL.value,'lineWidth',2);
% plot(tireData.wheelTicksFR.time,tireData.wheelTicksFR.value,'.','markerSize',2);
% plot(tireData.wheelTicksRL.time,tireData.wheelTicksRL.value,'.','markerSize',2);
% plot(tireData.wheelTicksRR.time,tireData.wheelTicksRR.value,'.','markerSize',2);
% plot(insData.vxCG.time,insData.vxCG.value,'lineWidth',2);
legend('logitudinal velocity');
xlabel('Time(s)')
ylabel('longitudinal velocity (m/s)');
xlim([0 30]);
% ylim([-2 6]);
grid on

% print -depsc ex-32.eps
%% fig3

% 
% figure(3);
% 
% plot(tireData.wheelTicksFL.time,FLvel,'lineWidth',2);
% % plot(tireData.wheelTicksFR.time,tireData.wheelTicksFR.value,'.','markerSize',2);
% % plot(tireData.wheelTicksRL.time,tireData.wheelTicksRL.value,'.','markerSize',2);
% % plot(tireData.wheelTicksRR.time,tireData.wheelTicksRR.value,'.','markerSize',2);
% % plot(insData.vxCG.time,insData.vxCG.value,'lineWidth',2);
% legend('logitudinal velocity');
% xlabel('Time(s)')
% ylabel('longitudinal velocity (m/s)');
% % ylim([-2 6]);
% grid on

figure(3);

plot(insData.vxCG.time,insData.vxCG.value,'lineWidth',5);
hold on
plot(non_zero_tick_RL(:,1),non_zero_tick_RL(:,3),'lineWidth',5);
plot(non_zero_tick(:,1),non_zero_tick(:,3),'lineWidth',5);
legend('INS','RL','RR');
xlabel('Time(s)')
ylabel('longitudinal speed (m/s)');
grid on

hold off
print -depsc graphs/ex-32.eps
%% fig 4
figure(4);

plot(insData.yawRate.time,deg2rad(insData.yawRate.value),'lineWidth',3);
hold on
plot(insData.yawRate.time,deg2rad(insData.yawRate.value),'lineWidth',3);
legend('yaw rate','yaw rate filtered');
xlabel('Time(s)')
ylabel('$\Omega$ (rad/s)');
grid on

hold off
print -depsc graphs/ex-33.eps
%% fig5
figure(5);

laterial_acc = deg2rad(insData.yawRate.value).* insData.vxCG.value;
laterial_acc_filtered = deg2rad(insData.yawRateFilt.value).* insData.vxCG.value;

plot(insData.yawRate.time,insData.ayCGFilt.value,'lineWidth',3);
hold on
plot(insData.yawRate.time,laterial_acc,'lineWidth',3);
% plot(insData.yawRate.time,laterial_acc_filtered,'lineWidth',3);
legend('INS filtered','calculated');
xlabel('Time(s)')
ylabel('$A_{y}  (m/s^{2})$');
grid on

hold off
print -depsc graphs/ex-34.eps
%% fig6

acc_calc = gradient(non_zero_tick(:,3)) ./non_zero_tick(:,2);
acc_calc_mean = movmean(acc_calc,6);
acc_calc_RL = gradient(non_zero_tick_RL(:,3)) ./non_zero_tick_RL(:,2);

% acc_calc = [0;diff(non_zero_tick(:,3))];
% acc_calc_RL = [0;diff(non_zero_tick_RL(:,3))];

figure(6);
plot(insData.axCG.time,insData.axCG.value,'lineWidth',3);
hold on
% plot(non_zero_tick_RL(:,1),acc_calc_RL,'lineWidth',1);
plot(non_zero_tick(:,1),acc_calc_mean,'lineWidth',3);
legend('INS acceleration','calculated acceleration RR');
xlabel('Time(s)')
ylabel('$A_{y}  (m/s^{2})$');
grid on

slip_ang = 50 * atan(insData.vyCG.value ./  insData.vxCG.value);

hold off
print -depsc graphs/ex-35.eps
%% fig7
figure(7);
plot(insData.vxCG.time,slip_ang,'lineWidth',4);
hold on
plot(insData.sideSlip.time,insData.sideSlip.value,'lineWidth',2);
legend('calculated side slip','INS side slip');
xlabel('Time(s)')
ylabel('$\beta$ (deg)');
ylim ([-4 6]);
% xlim([0 700]);
grid on

hold off
print -depsc graphs/ex-36.eps