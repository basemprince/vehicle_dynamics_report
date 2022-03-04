% -------------------------------------
%% Pure longitudinal slip conditions
% -------------------------------------

clc
clearvars 
close all   

% Set LaTeX as default interpreter for axis labels, ticks and legends
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

set(0,'DefaultFigureWindowStyle','docked');
set(0,'defaultAxesFontSize',20)
set(0,'DefaultLegendFontSize',40)
blue = '#0072BD';
red = '#D95319';
orange = '#EDB120';
green = '#77AC30';
% plot_colors = [blue; red; orange; green];
plot_colors = ['r' 'b' 'g' 'm'];
% ------------------
%% Load the raw tyre data
% ------------------
load('B1464run30'); 

disp(testid)
disp(tireid)
%% Plot the data

figure (1), clf
% long slip %
ax(1) = subplot(511);
plot(SL,'LineWidth',2)
grid on
title('Longitudinal Slip $\kappa$')
% slip angle %
ax(2) = subplot(512);
plot(SA,'LineWidth',2)
grid on
title('Slip Angle $\alpha$ [deg]')
% camber %
ax(3) = subplot(513);
plot(IA,'LineWidth',2)
grid on
title('Camber $\gamma$ [deg]')
% Fz %
ax(4) = subplot(514);
plot(FZ,'LineWidth',2)
grid on
title('$F_z$ [N]')
%pressure %
ax(5) = subplot(515);
plot(P,'LineWidth',2)
grid on
title('Pressure [kPa]')
print -depsc graphs/ex-21.eps

%% Organize the data
%   create a vector of idxs dividing the dataset by parameters

% Pressure indexing
P_thresh = 20;
P_step = find(abs(diff(P))>P_thresh);
P_step = [P_step; length(P)];
P_test = unique(round(P(P_step)));
[~,index] = min(abs(P_test'-P),[],2);
P_fixed = P_test(index);

% camber indexing
IA_thresh = 0.5 ;
IA_step = find(abs(diff(IA))>IA_thresh);
IA_step = [IA_step; length(IA)];
IA_len = length(IA_step);
IA_test = sort(unique(round(IA(IA_step))));
[~,index] = min(abs(IA_test'-IA),[],2);
IA_fixed = IA_test(index);


% slip angle [logitudinal slip] indexing
SA_thresh = 0.5;
SA_step = find(abs(diff(SA))>SA_thresh);
SA_step = [SA_step; length(SA)];
SA_test = sort(unique(round(SA(SA_step))),'descend');
[~,index] = min(abs(SA_test'-SA),[],2);
SA_fixed = SA_test(index);

% vertical force indexing
FZ_test = [-210 -670 -890 -1120]; % tested vertical forces

% create a separate bin for each test conducted given the adjusted input parameters
test_bins = cell(1, IA_len);

bin_start = 1;
for i=1 :IA_len
    test_bins{i} = (bin_start:1:IA_step(i))';
    curr = test_bins{i};
    % approximate vertical forces FZ to the tested vertical forces
    [~,index] = min(abs(FZ_test-FZ(curr)),[],2);
    % [index_n, camber, slip angle, vertical force, pressure,
    % longitudina slip, longitudinal force]
    test_bins{i} = [curr, round(IA(curr)), SA_fixed(curr),FZ_test(index)',P_fixed(curr),SL(curr),FX(curr)];
    bin_start = IA_step(i)+1;
end

test_bin_params = [];
% remove bins that has variation in FZ
for i=1 :length(test_bins)
    curr = test_bins{i}(:,4);
    Fz_M = mean(curr);
    if ~any(FZ_test(:) == Fz_M)
        test_bins{i} = [];
    else
        [~,SL_L] = min(test_bins{i}(:,6));
        test_bins{i} = test_bins{i}(SL_L:end,:); % take only second half of each test [-0.2 -> 0.2]
        IA_M = mean(test_bins{i}(:,2));
        SA_M = mean(test_bins{i}(:,3));
        P_M = mean(test_bins{i}(:,5));
        test_bin_params = [test_bin_params; [IA_M,SA_M,Fz_M,P_M]];
    end
end

test_bins = test_bins(~cellfun('isempty',test_bins));

%% Fig2
%  Perfom some plot (es. k VS Fx for all Fz, with gamma = 0, alpha = 0)

IA_SA_zero = find(test_bin_params(:,1)==0 & test_bin_params(:,2)==0 & test_bin_params(:,4)== 83);
IA_SA_zero = test_bins(IA_SA_zero);
IA_SA_zero_t = [];
for i = 1 : length(IA_SA_zero)
    IA_SA_zero_t = [IA_SA_zero_t; IA_SA_zero{i}];
end

for i = 1 : length(FZ_test)
    FZ_curr = FZ_test(i);
    index_to_plot = find(IA_SA_zero_t(:,4) == FZ_curr);
    figure(2);
    ss= strcat('$F_z= ',num2str(-1*FZ_test(i)),'$');
    plot(IA_SA_zero_t(index_to_plot,6),IA_SA_zero_t(index_to_plot,7),':o','Color',plot_colors(i),'DisplayName',ss,'LineWidth',2,'MarkerSize',2);
    hold on
end
legend('Location','southeast','FontSize',30);
xlabel('$\kappa$')
ylabel('$F_x(\kappa)$');
xlim([-0.25 0.25]);
grid on
pbaspect([1.5 1 1]);
hold off
print -depsc graphs/ex-22.eps

%% Fig3

FZ67_IA_zero = find(test_bin_params(:,1)==0 & test_bin_params(:,3)==-670 & test_bin_params(:,4)== 83);
FZ67_IA_zero = test_bins(FZ67_IA_zero);
FZ67_IA_zero_t = [];
for i = 1 : length(FZ67_IA_zero)
    FZ67_IA_zero_t = [FZ67_IA_zero_t; FZ67_IA_zero{i}];
end


for i = 1 : length(SA_test)
    SA_curr = SA_test(i);
    index_to_plot = find(FZ67_IA_zero_t(:,3) == SA_curr);
    figure(3);
    ss= strcat('$\alpha= ',num2str(-1*SA_test(i)),'$');
    plot(FZ67_IA_zero_t(index_to_plot,6),FZ67_IA_zero_t(index_to_plot,7),':o','Color',plot_colors(i),'DisplayName',ss,'LineWidth',2,'MarkerSize',2);
    hold on
end
legend('Location','southeast','FontSize',30);
xlabel('$\kappa$')
ylabel('$F_x(\kappa)$');
grid on
xlim([-0.25 0.25]);
pbaspect([1.5 1 1]);
hold off
print -depsc graphs/ex-23.eps

%% Execute the first fitting
%   1. Select the data alpha = 0, gamma = 0, Fz=Fz_nom
%   2. Write Pacejka MC and fit the data 
%   3. Get the parameter

FZ89_IA_SA_zero = find(test_bin_params(:,1)==0 & test_bin_params(:,2)==0 & test_bin_params(:,3)==-890 & test_bin_params(:,4)== 83);
FZ89_IA_SA_zero = test_bins(FZ89_IA_SA_zero);
FZ89_IA_SA_zero_t = [];
for i = 1 : length(FZ89_IA_SA_zero)
    FZ89_IA_SA_zero_t = [FZ89_IA_SA_zero_t; FZ89_IA_SA_zero{i}];
end

SL_1 = FZ89_IA_SA_zero_t(:,6);
FX_1 = FZ89_IA_SA_zero_t(:,7);
FZ0_1 = -890;
IA0 = 0;
X0 = -1.5 * ones(1,15);
X0(8:14) = 0;
X0(15) = 20;
X_fz_nom_1 = fmincon(@(X)resid_pure_Fx(X,FX_1,SL_1,IA0,FZ0_1),X0);
Fx0 = pajekaFormula(X_fz_nom_1,SL_1,IA0,FZ0_1);

figure(4);
plot(SL_1,FX_1,'.b','MarkerSize',10,'DisplayName','$F_x$');
hold on
plot(SL_1,Fx0,'--r','LineWidth',5,'DisplayName','$F_x$ [fit]');

legend('Location','southeast','FontSize',30);
xlabel('$\kappa$')
ylabel('$F_x(\kappa)$');
grid on;
xlim([-0.25 0.25]);
pbaspect([1.5 1 1]);
hold off
print -depsc graphs/ex-24.eps

%% Execute the second fitting
%   1. Select the data for Fz=change and gamma = 0
%   2. Fit the data using the previus parameters
%   4. Get the new parameters

X0_fz = X_fz_nom_1;
X_fz_nom_2 = X_fz_nom_1;

for i = 1 : length(FZ_test)

    FZ_curr = FZ_test(i);
    relivant_ind = find(IA_SA_zero_t(:,4) == FZ_curr);
    SL_2 = IA_SA_zero_t(relivant_ind,6);
    FX_2 = IA_SA_zero_t(relivant_ind,7);
    FZ0_2= IA_SA_zero_t(relivant_ind,4);
    X_fz_nom_2 = fmincon(@(X)resid_pure_Fx_varFz(X,FX_2,SL_2,IA0,FZ_curr,X_fz_nom_2),X0_fz);
    X0_fz = X_fz_nom_2;
    Fx0_2 = pajekaFormula(X_fz_nom_2,SL_2,IA0,FZ_curr);
    figure(5);
    ss1= strcat('$F_z= ',num2str(-1*FZ_test(i)),'$');
    ss2= strcat('$F_z= ',num2str(-1*FZ_test(i)),'[fit]$');
    plot(SL_2,FX_2,'.','Color',plot_colors(i),'DisplayName',ss1,'LineWidth',2,'MarkerSize',10);
    hold on
    plot(SL_2,Fx0_2,'-','Color',plot_colors(i),'DisplayName',ss2,'LineWidth',2,'MarkerSize',2);
end

legend('Location','northwest','FontSize',20,'NumColumns',4);
xlabel('$\kappa$')
ylabel('$F_x(\kappa)$');
grid on
xlim([-0.25 0.25]);
pbaspect([1.5 1 1]);
hold off
print -depsc graphs/ex-25.eps

%% Execute the third fitting
%   1. Select the data for Fz=change and gamma = change
%   2. Fit the data using the previus parameters
%   4. Get the last parameters

FZ89_SA_zero = find(test_bin_params(:,2)==0 & test_bin_params(:,3)==-890 & test_bin_params(:,4)== 83);
FZ89_SA_zero = test_bins(FZ89_SA_zero);
FZ89_SA_zero_t = [];
for i = 1 : length(FZ89_SA_zero)
    FZ89_SA_zero_t = [FZ89_SA_zero_t; FZ89_SA_zero{i}];
end

X0_ai = X_fz_nom_2;
X_fz_nom_3 = X_fz_nom_2;
FZ0_3= -890;
for i = 1 : length(IA_test)

    IA_curr = IA_test(i);
    relivant_ind = find(FZ89_SA_zero_t(:,2) == IA_curr);
    SL_3 = FZ89_SA_zero_t(relivant_ind,6);
    FX_3 = FZ89_SA_zero_t(relivant_ind,7);
    X_fz_nom_3 = fmincon(@(X)resid_pure_Fx_varAI(X,FX_3,SL_3,IA_curr,FZ0_3,X_fz_nom_3),X0_ai);
    X0_ai = X_fz_nom_3;
    Fx0_3 = pajekaFormula(X_fz_nom_3,SL_3,IA_curr,FZ0_3);
    figure(6);
    ss1= strcat('$\gamma= ',num2str(IA_test(i)),'$');
    ss2= strcat('$\gamma= ',num2str(IA_test(i)),'[fit]$');
    plot(SL_3,FX_3,':','Color',plot_colors(i),'DisplayName',ss1,'LineWidth',2,'MarkerSize',22);
    hold on
    plot(SL_3,Fx0_3,'-','Color',plot_colors(i),'DisplayName',ss2,'LineWidth',2,'MarkerSize',2);
end

legend('Location','northwest','FontSize',20,'NumColumns',4);
xlabel('$\kappa$')
ylabel('$F_x(\kappa)$');
grid on
xlim([-0.25 0.25]);
pbaspect([1.5 1 1]);
hold off
print -depsc graphs/ex-26.eps