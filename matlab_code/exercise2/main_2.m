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


% ------------------
%% Load the raw tyre data
% ------------------
load('B1464run30'); 

disp(testid)
disp(tireid)
%% Plot the data

figure ('Name','Raw data', 'NumberTitle','off'), clf
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
print -depsc ex-21.eps

%% Organize the data
%   1. create a vector of idxs dividing the dataset by parameters
%   2. Perfom some plot (es. k VS Fx for all Fz, with gamma = 0, alpha = 0)

IAstep = find(abs(diff(IA))>0.5);
IAstep = [IAstep, round(IA(IAstep))];

eps = 0.5 ;
idx_IA_zero = IA<=eps;
IA_zero = IA(idx_IA_zero);


eps = -0.5 ;
idx_SA_zero = SA>=eps;
SA_zero = SA(idx_SA_zero);

FZtest = [-210 -670 -890 -1120];
FZ_fixed = [];
for i=1:length(FZ)
    [c,index] = min(abs(FZtest-FZ(i)));
    FZ_fixed = [FZ_fixed ; FZtest(index)];
end

zero_IA_SA = [];
FX_zero_IA_SA = [];
SL_zero_IA_SA = [];

for i=1:length(idx_IA_zero)
    if idx_IA_zero(i) ==1 && idx_SA_zero(i) == 1 % && P(i) < 60
        zero_IA_SA = [zero_IA_SA; i];
        FX_zero_IA_SA = [FX_zero_IA_SA; [FX(i),i]];
        SL_zero_IA_SA = [SL_zero_IA_SA; [SL(i),i]];
    end
end



for i = 1 : length(FZtest)
    result = find(FZ_fixed==FZtest(i));
    [c,index_to_plot,ib] = intersect(SL_zero_IA_SA(:,2),result);
    figure(2);
    ss= strcat('$F_z= ',num2str(FZtest(i)),'$');
    plot(SL_zero_IA_SA(index_to_plot),FX_zero_IA_SA(index_to_plot),'.','DisplayName',ss,'MarkerSize',15);
    hold on
    legend('Location','southeast','FontSize',30);
    xlabel('$\kappa$')
    ylabel('$F_x(\kappa)$');
    grid on
end
hold off
print -depsc ex-22.eps

%% fig3
SAstep = find(abs(diff(SA))>0.5);
SAtest = unique(round(SA(SAstep)));
SA_fixed = [];

for i=1:length(SA)
    [c,index] = min(abs(SAtest-SA(i)));
    SA_fixed = [SA_fixed ; SAtest(index)];
end


six_IA_zero_SA = [];
FX_six_IA_zero_SA = [];
SL_six_IA_zero_SA = [];
for i=1:length(idx_IA_zero)
    if idx_IA_zero(i) ==1 && FZ_fixed(i) == -670 % && P(i) < 60
        six_IA_zero_SA = [six_IA_zero_SA; i];
        FX_six_IA_zero_SA = [FX_six_IA_zero_SA; [FX(i),i]];
        SL_six_IA_zero_SA = [SL_six_IA_zero_SA; [SL(i),i]];
    end
end


for i = 1 : length(SAtest)
    result = find(SA_fixed==SAtest(i));
    [c,index_to_plot,ib] = intersect(SL_six_IA_zero_SA(:,2),result);
    figure(3);
    ss= strcat('$\alpha= ',num2str(SAtest(i)),'$');
    plot(SL_six_IA_zero_SA(index_to_plot),FX_six_IA_zero_SA(index_to_plot),'.','DisplayName',ss,'MarkerSize',15);
    hold on
    legend('Location','southeast','FontSize',30);
    xlabel('$\kappa$')
    ylabel('$F_x(\kappa)$');
    grid on
end
    hold off
print -depsc ex-23.eps
%% Execute the first fitting
%   1. Select the data alpha = 0, gamma = 0, Fz=Fz_nom
%   2. Write Pacejka MC and fit the data 
%   3. Get the parameter

Fz_l = [-1500 -950 -750 -300];
Fz_h = [-1000 -750 -600 0];
tol = 0.2;

IA_zero_test = find(IA < tol) ;
Fz_890 = find(FZ > Fz_l(:,2) & FZ < Fz_h(:,2));
P_82 = find(P>75);
SA_zero_test = find(SA > -tol);

idx1 = intersect(IA_zero_test,intersect(SA_zero_test,intersect(Fz_890,P_82)));
FX1 = FX(idx1);
SL1 = SL(idx1);
FZ0= 890;
IA0 = 0;
X0 = ones(1,15);
% X0 = [2.7379 3.2434 0.0031 -0.0371 -53.4831 -0.0044 0.0827 0.3242 -0.0041 0.0031 -0.0057 0.0104 0.5577 0.0099 0];

X_fz_nom = fmincon(@(X)resid_pure_Fx(X,FX1,SL1,IA0,FZ0),X0,[],[],[],[],[],[]);
Fx0 = pajekaFormula(X_fz_nom,SL1,IA0,FZ0);
% X_trial = [2.7379 3.2434 0.0031 -0.0371 -53.4831 -0.0044 0.0827 0.3242 -0.0041 0.0031 -0.0057 0.0104 0.5577 0.0099 0];
% Fx0_trial = pajekaFormula(X_trial,SL1,IA0,FZ0);
figure(4);
plot(SL1,Fx0,'.r','MarkerSize',10);
hold on
plot(SL1,FX1,'.b','LineWidth',2);
legend({'fitted Fx0','test data'},'Location','southeast','FontSize',30);
hold off
xlabel('$\kappa$')
ylabel('$F_x(\kappa)$');
% ylim([-3000 3000])
grid on;
print -depsc ex-24.eps
%% Execute the second fitting
%   1. Select the data for Fz=change and gamma = 0
%   2. Fit the data using the previus parameters
%   4. Get the new parameters

idx2 = intersect(IA_zero_test,intersect(SA_zero_test,P_82));
FX2 = FX(idx2);
SL2 = SL(idx2);
FZ0_2= abs(FZ(idx2));
FZ_fixed_2 = FZ_fixed(idx2);

X0_fz = X_fz_nom;

X_fz_nom_2 = fmincon(@(X)resid_pure_Fx_varFz(X,FX2,SL2,IA0,FZ0_2,X_fz_nom),X0_fz,[],[],[],[],[],[]);
Fx0_2 = pajekaFormula(X_fz_nom_2,SL2,IA0,FZ0_2);
% X_trial = [2.7379 3.2434 0.0031 -0.0371 -53.4831 -0.0044 0.0827 0.3242 -0.0041 0.0031 -0.0057 0.0104 0.5577 0.0099 0];
% Fx0_trial = pajekaFormula(X_trial,SL2,IA0,FZ0_2);
for i = 1 : length(FZtest)
    index_to_plot = find(FZ_fixed_2==FZtest(i));
    figure(5);
    ss= strcat('$F_z= ',num2str(FZtest(i)),'$');
    plot(SL2(index_to_plot),Fx0_2(index_to_plot),'.','DisplayName',ss,'MarkerSize',15);
    hold on
    legend;
    xlabel('$\kappa$')
    ylabel('$F_x(\kappa)$');
    grid on
end


%% Execute the third fitting
%   1. Select the data for Fz=change and gamma = change
%   2. Fit the data using the previus parameters
%   4. Get the last parameters