clc
clearvars
close all

set (0, 'defaulttextinterpreter','latex')
set (groot, 'defaultAxesTickLabelInterpreter','latex')
set (groot, 'defaultLegendInterpreter', 'latex')
set (0,'defaultAxesFontSize', 20)
set (0, 'DefaultLegendFontSize',20)

% exerices 1
%% part1
Bx = 12.2;
Cx = 1.87;
Dx = 2100;
Ex = 0.13;
Sh = 0.851e-3;
Sv = -77;
ki_v = -1:0.001:1;
Y = MagicFormula(ki_v,Bx,Cx,Dx,Ex,Sh,Sv);
figure(1);
plot(ki_v,Y,'-r','LineWidth',2);
xlabel('$\kappa$')
ylabel('$F_x(\kappa)$');
grid on;

[pks,I] = max(Y);
best_k = ki_v(I);
fprintf('maximum Fx = %f\n', pks);
fprintf('best k = %f\n', best_k);


%% part2
w = 70;
Re = 0.2;
Vcx = 13;

ls = longitudinalSlip(w,Re,Vcx);
fprintf('calculated longitduinal slip = %f\n', ls); 
Fx0 = MagicFormula(ls,Bx,Cx,Dx,Ex,Sh,Sv);
fprintf('calculated longitduinal force = %f\n', Fx0); 
BCD = Bx * Cx * Dx;
fprintf('calculated cornering stiffness Cfk = %f\n', BCD);

ki_v_s = -1:0.01:1;
Y_s = MagicFormula(ki_v_s,Bx,Cx,Dx,Ex,Sh,Sv);
Y_app = ki_v_s * BCD;
Y_er = abs(Y_s - Y_app)*100 ./abs(Y_s);

figure(2);
plot(ki_v,Y,'-r','LineWidth',2);
hold on
plot(ki_v_s,Y_app,'--b','LineWidth',2);
hold off
xlabel('$\kappa$')
ylabel('$F_x(\kappa)$');
ylim([-3000 3000])
grid on;


%% part2-2
figure(3);
Y_er_q= Y_er;
Y_er_q(Y_er_q>=100)=nan;
plot(ki_v_s,Y_er_q,'-b','LineWidth',2);
xlabel('$\kappa$')
ylabel('$F_x(\kappa)$ \% difference');
% xlim([-0.03 0.025])
ylim([0 100])
grid on;


%% exercise 2
k = 0.08;
Bxa = 23.308 / (sqrt(((19.414*k)^2)+1));
Cxa = 0.926;
SHxa = -0.1257*10^-2;
Dxa = 1 / (cos(Cxa*atan(Bxa*SHxa)));

Vcx_2 =  15;
Vcy_2 = -1.3;
alpha = sideSlip(Vcx_2,Vcy_2);
Gxa = weighingFunc(Dxa,Cxa,Bxa,alpha,SHxa);
Fx0_k = MagicFormula(k,Bx,Cx,Dx,Ex,Sh,Sv);
Fx = Gxa * Fx0_k;
fprintf('calculated longitduinal force Fx0 at 0.8= %f\n', Fx0); 
fprintf('calculated slide slip alpha = %f\n', alpha); 
fprintf('calculated weighing function = %f\n', Gxa);
fprintf('calculated combined force Fx = %f\n', Fx);

alpha_array = [0 2 4 6 8];
alpha_array_d = deg2rad(alpha_array);
Bxa_f = 23.308 ./ (sqrt((19.414*ki_v).^2+1));
Dxa_f = 1 ./ (cos(Cxa*atan(Bxa_f*SHxa)));

% alpha_c = deg2rad(alpha_array(1));
% Gxa_c = weighingFunc(Dxa,Cxa,Bxa,alpha_c,SHxa);
% Fx_c = Gxa_c * Y;
% fprintf('slide slip alpha_c = %f\n', alpha_c); 
% fprintf('calculated weighing function Gxa_c= %f\n', Gxa_c);
% fprintf('calculated combined force Fx_c = %f\n', Fx_c);
% figure(4);
% plot(ki_v,Fx_c,'-r','LineWidth',2);
% xlabel('$\kappa$')
% ylabel('$F_x(\kappa)$');
% grid on;


for i = 1 : length(alpha_array)
alpha_c = alpha_array_d(i);
Gxa_c = weighingFunc(Dxa_f,Cxa,Bxa_f,alpha_c,SHxa);
figure(4);
ss= strcat('$\alpha=',num2str(alpha_array(i)),'$');
Fx_c = Gxa_c .* Y;
plot(ki_v,Fx_c,'DisplayName',ss,'LineWidth',2);
hold on
legend;
xlabel('$\kappa$')
ylabel('$F_x(\kappa)$');
grid on
end

%% figure 5
for i = 1 : length(alpha_array)
alpha_c = alpha_array_d(i);
Gxa_c = weighingFunc(Dxa_f,Cxa,Bxa_f,alpha_c,SHxa);
figure(5);
ss= strcat('$\alpha=',num2str(alpha_array(i)),'$');
plot(ki_v,Gxa_c,'DisplayName',ss,'LineWidth',2);
hold on
legend;
xlabel('$\kappa$')
ylabel('$G_xa$');
grid on
end
print -depsc myfig4.eps


