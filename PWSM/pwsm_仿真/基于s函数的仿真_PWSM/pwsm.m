function [sys,x0,str,ts] = pmsm(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts] = mdlInitializeSizes;
case 1,
    sys = mdlDerivatives(t,x,u);
case 3,
    sys = mdlOutputs(t,x,u);
case {2,4,9}
    sys = [];
otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

%==========================================================================
function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
% 定义输入输出的个数，系统状态变量个数以及其他
sizes.NumContStates  = 3;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1; % at least one sample time is needed

sys = simsizes(sizes);

% initialize the initial conditions
x0  = [0;0;0]; % 系统的初始状态
% str is always an empty matrix
str = [];
% initialize the array of sample times
ts  = [0 0];
simStateCompliance = 'UnknownSimState';

%==========================================================================
function sys = mdlDerivatives(t,x,u) %连续时间系统的微分方程
%% 电机参数设置
R = 2.875;
Ld = 8.5e-3;
Lq = 8.5e-3;
Pn = 4;
Phi = 0.175;
J = 0.001;
B = 0.008;

% x(1),x(2),x(3)分别对应系统的三个状态变量 id,iq 和 wm
% u(1),u(2),u(3)分别对应 ud,uq 和 TL

sys(1) = (1/Ld)*u(1) - (R/Ld)*x(1) + (Lq/Ld)*Pn*x(2)*x(3);

sys(2) = (1/Lq)*u(2) - (R/Lq)*x(2) - (Ld/Lq)*Pn*x(3)*x(2) - (Phi*Pn/Lq)*x(3);

sys(3) = (1/J)*(1.5 * Pn * (Phi*x(2) + (Ld-Lq)*x(2)*x(3)) - B*x(3) - u(3));

%==========================================================================
function sys = mdlOutputs(t,x,u) %设定系统的输出变量
sys(1) = x(1);
sys(2) = x(2);
sys(3) = x(3);
