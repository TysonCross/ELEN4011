%% ELEN4011 Engineering Design
% Control system for UAV
% Tyson Cross       1239448

clc; clear all; close all;

system_setup;

%% State Space equations of motion (small peturbations)
% ------------------------------------------------------------------------
%%% Longitudinal
disp('==================== Longitudinal EOM ============================')
sys.long.A = [	deriv.x_u, 	deriv.x_w,	deriv.x_q,	deriv.x_theta,  0 ;
                deriv.z_u,	deriv.z_w,	deriv.z_q,	deriv.z_theta,  0 ;
                deriv.m_u, 	deriv.m_w,	deriv.m_q,	deriv.m_theta,  0 ;
                0,         	0,          1,          0,              0 ;
                deriv.th_u, deriv.th_w, deriv.th_q, env.V_0,        0 ];

sys.long.B = [ 	deriv.x_eta,    deriv.x_tau     ;
                deriv.z_eta,    deriv.z_tau     ;
                deriv.m_eta,    deriv.m_tau     ;
                0,              0               ;
                0,              0               ];

sys.long.C =  eye(5);

sys.long.D = zeros(size(sys.long.C,1),size(sys.long.B,2));

sys.long.SS = ss(sys.long.A, sys.long.B, sys.long.C, sys.long.D, ...
    'Name', 'Longitude model', ...
    'StateName', {'u'; 'w'; 'q'; 'theta'; 'h'}, ...
    'InputName', {'elevator'; 'throttle'}, ...
    'OutputName', {'x'; 'z'; 'pitch rate'; 'pitch angle'; 'height'});

sys.long.TF = tf(sys.long.SS);

sys.long.poles = eig(sys.long.SS);
figs.long.fig1 = figure('Position',[1240,120,650,700], ...
    'Name', 'Longitudinal Step Response');
step(sys.long.TF,50);

figs.long.fig2 = figure('Position',[680,490,550,325], ...
    'Name', 'Longitudinal Pole Zero Map');
pzmap(sys.long.SS);
grid on;

% check observability/controlability
if size(sys.long.A,1)==rank(ctrb(sys.long.A, sys.long.B)) 
    disp('The LTI system is controllable')
else
    warning('The LTI system is not controllable!')
end

if size(sys.long.A,1)==rank(obsv(sys.long.A, sys.long.C))
        disp('The LTI system is observable')
else
    warning('The LTI system is not observable!')
end

sys.long.num = sys.long.TF.Numerator;
sys.long.den = sys.long.TF.Denominator;

sys.long.SS

disp('======================= Lateral EOM ==============================')
% ------------------------------------------------------------------------
%%% Lateral
sys.lat.A = [   deriv.y_v,	deriv.y_p,	deriv.y_r,	deriv.y_phi,    deriv.y_psi ;
                deriv.l_v,	deriv.l_p,	deriv.l_r,	deriv.l_phi,    deriv.l_psi ;
                deriv.n_v,	deriv.n_p,	deriv.n_r,	deriv.n_phi,    deriv.n_psi ;
                0,         	1,          0,          0,              0           ;
                0,         	0,          1,          0,              0           ];

sys.lat.B = [   deriv.y_xi,	deriv.y_zeta ;
                deriv.l_xi,	deriv.l_zeta ;
                deriv.n_xi,	deriv.n_zeta ;
                0,         	0            ;
                0,         	0            ];

sys.lat.C =  eye(5);

sys.lat.D = zeros(size(sys.lat.C, 1), size(sys.lat.B, 2));

sys.lat.SS = ss(sys.lat.A, sys.lat.B, sys.lat.C, sys.lat.D, ...
    'Name', 'Lateral model', ...
    'StateName', {'v'; 'p'; 'r'; 'phi'; 'psi'}, ...
    'InputName', {'aerilons'; 'rudder'}, ...
    'OutputName', {'v'; 'roll rate'; 'yaw rate'; 'roll angle'; 'yaw angle'});

sys.lat.TF = tf(sys.lat.SS);

sys.lat.poles = eig(sys.lat.SS);
figs.lat.fig1 = figure('Position', [1900,120,650,700], ...
    'Name', 'Latitudinal Step Response');
step(sys.lat.TF, 50);

figs.lat.fig2 = figure('Position',[680,60,550,325], ...
    'Name', 'Latitudinal Pole Zero Map');
pzmap(sys.lat.SS);
grid on;

% check observability/controlability
if size(sys.lat.A, 1)==rank(ctrb(sys.lat.A, sys.lat.B)) 
    disp('The LTI system is controllable')
else
    warning('The LTI system is not controllable!')
end

if size(sys.lat.A, 1)==rank(obsv(sys.lat.A, sys.lat.C))
        disp('The LTI system is observable')
else
    warning('The LTI system is not observable!')
end

sys.lat.num = sys.lat.TF.Numerator;
sys.lat.den = sys.lat.TF.Denominator;

sys.lat.SS
pzmap(sys.lat.SS)
grid on;

actuators.naturalFreq = 35;
actuators.dampingRatio = 0.75;
[actuators.num, actuators.den]= ord2(35,0.75)
actuators.TF = tf(actuators.num,actuators.den)

actuators.thrust.gain = 1;
actuators.thrust.T_tau = 0.01;
actuators.thrust.TF = tf(actuators.thrust.gain,[-actuators.thrust.T_tau 1]);
