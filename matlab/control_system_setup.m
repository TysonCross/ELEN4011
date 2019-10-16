%% ELEN4011 Engineering Design
% Control system for UAV
% Tyson Cross       1239448

clc; clear all; close all;

system_setup;

%% State Space equations of motion (small peturbations)
% ------------------------------------------------------------------------
%%% Longitudinal
disp('==================== Longitudinal EOM ============================')
sys.A.long = [	deriv.x_u, 	deriv.x_w,	deriv.x_q,	deriv.x_theta,  0 ;
                deriv.z_u,	deriv.z_w,	deriv.z_q,	deriv.z_theta,  0 ;
                deriv.m_u, 	deriv.m_w,	deriv.m_q,	deriv.m_theta,  0 ;
                0,         	0,          1,          0,              0 ;
                deriv.th_u, deriv.th_w, deriv.th_q, env.V_0,        0 ];

sys.B.long = [ 	deriv.x_eta,    deriv.x_tau     ;
                deriv.z_eta,    deriv.z_tau     ;
                deriv.m_eta,    deriv.m_tau     ;
                0,              0               ;
                0,              0               ];

sys.C.long =  eye(5);

sys.D.long = zeros(size(sys.C.long,1),size(sys.B.long,2));

sys.SS.long = ss(sys.A.long, sys.B.long, sys.C.long, sys.D.long, ...
    'Name', 'Longitude model', ...
    'StateName', {'u'; 'w'; 'q'; 'theta'; 'h'}, ...
    'InputName', {'elevator'; 'throttle'}, ...
    'OutputName', {'x'; 'z'; 'pitch rate'; 'pitch angle'; 'height'});

sys.TF.long = tf(sys.SS.long);

sys.poles.long = eig(sys.SS.long);
figs.long.fig1 = figure('Position',[1240,120,650,700], ...
    'Name', 'Longitudinal Step Response');
step(sys.TF.long,50);

figs.long.fig2 = figure('Position',[680,490,550,325], ...
    'Name', 'Longitudinal Pole Zero Map');
pzmap(sys.SS.long);
grid on;

% check observability/controlability
if size(sys.A.long,1)==rank(ctrb(sys.A.long, sys.B.long)) 
    disp('The LTI system is controllable')
else
    warning('The LTI system is not controllable!')
end

if size(sys.A.long,1)==rank(obsv(sys.A.long, sys.C.long))
        disp('The LTI system is observable')
else
    warning('The LTI system is not observable!')
end

sys.num.long = sys.TF.long.Numerator;
sys.den.long = sys.TF.long.Denominator;

sys.SS.long

disp('======================= Lateral EOM ==============================')
% ------------------------------------------------------------------------
%%% Lateral
sys.A.lat = [   deriv.y_v,	deriv.y_p,	deriv.y_r,	deriv.y_phi,    deriv.y_psi ;
                deriv.l_v,	deriv.l_p,	deriv.l_r,	deriv.l_phi,    deriv.l_psi ;
                deriv.n_v,	deriv.n_p,	deriv.n_r,	deriv.n_phi,    deriv.n_psi ;
                0,         	1,          0,          0,              0           ;
                0,         	0,          1,          0,              0           ];

sys.B.lat = [   deriv.y_xi,	deriv.y_zeta ;
                deriv.l_xi,	deriv.l_zeta ;
                deriv.n_xi,	deriv.n_zeta ;
                0,         	0            ;
                0,         	0            ];

sys.C.lat =  eye(5);

sys.D.lat = zeros(size(sys.C, 1), size(sys.B, 2));

sys.SS.lat = ss(sys.A.lat, sys.B.lat, sys.C.lat, sys.D.lat, ...
    'Name', 'Lateral model', ...
    'StateName', {'v'; 'p'; 'r'; 'phi'; 'psi'}, ...
    'InputName', {'aerilons'; 'rudder'}, ...
    'OutputName', {'v'; 'roll rate'; 'yaw rate'; 'roll angle'; 'yaw angle'});

sys.TF.lat = tf(sys.SS.lat);

sys.poles.lat = eig(sys.SS.lat);
figs.lat.fig1 = figure('Position', [1900,120,650,700], ...
    'Name', 'Latitudinal Step Response');
step(sys.TF.lat, 50);

figs.lat.fig2 = figure('Position',[680,60,550,325], ...
    'Name', 'Latitudinal Pole Zero Map');
pzmap(sys.SS.lat);
grid on;

% check observability/controlability
if size(sys.A.lat, 1)==rank(ctrb(sys.A.lat, sys.B.lat)) 
    disp('The LTI system is controllable')
else
    warning('The LTI system is not controllable!')
end

if size(sys.A.lat, 1)==rank(obsv(sys.A.lat, sys.C.lat))
        disp('The LTI system is observable')
else
    warning('The LTI system is not observable!')
end

sys.num.lat = sys.TF.lat.Numerator;
sys.den.lat = sys.TF.lat.Denominator;

sys.SS.lat
pzmap(sys.SS.lat)
grid on;