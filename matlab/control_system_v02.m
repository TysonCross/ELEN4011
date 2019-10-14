%% ELEN4011 Engineering Design
% Control system for UAV
% Tyson Cross       1239448

clc; clear all; close all;

system_setup;

%% State Space equations of motion (small peturbations)
height_augmented = false;

% ------------------------------------------------------------------------
%%% Longitudinal
disp('==================== Longitudinal EOM ============================')
if height_augmented
    A_long = [	deriv.x_u, 	deriv.x_w,	deriv.x_q,	deriv.x_theta,  0 ;
                deriv.z_u,	deriv.z_w,	deriv.z_q,	deriv.z_theta,  0 ;
                deriv.m_u, 	deriv.m_w,	deriv.m_q,	deriv.m_theta,  0 ;
                0,         	0,          1,          0,              0 ;
                0,         	-1,         0,          env.V_0,        0 ]

    B_long = [ 	deriv.x_eta,    deriv.x_tau     ;
                deriv.z_eta,    deriv.z_tau     ;
                deriv.m_eta,    deriv.m_tau     ;
                0,              0               ;
                0,              0               ]

    C_long =  eye(5)

    D_long = zeros(size(C_long,1),size(B_long,2));

    sys_long = ss(A_long,B_long,C_long,D_long,'StateName', {'u'; 'w'; 'q'; 'theta'; 'h'}, ...
        'InputName', {'elevator'; 'throttle'}, ...
        'OutputName', {'x'; 'z'; 'pitch rate'; 'pitch angle'; 'height'})
else
    A_long = [	deriv.x_u, 	deriv.x_w,	deriv.x_q,	deriv.x_theta ;
               	deriv.z_u,	deriv.z_w,	deriv.z_q,	deriv.z_theta ;
              	deriv.m_u, 	deriv.m_w,	deriv.m_q,	deriv.m_theta ;
               	0,         	0,       	1,          0             ]

    B_long = [ 	deriv.x_eta,    deriv.x_tau ;
                deriv.z_eta,    deriv.z_tau ;
                deriv.m_eta,    deriv.m_tau ;
                0,              0           ]

    C_long =  eye(4)

    D_long = zeros(size(C_long,1),size(B_long,2));

    sys_long = ss(A_long,B_long,C_long,D_long, ...
        'StateName', {'u'; 'w'; 'q'; 'theta'}, ...
        'InputName', {'elevator'; 'throttle'}, ...
        'OutputName', {'u'; 'w'; 'pitch rate'; 'pitch angle'})
end

H_long = tf(sys_long)

poles_long = pole(H_long)
fig1 = figure(1);
step(H_long,50)

% check observability/controlability
if size(A_long,1)==rank(ctrb(A_long,B_long)) 
    disp('The LTI system is controllable')
else
    warning('The LTI system is not controllable!')
end

if size(A_long,1)==rank(obsv(A_long,C_long))
        disp('The LTI system is observable')
else
    warning('The LTI system is not observable!')
end

num_long = H_long.Numerator;
den_long = H_long.Denominator;

disp('======================= Lateral EOM ==============================')
% ------------------------------------------------------------------------
%%% Lateral
    A_lat = [	deriv.y_v,	deriv.y_p,	deriv.y_r,	deriv.y_phi,    deriv.y_psi ;
                deriv.l_v,	deriv.l_p,	deriv.l_r,	deriv.l_phi,    deriv.l_psi ;
                deriv.n_v,	deriv.n_p,	deriv.n_r,	deriv.n_phi,    deriv.n_psi ;
                0,         	1,          0,          0,              0           ;
                0,         	0,          1,          0,              0           ]

    B_lat = [	deriv.y_xi,	deriv.y_zeta ;
                deriv.l_xi,	deriv.l_zeta ;
                deriv.n_xi,	deriv.n_zeta ;
                0,         	0            ;
                0,         	0            ]

    C_lat =  eye(5)

    D_lat = zeros(size(C_lat,1),size(B_lat,2));

    sys_lat = ss(A_lat,B_lat,C_lat,D_lat, ...
        'StateName', {'v'; 'p'; 'r'; 'phi'; 'psi'}, ...
        'InputName', {'aerilons'; 'rudder'}, ...
        'OutputName', {'v'; 'roll rate'; 'yaw rate'; 'roll angle'; 'yaw angle'})

H_lat = tf(sys_lat)

poles_lat = pole(H_lat)
fig2 = figure(2);
step(H_lat,50)

% check observability/controlability
if size(A_lat,1)==rank(ctrb(A_lat,B_lat)) 
    disp('The LTI system is controllable')
else
    warning('The LTI system is not controllable!')
end

if size(A_lat,1)==rank(obsv(A_lat,C_lat))
        disp('The LTI system is observable')
else
    warning('The LTI system is not observable!')
end

num_lat = H_lat.Numerator;
den_lat = H_lat.Denominator;
