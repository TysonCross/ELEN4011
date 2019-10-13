%% ELEN4011 Engineering Design
% Control system for UAV
% Tyson Cross       1239448

clc; clear all; close all;

system_setup;

%% State Space equations of motion (small peturbations)
height_augmented = false;

if height_augmented
    A = [ deriv.x_u,    deriv.x_w,	deriv.x_q,	deriv.x_theta,  0 ;
          deriv.z_u,	deriv.z_w,	deriv.z_q,	deriv.z_theta,  0 ;
          deriv.m_u,    deriv.m_w,	deriv.m_q,	deriv.m_theta,  0 ;
          0,            0,          1,          0,              0 ;
          0,            -1,         0,          env.V_0,        0 ]

    B = [   deriv.x_eta,    deriv.x_tau     ;
            deriv.z_eta,    deriv.z_tau     ;
            deriv.m_eta,    deriv.m_tau     ;
            0,              0               ;
            0,              0               ]

    C =  eye(5)

    D = zeros(size(C,1),size(B,2));

    sys = ss(A,B,C,D,'StateName', {'u'; 'w'; 'q'; 'theta'; 'h'}, ...
        'InputName', {'elevator'; 'throttle'}, ...
        'OutputName', {'x'; 'z'; 'pitch rate'; 'pitch angle'; 'height'})
else
    A = [ deriv.x_u,    deriv.x_w,	deriv.x_q,	deriv.x_theta;
          deriv.z_u,	deriv.z_w,	deriv.z_q,	deriv.z_theta;
          deriv.m_u,    deriv.m_w,	deriv.m_q,	deriv.m_theta;
          0,            0,          1,          0            ]

    B = [   deriv.x_eta,    deriv.x_tau     ;
            deriv.z_eta,    deriv.z_tau     ;
            deriv.m_eta,    deriv.m_tau     ;
            0,              0               ]

    C =  eye(4)

    D = zeros(size(C,1),size(B,2));

    sys = ss(A,B,C,D,'StateName', {'u'; 'w'; 'q'; 'theta'}, ...
        'InputName', {'elevator'; 'throttle'}, ...
        'OutputName', {'u'; 'w'; 'pitch rate'; 'pitch angle'})
end

H = tf(sys)

poles = pole(H)
step(H,50)

% check observability/controlability
if size(A,1)==rank(ctrb(A,B)) 
    disp('The LTI system is controllable')
else
    warning('The LTI system is not controllable!')
end

if size(A,1)==rank(obsv(A,C))
        disp('The LTI system is observable')
else
    warning('The LTI system is not observable!')
end

num = H.Numerator;
den = H.Denominator;