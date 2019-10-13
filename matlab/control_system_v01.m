%% ELEN4011 Engineering Design
% Control system for UAV
% Tyson Cross       1239448

% clc; clear all; close all;

%% Data

%%% Environment properties
env.rho = 1.225;                % Air Density (at Sea Level, STP) [kg/m^3]
env.g = 9.81;                   % Gravity [m/s^2]
env.V_0 = 20;                   % Speed [m/s]
env.h = 100;                    % Height [m]

%%% ARF60 geometric properties
geometry.m = 3.5105;            % Weight (fixed mass w/electric motor) [kg]
geometry.b = 1.87;              % Wing Span [m]
geometry.s = 0.6069;            % Wing Area [m^2]
geometry.c = 0.324;             % Chord (mean aerodynamic) [m]
geometry.cg = [0;0;0];          % Centre of gravity (x,y,z)
inertia.Ixx = 0.1996;           % Inertial Moment (roll) [kg.m^2]
inertia.Iyy = 0.24086;          % Inertial Moment (pitch) [kg.m^2]
inertia.Izz = 0.396;            % Inertial Moment (yaw) [kg.m^2]

% geometry.gamma = 6;            % Wing angle (dihedral) [deg]
% geometry.lamda = 1;            % Wing angle (taper) [deg]
% geometry.s_a = 0.0581;         % Ailerons Surface Area [m^2]
% geometry.a_a = 0.4277;         % Ailerons Around Area [m^2]
% geometry.b_t = 0.6700;         % Horizontal Tail Span [m]
% geometry.C_t = 0.1950;         % Horizontal Tail Chord (mean aerodynamic) [m]
% geometry.S_t = 0.1307;         % Horizontal Tail Area [m^2]
% geometry.V_h = 0.5457;         % Horizontal Tail Volume Ratio
% geometry.S_e = 0.0232;         % Elevator Surface Area [m^2]
% geometry.l_t_h = 0.8200;       % CG to Horizontal Tail 1/4 Chord (distance) [m]
% geometry.b_v = 0.2350;         % Vertical Tail Span [m]
% geometry.C_v = 0.2150;         % Vertical Tail Chord (mean aerodynamic) [m]
% geometry.S_v = 0.0505;         % Vertical Tail Area [m^2]
% geometry.V_v = 0.2162;         % Vertical Tail Volume Ratio
% geometry.S_r = 0.1307;         % Rudder Surface Area [m^2]
% geometry.l_t_v = 0.8400;       % CG to Vertical Tail 1/4 Chord (distance) [m]

%%% Stability derivative coefficients (
% Lift
coeff.C_L_0 = 0.4100;           % Zero alpha lift 
coeff.C_L_alpha = 4.3842;     	% Alpha derivative
coeff.C_L_alpha_t = 3.6369;  	% Alpha derivative (tail)
coeff.C_L_alpha_v = 0.9114;    	% Alpha derivative (vertical tail)
coeff.C_L_delta_e = 0.3059;    	% Pitch control (elevator) derivative
coeff.C_L_alpha_dot = 0;       	% Alpha-dot derivative
coeff.C_L_q = 0.6431;          	% Pitch rate derivative

% Drag
coeff.C_D_0 = 0.0500;         	% Zero-lift drag
coeff.C_D_delta_e = 0;        	% Elevator Control derivative (pitch)
coeff.C_D_delta_a = 0;        	% Aerilon Control derivative (roll)
coeff.C_D_delta_r = 0;         	% Rudder Control derivative (yaw)

% Sideslip
coeff.C_Y_beta = -0.1795;     	% Sideslip derivative
coeff.C_Y_delta_a = 0;        	% Roll control derivative (airelons)
coeff.C_Y_delta_r = 0.1132;    	% Yaw control derivative (rudder)
coeff.C_Y_q = 0;               	% Roll rate derivative
coeff.C_Y_r = 0;               	% Yaw rate derivative

% Rolling moments
coeff.C_l_beta = -0.0013;     	% Sideslip derivative
coeff.C_l_delta_a = 0.8275;   	% Roll controll derivative
coeff.C_l_delta_r = 0.0036;   	% Yaw control derivative
coeff.C_l_p = -1.8267;        	% Roll rate derivative
coeff.C_L_r = 0.0052;         	% Yaw rate derivative

% Pitching moments
coeff.C_m_0 = 0;              	% Zero alpha pitch
coeff.C_m_alpha = 0;          	% Alpha derivative
coeff.C_m_alpha_dot = -6.0730; 	% Alpha-dot derivative
coeff.C_m_delta_e = -0.7741;   	% Pitch control derivative
coeff.C_m_q = -10.0467;        	% Pitch rate derivative

% Yaw moments
coeff.C_n_beta = 0.4653;       	% Sideslip derivative
coeff.C_n_delta_a = -0.1357;   	% Roll controll derivative
coeff.C_n_delta_r = -0.2934;   	% Yaw control derivative
coeff.C_n_p = -1.8267;         	% Roll rate derivative
coeff.C_n_r = -0.2934;        	% Yaw rate derivative

%%% Dimensionless derivatives (with ref to Body axes)
%Longitudinal
deriv.x_u = -0.2289;
deriv.x_w = 0.3712;
deriv.x_q = 0;
deriv.x_theta = -9.81;
deriv.x_eta = 0;
deriv.x_tau = 51.5;

deriv.z_u = -1.8772;
deriv.z_w = -10.1512;
deriv.z_q = 20;
deriv.z_theta = 0;
deriv.z_eta = -14.0042;
deriv.z_tau = 0;

deriv.m_u = 0.9219;
deriv.m_w = -7.0403;
deriv.m_q = -26.072;
deriv.m_theta = 0;
deriv.m_eta = -147.6913;
deriv.m_tau = 0;

%Lateral
% deriv.y_v = 
% deriv.y_p =
% deriv.y_r =
% deriv.y_phi =
% deriv.y_psi =
% deriv.y_xi =
% deriv.y_zeta = 
% 
% deriv.l_v = 
% deriv.l_p =
% deriv.l_r =
% deriv.l_phi =
% deriv.l_psi =
% deriv.l_xi =
% deriv.l_zeta = 
% 
% deriv.n_v = 
% deriv.n_p =
% deriv.n_r =
% deriv.n_phi =
% deriv.n_psi =
% deriv.n_xi =
% deriv.n_zeta = 

% Initial conditions (stable, trimmed flight)

    
% syms u w q theta;           % state variables: perturbance along x, z, pitch rate, pitch angle
% syms eta tau;               % control variables: peturbance in elevator angle, thrust
% x = [ u; w; q; theta ]
% u = [ eta; tau ]
% y = eye(4)*x

%% State Space equations of motion (small peturbations)
augmented = false;
if ~augmented
    A = [ deriv.x_u,    deriv.x_w,	deriv.x_q,	deriv.x_theta;
          deriv.z_u,	deriv.z_w,	deriv.z_q,	deriv.z_theta;
          deriv.m_u,    deriv.m_w,	deriv.m_q,	deriv.m_theta;
          0,            0,          1,          0            ]

    B = [   deriv.x_eta,    deriv.x_tau     ;
            deriv.z_eta,    deriv.z_tau     ;
            deriv.m_eta,    deriv.m_tau     ;
            0,              0               ]

    C =  [0 0 1 0] % output is pitch rate

    D = 0; % zeros(4,2);

    sys = ss(A,B,C,D,'StateName', {'u'; 'w'; 'q'; 'theta'}, 'InputName', {'eta';'epsilon'})
else
    % Augmented with height
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

    C =  [0 0 1 0 0]

    D = 0;

    sys = ss(A,B,C,D,'StateName', {'u'; 'w'; 'q'; 'theta'; 'h'}, 'InputName', {'eta';'epsilon'})
end

H = tf(sys)

% Poles of elevator to pitch TF
poles = pole(H(1))
rlocus(H(1))
pzmap(H(1))
step(H)
num = cell2mat(H(1).Numerator);
den = cell2mat(H(1).Denominator);