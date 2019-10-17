%% Data
%%% Environment properties
env.rho = 1.225;                % Air Density (at Sea Level, STP) [kg/m^3]
env.g = 9.81;                   % Gravity [m/s^2]
env.V_0 = 40/3.6;               % Speed [m/s]
env.h = 100;                    % Height [m]

%%% UltraStick 25e geometric properties
geometry.m = 1.943;            	% Weight (fixed mass w/electric motor) [kg]
geometry.b = 1.27;              % Wing Span [m]
geometry.s = 0.3097;          	% Wing Area [m^2]
geometry.c = 0.25;              % Chord (mean aerodynamic) [m]
geometry.cg = [0;0;0];          % Centre of gravity (x,y,z)
geometry.inertia.Ix = 0.0895; 	% Inertial Moment (x) [kg.m^2]
geometry.inertia.Iy = 0.1444;	% Inertial Moment (y) [kg.m^2]
geometry.inertia.Iz = 0.1620;  	% Inertial Moment (x) [kg.m^2]
geometry.inertia.Ixz = 0.0140; 	% Inertial Moment (planar) [kg.m^2]

%%% Stability derivative coefficients 
% Dimensionless derivatives (with ref to Body axes)
% Longitudinal
deriv.x_u = -0.1492;
deriv.x_w = 0.1490;
deriv.x_q = 0;
deriv.x_theta = -env.g;
deriv.x_h = 0;
deriv.x_eta = 8.4872;
deriv.x_tau = 0;

deriv.z_u = -0.5272;
deriv.z_w = -5.2988;
deriv.z_q = 11.1100;
deriv.z_theta = 0;
deriv.z_h = 0;
deriv.z_eta = 0;
deriv.z_tau = -1.6553;

deriv.m_u = 2.6669;
deriv.m_w = -3.3818;
deriv.m_q = -32.9054;
deriv.m_theta = 0;
deriv.m_h = 0;
deriv.m_eta = 0.2546;
deriv.m_tau = -49.7923;

deriv.th_u = -0.523;
deriv.th_w = -0.9986;
deriv.th_q = 0;

%Lateral
deriv.y_v = -0.9512;
deriv.y_p = 0;
deriv.y_r = -1;
deriv.y_phi = 0.8830;
deriv.y_psi = 0; 
deriv.y_xi = 0;
deriv.y_zeta = 0.2189;

deriv.l_v = -12.0240;
deriv.l_p = -7.4665;
deriv.l_r = 5.8687;
deriv.l_phi = 0;
deriv.l_psi = 0; 
deriv.l_xi = 21.6477;
deriv.l_zeta = 4.4873;

deriv.n_v = 5.0421;
deriv.n_p = -1.9428;
deriv.n_r = -6.6032;
deriv.n_phi = 0;
deriv.n_psi = 0;
deriv.n_xi = -0.2506;
deriv.n_zeta = -5.7111;

% Initial conditions (stable, trimmed flight)

    
% syms u w q theta;           % state variables: perturbance along x, z, pitch rate, pitch angle
% syms eta tau;               % control variables: peturbance in elevator angle, thrust
% x = [ u; w; q; theta ]
% u = [ eta; tau ]
% y = eye(4)*x