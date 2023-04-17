%% Sopravvivenza_AIRO! 
% Authors: Massimo, Leonardo, Paolo, Francesco
% Implementation of Newton-Euler method for numerical 
% computation of robots dynamic model

clc 
close all
clear all

my_path = getenv("ROB2LIB_PATH");
addpath(my_path);
rob2fun = rob2lib();

% Toogle output display
enable_output = true

%% INPUT
% PAY ATTENTION: update for each problem!

N = 3; % number of joints

% Standard symbolic variables
syms q [1 N]
syms q_dot [1 N]
syms q_d_dot [1 N]
syms m [1 N]
syms L [1 N]
syms d [1 N]
syms g0

% Conversion to colum vectros
q = transpose(q);
q_dot = transpose(q_dot);
q_d_dot = transpose(q_d_dot);
m = transpose(m);
L = transpose(L);
d = transpose(d);

% Add the necessary symbols for the problem
syms V F D C E

% Direct kinematics.
DHTABLE = [        
    0    L1     0   q1;
    0    L2     0   q2;
    0    L3     0   q3;
];

% Rotation matrix from last frame to end effector
n_R_e = eye(N);

% Vectro of the centers of masses
R_CoM = {[-L1+d1;0;0], [-L2+d2;0;0], [-L3+d3;0;0]};

% sigma vector:
% 0: revolout 
% 1: prismatic
sigma = [
    0;
    0;
    0;
];

% Initial conditions for angular velocity
initial_omega = [
    0;
    0;
    0;
];
OMEGA = cell(1, N);
OMEGA{1} = initial_omega;

% Initial conditions for angular acceleration
initial_omega_dot = [
    0;
    0;
    0;
];
OMEGA_DOT = cell(1, N);
OMEGA_DOT{1} = initial_omega_dot;

% Initial conditions for linear velocity
initial_velocity = [
    0;
    0;
    0;
];
VELOCITY = cell(1, N);
VELOCITY{1} = initial_velocity;

% Initial conditions for linear acceleration
% Mind the position of gravity
initial_acceleration = [
    g0;
    0;
    0;
];
ACCELERATION = cell(1, N);
ACCELERATION{1} = initial_acceleration;

% Center of mass acceleration
CoM_ACCELERATION = cell(1,N);

% Initial conditions for linear acceleration
external_forces = [
    0;
    0;
    0;
];
FORCES = cell(1, N+1);
FORCES{N+1} = external_forces;

% Initial conditions for linear acceleration
external_torques = [
    0;
    0;
    0;
];
TORQUES = cell(1, N+1);
TORQUES{N+1} = external_torques;

% Generalized forces
GEN_FORCES = cell(1,N);

% Viscous frictions (update with given values)
V_FRICTIONS = cell(1,N);
V_FRICTIONS(:) = {0}; %{zeros(3,1)};

% Input values for q, q_dot, q_d_dot --> NE(q, q_dot, q_d_dot)
q_in = [
    pi;
    0;
    pi/2;
];

q_dot_in = [
    0;
    0;
    0;
];

q_d_dot_in = [
    0;
    0;
    0;
];

% Extraction of DH parametrs
dh_param = rob2fun.compute_dir_kin(DHTABLE);
A = dh_param{1}; % cell array of chain transformations

% Inertia Matricies (Evaluate if you have numbers!)
% X
syms Ixx [1 N]
syms Ixy [1 N]
syms Ixz [1 N]
% Y
syms Iyx [1 N]
syms Iyy [1 N]
syms Iyz [1 N]
% Z
syms Izz [1 N]
syms Izx [1 N]
syms Izy [1 N]

I = cell(1,3);
for i =(1:N)
    I{i} = [
        Ixx(i), Ixy(i), Ixz(i);
        Ixy(i), Iyy(i), Iyz(i);
        Ixz(i), Iyz(i), Izz(i);
    ];
end
%celldisp(I) % uncomment for debug

%% END OF INPUTS


% Convert symbolic values to numerical values. 
% Comment this part for symbolic computation
% DHTABLE = subs(DHTABLE, q, q_in);
% q = subs(q, q, q_in);
q_dot = subs(q_dot, q_dot, q_dot_in);
q_d_dot = subs(q_d_dot, q_d_dot, q_dot_in);
% L = subs(L, L, L_in)
% d = subs(d, d, d_in)
% m = subs(m, m, m_in)

% Forward step: computaion of linear and angular velocity 
% of each link and velocity of CoM. 
% Mind the gap: 
% i     ---> is the PREVIOUS STEP (exept for CoM Acceleration)
% i+1   ---> is the CURRENT STEP
for i = (1 : N)
    R_i = A{i}(1:3, 1:3); % Rotation matrix
    R_i_T = transpose(R_i);
    im1_r_im1_i = A{i}(1:3, 4);
    i_r_im1_i = R_i_T * im1_r_im1_i;
    z = [0;0;1];

    % Angular velocity
    im1_omega_i = (OMEGA{i} + (1-sigma(i))*q_dot(i)*z);
    i_omega_i = R_i_T * im1_omega_i;
    OMEGA{i+1} = simplify(i_omega_i);

    % Angular acceleration
    rotational_part = (1-sigma(i))*(q_d_dot(i)*z + q_dot(i)*cross(OMEGA{i},z));
    i_omega_dot_i = R_i_T * (OMEGA_DOT{i} + rotational_part);                       
    OMEGA_DOT{i+1} = simplify(i_omega_dot_i);
  
    % Linear acceleration
    ACCELERATION{i+1} = (R_i_T*ACCELERATION{i}) + ...
                        + sigma(i)*(q_d_dot(i)*z + cross(2*q_dot(i)*OMEGA{i+1},z)) +...
                        + cross(OMEGA_DOT{i+1},i_r_im1_i) + ...
                        + cross(OMEGA{i+1}, cross(OMEGA{i+1}, i_r_im1_i));
    ACCELERATION{i+1} = simplify(ACCELERATION{i+1});

    % CoM acceleration
    CoM_ACCELERATION{i} = ACCELERATION{i+1} + cross(OMEGA_DOT{i+1}, R_CoM{i}) + ...
                          + cross(OMEGA{i+1}, cross(OMEGA{i}, R_CoM{i}));
    CoM_ACCELERATION{i} = simplify(CoM_ACCELERATION{i});
end

% uncomment for debug
% celldisp(OMEGA)
% celldisp(OMEGA_DOT)
% celldisp(ACCELERATION)
% celldisp(CoM_ACCELERATION)

% Output display
% if enable_output
%     for i = (i:N)
%         fprintf("OMEGA "+ i-1)
%     end
% end 


% Backward step
for i = (N:-1:1)
    if (i == N)
        R_i = n_R_e;
    else
        R_i = A{i}(1:3, 1:3); % Rotation matrix
    end
    im1_r_im1_i = A{i}(1:3, 4);
    i_r_im1_i = R_i_T * im1_r_im1_i;
    
    % Linear Force
    FORCES{i} = R_i*FORCES{i+1} + m(i)*(CoM_ACCELERATION{i});
    FORCES{i} = simplify(FORCES{i});

    % Torques
    TORQUES{i} = R_i*TORQUES{i+1} + cross(R_i*FORCES{i+1}, R_CoM{i}) + ...
                 - cross(FORCES{i}, (i_r_im1_i + R_CoM{i})) + ... 
                 + I{i}*OMEGA_DOT{i} + cross(OMEGA{i}, (I{i}*OMEGA{i}));
    TORQUES{i} = simplify(TORQUES{i});
    
    % Generalized forces
    if (sigma(i) == 0) % revolut case
        GEN_FORCES{i} = transpose(TORQUES{i}) * z + V_FRICTIONS{i}*q_dot(i);
    else % (sigma(i) == 1
        GEN_FORCES{i} = transpose(FORCES{i}) * z + V_FRICTIONS{i}*q_dot(i);
    end
    GEN_FORCES{i} = simplify(GEN_FORCES{i});
end

% uncomment for debug
celldisp(FORCES)
celldisp(TORQUES)
celldisp(GEN_FORCES)

fprintf("CVD (not SVD,neighter DLS) \n")
