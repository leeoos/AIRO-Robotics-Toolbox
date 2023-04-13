%% Sopravvivenza_AIRO! 
% Authors: Massimo, Leonardo, Paolo, Francesco
% Implementation of Newton-Euler method for numerical 
% computation of robots dynamic model

clc 
close all
clear all

my_path = getenv("ROB2LIB_PATH");
addpath(my_path);
FunObj = Rob2Lib();


%% INPUTS FOR THE PROBLEM
% PAY ATTENTION: update for each problem!

syms L1 L2 L3 q1 q2 q3 

% Direct kinematics.
DHTABLE = [        
    0    L1     0   q1;
    pi/2 0      0   q2;
    0    L3     0   q3;
];
N = size(DHTABLE, 1); % number of joints

syms d1 d2 d3 V F D C E
% Vectro of the centers of masses
RCoM = {[-L1+d1;0;0], [0;-L2+d2;0], [-L3+d3;0;0]};

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
initial_acceleration = [
    0;
    0;
    0;
];
ACCELERATION = cell(1, N);
ACCELERATION{1} = initial_velocity;

% Desired values for q, q_dot, q_d_dot
q_eval = [pi,0,pi/2]
q_dot = [3;3;3]
q_d_dot = [2;2;2]

%% END OF INPUTS

pose = FunObj.compute_dir_kin(DHTABLE);
A = pose{1}; % cell array of chain transformations

% Initialization of symbolics vectors
q = (sym('q',[1 N]));
%q_dot = (sym('q_dot',[1 N]));
%q_d_dot = (sym('q_d_dot',[1 N]));
masses = (sym('m',[1 N]));

% Inertia Matricies (Evaluate if you have numbers!)
Ixxsymb = (sym('Ixx',[1 N]));
Iyysymb = (sym('Iyy',[1 N]));
Izzsymb = (sym('Izz',[1 N]));
Ixysymb = (sym('Ixy',[1 N]));
Iyxsymb = (sym('Iyx',[1 N]));
Izxsymb = (sym('Izx',[1 N]));
Ixzsymb = (sym('Ixz',[1 N]));
Iyzsymb = (sym('Iyz',[1 N]));
Izysymb = (sym('Izy',[1 N]));
I = cell(1,3);
for i =(1:N)
    I{i} = [
        Ixxsymb(i), Ixysymb(i), Ixzsymb(i);
        Ixysymb(i), Iyysymb(i), Iyzsymb(i);
        Ixzsymb(i), Iyzsymb(i), Izzsymb(i);
    ];
end


% Computaion of linear and angular velocity of each link,
% and velocity of CoM.
for i = (1 : N)
    R_i = A{i}(1:3, 1:3); % Rotation matrix
    R_i = subs(R_i, q, q_eval);
    r_i = A{i}(1:3, 4);
    z = [0;0;1];

    % Angular velocity
    im1_omega_i = (OMEGA{i} + (1-sigma(i))*q_dot(i)*z);
    i_omega_i = transpose(R_i)*im1_omega_i;
    OMEGA{i+1} = simplify(i_omega_i);

    % Angular acceleration
    rotational_part = (1-sigma(i))* (q_d_dot(i)*z + q_dot(i)*cross(OMEGA{i},z));
    i_omega_dot_i = transpose(R_i)*(OMEGA_DOT{i} + rotational_part);                       
    OMEGA_DOT{i+1} = simplify(i_omega_dot_i);
  
    % Linear acceleration
      
end

OMEGA{1}
OMEGA{2}
OMEGA{3}

OMEGA_DOT{1}
OMEGA_DOT{2}
OMEGA_DOT{3}


% Function to compute the derivative of a rotation matrix
function R_dot = diff_rotation_matrix(R, w)
    syms wx wy wz
    w_sym = [wx,wy,wz];
    
    Sw = [  
            0, -wz, wy; 
            wz, 0, -wx; 
            -wy, wx,  0
    ];
    
    Sw = subs(Sw, w_sym, w);
    R_dot = Sw * R;
end
