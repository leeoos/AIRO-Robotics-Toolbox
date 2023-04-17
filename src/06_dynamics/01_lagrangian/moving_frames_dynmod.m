%% Sopravvivenza_AIRO! 
% Authors: Massimo, Leonardo, Paolo, Francesco
% Implementation of moving frames algorithm to compute
% the symbolic dynamic model of a general robot.

clc 
close all
clear all

my_path = getenv("ROB2LIB_PATH");
addpath(my_path);
rob2fun = rob2lib();

%% INPUTS 
% PAY ATTENTION: update for each problem!

syms L1 L2 L3 q1 q2 q3 g0

g = [
    0;
    g0;
    0;
    
]

% Direct kinematics.
DHTABLE = [        
    0    L1     0   q1;
    pi/2 0      0   q2;
    0    L3     0   q3;
];
N = size(DHTABLE, 1); % number of joints

syms d1 d2 d3 V F D C E
% Vectro of the centers of m
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

% Initial conditions for linear velocity
initial_velocity = [
    0;
    0;
    0;
];
VELOCITY = cell(1, N);
VELOCITY{1} = initial_velocity;

% DH parameters
dh_param = rob2fun.compute_dir_kin(DHTABLE);
A = dh_param{1}; % cell array of chain transformations

% Extraction of CoM vectors
W_CoM = cell(1,N);
A_i = eye(4);

for i = (1:N)
    A_i = A_i*A{i};
    A_i(1:3,:)
    W_CoM{i} = A_i * [RCoM{i};1];
end

% Initialization of symbolics vectors
syms q [1 N]
syms q_dot [1 N]
syms m [1 N]
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

%% END OF INPUTS

% Inizialization of Kinetic Energy
% KINETIC_ENERGY = m(1) * ((transpose(VELOCITY{1})*VELOCITY{1}) + ... 
%                  (transpose(OMEGA{1})*I{1}*OMEGA{1}));
KINETIC_ENERGY = 0;

% Computaion of linear and angular velocity of each link,
% velocity of CoM and kinetic energy of each link, with 
% moving frames algorithm
for i = (1 : N)
    R_i = A{i}(1:3, 1:3); % Rotation matrix
    im1_r_im1_i = A{i}(1:3, 4);
    z = [0;0;1];

    % Angular velocity
    im1_omega_i = (OMEGA{i} + (1-sigma(i))*q_dot(i)*z);
    i_omega_i = transpose(R_i)*im1_omega_i;
    OMEGA{i+1} = simplify(i_omega_i);
  

    % Linear velocity
    velocity_new = transpose(R_i) * ...
                   (VELOCITY{i} + (sigma(i)*q_dot(i)*z) + cross(im1_omega_i,im1_r_im1_i));
    VELOCITY{i+1} = simplify(velocity_new);  
    

    % Velocity of centers of m
    velocity_CoM = simplify(VELOCITY{i+1} + cross(OMEGA{i+1}, RCoM{i}));

    KINETIC_ENERGY = KINETIC_ENERGY + (...
                     (1/2*m(i)*transpose(velocity_CoM)*velocity_CoM) + ... 
                     (1/2*transpose(OMEGA{i+1})*I{i}*OMEGA{i+1})...
                    );
    disp(["Step ", i])
    KINETIC_ENERGY = simplify(KINETIC_ENERGY)
    
end

KINETIC_ENERGY = simplify(expand(KINETIC_ENERGY))

%% EXPERIMENTAL
aliases = sym("alias",[1 N]);
q_dot_squared = [sym("foo")];
for i = (1:N)
    q_dot_squared(i) = str2sym("q_dot"+string(i)+"^2");
end
q_dot_squared;
KIN_alias = subs(KINETIC_ENERGY,q_dot_squared,aliases);

reduced_q_dot = [sym("foo")];
M = [sym("bar")];
for r = (1:N)
    for c = (1:N)
        if (r == c)
            reduced_alias = aliases(aliases~=aliases(r));
            KIN_q_dot_squared = subs(KIN_alias, q_dot, zeros(1,N));
            KIN_qr_dot_squared = subs(KIN_q_dot_squared, reduced_alias, zeros(1,N-1));
            M(r,c) = simplify(2*subs(KIN_qr_dot_squared, aliases(r), 1));
        else
            reduced_q_dot = q_dot;
            reduced_q_dot(reduced_q_dot == q_dot(c)) = [];
            reduced_q_dot(reduced_q_dot == q_dot(r)) = [];
            KIN_q_dot = subs(KIN_alias, aliases, zeros(1,N));
            if not(isempty(reduced_q_dot))
                KIN_q_dot = subs(KIN_q_dot, reduced_q_dot, zeros(1,N-2));
            end
            M(r,c) = subs(KIN_q_dot, {q_dot(r),q_dot(c)}, {1,1});
        end
    end
end
%% EXPERIMENTAL

syms foo % tmp symbol
M_q = [foo];
for i = (1:N)
    KINETIC_ENERGY = collect(KINETIC_ENERGY, q_dot(i)^2);
end

for r = (1:N)
    for c = (1:N)
        if ((r == c))
            M_q(r,c) = simplify(diff(KINETIC_ENERGY, q_dot(r), 2));
        else
            K_reduced_qr = simplify(diff(KINETIC_ENERGY, q_dot(c)));
            K_reduced_qrc = simplify(diff(K_reduced_qr, q_dot(r)));
            M_q(r,c) = simplify(K_reduced_qrc);
        end
    end
end
M_q
disp(["Are M_experimental and M_q equal?", isequal(M, M_q)])

% Computation of coriolis and centrifugal term
COR_CENT = []; 

for i = (1:N)
    dMi_dq = jacobian(M_q(:,i), q);
    dM_dqi = diff(M_q, q(i))
    Ci = 1/2*(dMi_dq + transpose(dMi_dq) - dM_dqi);
    ci = simplify(q_dot*Ci*transpose(q_dot));
    COR_CENT = [c;ci];
end
COR_CENT

% Computation Potential energy 
U = 0;
for i = (1:N)
    U_i = simplify(- m(i)*(transpose(g))*W_CoM{i}(1:3));
    U = simplify(U + U_i);
end
U;

% Computation of gravity term g(q)
g_q = simplify(expand(gradient(U, transpose(q))));



