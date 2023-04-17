%% Sopravvivenza_AIRO! 
% Authors: Massimo, Leonardo, Paolo, Francesco
% Implementation of Koning Theorem to compute the 
% dynamic model of a generic planar robot,
% given the pose of CoM and angular velocities.

clc 
close all
clear all

my_path = getenv("ROB2LIB_PATH");
addpath(my_path);
rob2fun = rob2lib();

%% INPUTS
% PAY ATTENTION: update for each problem!

N = 4; % number of joints

% Standard symbolic variables
syms q [1 N]
syms q_dot [1 N]
syms q_d_dot [1 N]
syms m [1 N]
syms L [1 N]
syms d [1 N]
syms t g0

% Definition of gravity vector
% Mind the position of gravity
g = [
    0;
    g0;
    0;
];

% Inertia Matricies (Evaluate if you have numbers!)
% Set this value to false if you want to use a full inertia matrix
diagonal_inertia = true

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
if diagonal_inertia
    for i =(1:N)
        I{i} = [
            Ixx(i), 0, 0;
            0, Iyy(i), 0;
            0, 0, Izz(i);
        ];
    end
else 
    for i =(1:N)
        I{i} = [
            Ixx(i), Ixy(i), Ixz(i);
            Ixy(i), Iyy(i), Iyz(i);
            Ixz(i), Iyz(i), Izz(i);
        ];
    end
end
%celldisp(I) % uncomment for debug

% PAY ATTENTION: this part require a precomputation that depends on the problem
% Vectors of the centers of masses w.r.t world frame
W_CoM = {
    [q1-d1; 0; 0], ...
    [q1; q2-d2; 0], ...
    [q1 + d3*cos(q3);
     q2 + d3*sin(q3);
     0
    ], ...
    [q1 + L3*cos(q3) + d4*cos(q3+q4);
     q2 + L3*sin(q3) + d4*sin(q3+q4);
     0
    ], ...
};
% uncomment for debug
celldisp(W_CoM)

% Velocities of the centers of masses w.r.t world frame
CoM_VELOCITY = {
    [q_dot1; 0; 0], ... 
    [q_dot1; q_dot2; 0], ...
    [q_dot1 - d3*sin(q3)*q_dot3; 
     q_dot2 + d3*cos(q3)*q_dot3; 
     0], ...
    [q_dot1 - L3*sin(q3)*q_dot3 - d4*sin(q3+q4)*(q_dot3+q_dot4); 
     q_dot2 + L3*cos(q3)*q_dot3 + d4*cos(q3+q4)*(q_dot3+q_dot4); 
     0], ...
};
% uncomment for debug
celldisp(CoM_VELOCITY)

% % Angular velocities of each link
OMEGA = {
    [0; 0; 0], ... 
    [0; 0; 0], ...
    [0; 0; q_dot3], ...
    [0; 0; q_dot3 + q_dot4], ...
};
% uncomment for debug
celldisp(OMEGA)

%% END OF INPUTS

% Implementation of Koening theorem for each link
KINETIC_ENERGY = cell(1, N);
for i = (1:N) 
     KINETIC_ENERGY{i} = (1/2*m(i)*transpose(CoM_VELOCITY{i})*CoM_VELOCITY{i} + ...
         + (1/2*transpose(OMEGA{i})*I{i}*OMEGA{i}));
     KINETIC_ENERGY{i} = collect(simplify(expand(KINETIC_ENERGY{i})),q_dot.^2); %[q_dot1^2, q_dot2^2, q_dot3^2, q_dot4^2]);
     KINETIC_ENERGY{i} = collect(KINETIC_ENERGY{i},m);
end
%celldisp(KINETIC_ENERGY) % uncomment this for debug

KINETIC_ENERGY = sum([KINETIC_ENERGY{:}])

% Extraction M(q) elements
M_q = rob2fun.get_inertia_matrix(KINETIC_ENERGY, N);
M_q

% Computation of coriolis and centrifugal term
c_q_q_dot = []; 

for i = (1:N)
    dMi_dq = jacobian(M_q(:,i), q);
    dM_dqi = diff(M_q, q(i));
    Ci = 1/2*(dMi_dq + transpose(dMi_dq) - dM_dqi);
    ci = simplify(q_dot*Ci*transpose(q_dot));
    c_q_q_dot = [c_q_q_dot;ci];
end
c_q_q_dot

% Computation Potential energy 
U = 0;
for i = (1:N)
    U_i = simplify(- m(i)*(transpose(g))*W_CoM{i}(1:3));
    U = simplify(U + U_i);
end
U;

% Computation of gravity term g(q)
g_q = simplify(expand(gradient(U, transpose(q))))


%% EXPERIMENTAL
% Extraction of dynamics coefficents

% Building subs array
% subs_array = [];
% for i = (1:N)
%     cos_q = [cos(q1), cos(q2), cos(q1+q2), cos(q2+q3)]; %cos(sum(q(1:i)));
%     sin_q = [sin(1), sin(2)]; %sin(sum(q(1:i)));
%     subs_array = [subs_array, cos_q, sin_q];
% end
% subs_values = ones(1,size(subs_array,2));


DYN_COEFF = M_q
DYN_COEFF = subs(DYN_COEFF, g0, 1);
DYN_COEFF = subs(DYN_COEFF, q, ones(1,size(q,2)));
DYN_COEFF = subs(DYN_COEFF, q_dot, ones(1,size(q_dot,2)));
DYN_COEFF = subs(DYN_COEFF, q_d_dot, ones(1,size(q_d_dot,2)));
DYN_COEFF = subs(DYN_COEFF, L, ones(1,size(L,2)));
%DYN_COEFF{i} = subs(DYN_COEFF{i}, subs_array, subs_values);

syms foo
dynamic_coeff = [];

for r = (1:N)
    for c = (r:N)
        if not(DYN_COEFF(r,c) == 0)
            DYN_COEFF(r,c)
            dynamic_coeff = [dynamic_coeff; DYN_COEFF(r,c)];
        end
    end
end

% dynamic_coeff = [];
% for i = (1:N)
%     if i == N
%         a_i = DYN_COEFF{i};
%     else
%         a_i = DYN_COEFF{i} - DYN_COEFF{i+1};
%     end
%     dynamic_coeff = [dynamic_coeff; a_i];
% end

dynamic_coeff

