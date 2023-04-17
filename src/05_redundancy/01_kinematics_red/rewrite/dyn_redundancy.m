%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco

% This script compute the kinetic energy of each link of a Robot
% through the Konig Theorem. 
% Input => 1) LINEAR VELOCITY OF EACH LINK
%          2) ANGULAR VELOCITY OF EACH LINK
%          3) MASS AND INERTIA OF EACH LINK

clc
clear all
close all

N = 2; % number of joints
M = 1; % task dimentions

syms q [1 N] 
syms q_dot [1 N]
syms L [1 N]
syms m [1 N]
syms d [1 N]
syms a b c d e              
syms r_d_dot [1 M]

assume(a, 'real')
assume(b, 'real')
assume(c, 'real')
q  = transpose(q); assume(q, 'real')
q_dot  = transpose(q_dot); assume(q_dot, 'real')
r_d_dot  = transpose(r_d_dot); assume(r_d_dot, 'real')


M = [
    a,          b*cos(q2);
    b*cos(q2),  c;
];

M_m1 = (M)^(-1)

J = [1, L2*cos(q2)];

J_dot = [0, L2*cos(q2)*q_dot2];

%% Computation of symbolic c(q,q_dot) and g(q)

c = []; % Coriolis and Centrifugal vector

for i=1:N
    dMidq = jacobian(M(:,i), q);
    dMdqi = diff(M, q(i));
    Ci = 1/2*(dMidq + transpose(dMidq) - dMdqi);
    ci = simplify(transpose(q_dot)*Ci*q_dot);
    ci = simplify(collect(ci, q_dot));
    c = [c;ci];
end

fprintf("This is the coriolis and centrifugal term c(q,dq):");
simplify(c)

%% COMPUTATION OF GRAVITY TERM

% DEFINITION OF GRAVITY VECTOR AND r0ci (height of each com wrt RF0)
syms g0

% You should change the gravity component depend on you RF0 FRAME
g = [0,-g0,0];

% WRITE THE POSITION OF CENTER OF MASS IN RFi (IN HOMOGENEOUS COORD)
o_r_i_com = {[0;q1;0], ...
            [d2*cos(q2);q1+d2*sin(q2);0], ...
            };

% Potential Energy Computation
U_tot = 0;
for i=1:N
    fprintf("This is the U%d Potential Energy: \n", i)
    Ui = simplify(-m(i)*g*o_r_i_com{i})
    U_tot = U_tot + Ui;
end
fprintf("This is the gravity term g(q): \n")
g_q = transpose(jacobian(U_tot,q))

% There are three possible objective function for dynamics redundancy

tao_1 = pinv(J*M_m1) * (r_d_dot - J_dot*q_dot + J*M_m1*N);

