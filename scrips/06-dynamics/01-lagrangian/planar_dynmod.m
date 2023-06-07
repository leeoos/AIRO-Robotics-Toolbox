%% Sopravvivenza_AIRO! 
% Authors: Massimo, Leonardo, Paolo, Francesco
% Implementation of Koning Theorem to compute the 
% dynamic model of a generic planar robot,
% given the pose of CoM and angular velocities.

clc 
close all
clearvars
clear all

lib_path = getenv("ROB2LIB_PATH");
addpath(lib_path);
rob2fun = rob2lib();

%% LOAD MODEL
% load robot datasheet
run(lib_path+"/models/prrr_planar.m")

%% KINETIC ENERGY
% Implementation of Koening theorem for computing the 
% kinetic energy of each link
KINETIC_ENERGY = cell(1, N);
for i = (1:N) 
     KINETIC_ENERGY{i} = (1/2*m(i)*transpose(CoM_VELOCITY{i})*CoM_VELOCITY{i} + ...
         + (1/2*transpose(OMEGA{i})*I{i}*OMEGA{i}));
     KINETIC_ENERGY{i} = collect(simplify(expand(simplify(KINETIC_ENERGY{i}))),q_dot.^2);
     %KINETIC_ENERGY{i} = collect(KINETIC_ENERGY{i},m);
end
celldisp(KINETIC_ENERGY) % uncomment this for debug

KINETIC_ENERGY = sum([KINETIC_ENERGY{:}]);
disp("Kinetic energy T")
KINETIC_ENERGY

%% M(q) INERTIA MATRIX
% Extraction M(q) elements
M_q = rob2fun.compute_inertia_matrix(KINETIC_ENERGY, N);
disp("M(q) Inertia Matrix")
M_q

%% CORIOLIS, CENTRIFUGAL c(q, q_dot) AND GRAVITY g(q)
disp("Computation of coriolis, centrifugal and gravity terms")
N_terms = rob2fun.compute_n_terms(M_q, W_CoM, g, N);

c_q_q_dot = N_terms{1};
disp("Coriolis and centrifugal term")
c_q_q_dot

S_q_q_dot = N_terms{2};
disp("S matrix for coriolis and centrifugal term")
S_q_q_dot

% Skew Symmetry test for M_dot-2S
is_M_2S_skew = rob2fun.skew_symmetry_test(M_q, S_q_q_dot, q, q_dot);
disp(" ")

% Potential energy
POTENTIAL_ENERGY = N_terms{3};
disp("Potential energy U")
POTENTIAL_ENERGY

g_q = N_terms{4};
disp("Gravity term")
g_q

disp("Hessian of potential energy")
A = jacobian(g_q, q);
A

disp("Charaterisctic polynom of A")
A_t_A = transpose(A)*A;
A_t_A
poly = det(lambda*eye(size(A_t_A,1)) - A_t_A);
poly

%% DYNAMIC MODEL u
model = M_q*q_d_dot + c_q_q_dot + g_q;
disp("Dynamic Model with a")
model

%% COMPUTATION OF DYNAMICS COEFFICIENTS (EXPERIMENTAL)
% PAY ATTENTION !!! 
disp("PAY ATTENTION THIS MAY NOT WORK !!!")
disp("This part is dedicated to the computation of the dynamic coefficient.")
disp("This means that require some manual computation !!!")
disp(" ")

% "Easy" way to extract dynamic coefficents
triang_M = [sym("foo"); sym("bar")];
index = 1;
for r = (1:N)
    for c = (r:N)
        triang_M(index) = M_q(r,c);
        index = index + 1;
        %triang_M = [triang_M; M_q(r,c)];
    end
end
disp("Extract the dynamic coefficients form here")
triang_M 

% % PAY ATTENTION !!! Insert here the values for a1, a2, ..., an
% da1 = m1 + m2 + m3 + m4;
% da2 = m2 + m3 + m4;
% da3 = Izz3 + m3*d3^2 + Izz4 + m4*d4^2 + m4*L3^2;
% da4 = Izz4 + m4*d4^2;
% da5 = m4*d4;
% da6 = m3*d3 + m4*L3;
% 
% % PAY ATTENTION !!! edit also this vector
% da = [da1, da2, da3, da4, da5, da6];
% dynamic_coefficients = transpose(da);
% disp("Dynamic coefficients")
% dynamic_coefficients
% 
% % Symbolic dynamic coefficients
% NUM_DYN = size(da,2);
% syms a [1 NUM_DYN]
% 
% % Comutation of matrix Y
% Y_q_q_dot_q_d_dot = rob2fun.compute_dyn_matrix(model, da, a, N);
% disp("Y matrix MAY BE NOT COMPLETE")
% Y_q_q_dot_q_d_dot


%% EMERGENCY STAFF
% % Standard symbolic variables for joints
% syms q [1 N]
% syms q_dot [1 N]
% syms q_d_dot [1 N]
% q = transpose(q);
% q_dot = transpose(q_dot);
% q_d_dot = transpose(q_d_dot);
% 
% % Symbolic variables for dynamic coefficients
% syms m [1 N]
% syms L [1 N]
% syms d [1 N]
% syms t g0
% % Inertia Matricies (Evaluate if you have numbers!)
% % Set this value to false if you want to use a full inertia matrix
% diagonal_inertia = true;
% 
% % X
% syms Ixx [1 N]
% syms Ixy [1 N] 
% syms Ixz [1 N]
% % Y
% syms Iyx [1 N]
% syms Iyy [1 N]
% syms Iyz [1 N]
% % Z
% syms Izz [1 N]
% syms Izx [1 N]
% syms Izy [1 N]
% 
% I = cell(1,3);
% if diagonal_inertia
%     for i =(1:N)
%         I{i} = [
%             Ixx(i), 0, 0;
%             0, Iyy(i), 0;
%             0, 0, Izz(i);
%         ];
%     end
% else 
%     for i =(1:N)
%         I{i} = [
%             Ixx(i), Ixy(i), Ixz(i);
%             Ixy(i), Iyy(i), Iyz(i);
%             Ixz(i), Iyz(i), Izz(i);
%         ];
%     end
% end
% % PAY ATTENTION: this part require a precomputation that depends on the problem
% % Vectors of the centers of masses w.r.t world frame
% W_CoM = {
%     [q1-d1; 0; 0], ...
%     [q1; q2-d2; 0], ...
%     [q1 + d3*cos(q3);
%      q2 + d3*sin(q3);
%      0
%     ], ...
%     [q1 + L3*cos(q3) + d4*cos(q3+q4);
%      q2 + L3*sin(q3) + d4*sin(q3+q4);
%      0
%     ], ...
% };
% % uncomment for debug
% celldisp(W_CoM)
% 
% % Velocities of the centers of masses w.r.t world frame
% CoM_VELOCITY = {
%     [q_dot1; 0; 0], ... 
%     [q_dot1; q_dot2; 0], ...
%     [q_dot1 - d3*sin(q3)*q_dot3; 
%      q_dot2 + d3*cos(q3)*q_dot3; 
%      0], ...
%     [q_dot1 - L3*sin(q3)*q_dot3 - d4*sin(q3+q4)*(q_dot3+q_dot4); 
%      q_dot2 + L3*cos(q3)*q_dot3 + d4*cos(q3+q4)*(q_dot3+q_dot4); 
%      0], ...
% };
% % uncomment for debug
% celldisp(CoM_VELOCITY)
% 
% % % Angular velocities of each link
% OMEGA = {
%     [0; 0; 0], ... 
%     [0; 0; 0], ...
%     [0; 0; q_dot3], ...
%     [0; 0; q_dot3 + q_dot4], ...
% };
% % uncomment for debug
% celldisp(OMEGA)
