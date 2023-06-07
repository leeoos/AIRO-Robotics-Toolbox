%% Sopravvivenza_AIRO! 
% Authors: Massimo, Leonardo, Paolo, Francesco
% Implementation of moving frames algorithm to compute
% the symbolic dynamic model of a general robot.

clc 
close all
clearvars
clear all

lib_path = getenv("ROB2LIB_PATH");
addpath(lib_path);
rob2fun = rob2lib();

%% LOAD THE MODEL

% load robot datasheet
run(lib_path+"/models/spatial_model.m")

%% INFORMATION EXTRACTION
% DH parameters
dh_param = rob2fun.compute_dir_kin(DHTABLE);
A = dh_param{1}; % cell array of chain transformations

% Extraction of CoM vectors
W_CoM = cell(1,N);
A_i = eye(4);

for i = (1:N)
    A_i = A_i*A{i};
    A_i(1:3,:)
    W_CoM{i} = A_i * [R_CoM{i};1];
end

%% KINETIC ENERGY 
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
    velocity_CoM = simplify(VELOCITY{i+1} + cross(OMEGA{i+1}, R_CoM{i}));

    % Kinetic energy of each link
    KINETIC_ENERGY = KINETIC_ENERGY + (...
                     (1/2*m(i)*transpose(velocity_CoM)*velocity_CoM) + ... 
                     (1/2*transpose(OMEGA{i+1})*I{i}*OMEGA{i+1})...
                    );
    disp(["Step ", i])
    KINETIC_ENERGY = simplify(KINETIC_ENERGY);

    % Semplification of kinetic energy
    collect(simplify(expand(simplify(KINETIC_ENERGY))),q_dot.^2)
    
end
KINETIC_ENERGY = simplify(expand(simplify(KINETIC_ENERGY)));
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

% Potential energy
POTENTIAL_ENERGY = N_terms{2};
disp("Potential energy U")
POTENTIAL_ENERGY

g_q = N_terms{3};
disp("Gravity term")
g_q

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
% 
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

% % Comutation of matrix Y
% Y_q_q_dot_q_d_dot = rob2fun.compute_dyn_matrix(model, da, a, N);
% disp("Y matrix MAY BE NOT COMPLETE")
% Y_q_q_dot_q_d_dot


%% EMERGENCY STAFF
% % Initialization of symbolics vectors for q, q_dot and q_d_dot
% syms q [1 N]
% syms q_dot [1 N]
% q = transpose(q);
% q_dot = transpose(q_dot);
% 
% syms m [1 N]
% % Inertia Matricies (Evaluate if you have numbers!)
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
% for i =(1:N)
%     I{i} = [
%         Ixx(i), Ixy(i), Ixz(i);
%         Ixy(i), Iyy(i), Iyz(i);
%         Ixz(i), Iyz(i), Izz(i);
%     ];
% end

% % Direct kinematics.
% DHTABLE = [        
%     0    L1     0   q1;
%     pi/2 0      0   q2;
%     0    L3     0   q3;
% ];
% 
% 
% syms V F D C E
% % Vectro of the centers of m
% R_CoM = {[-L1+d1;0;0], [0;-L2+d2;0], [-L3+d3;0;0]};
% 
% % sigma vector:
% % 0: revolout 
% % 1: prismatic
% sigma = [
%     0;
%     0;
%     0;
% ];
% 
% % Initial conditions for angular velocity
% initial_omega = [
%     0;
%     0;
%     0;
% ];
% OMEGA = cell(1, N);
% OMEGA{1} = initial_omega;
% 
% % Initial conditions for linear velocity
% initial_velocity = [
%     0;
%     0;
%     0;
% ];
% VELOCITY = cell(1, N);
% VELOCITY{1} = initial_velocity;

% %% M(q) INERTIA MATRIX
% % Collection of q_dot terms
% for i = (1:N)
%     KINETIC_ENERGY = collect(KINETIC_ENERGY, q_dot(i)^2);
% end
% 
% syms foo % tmp symbol
% M_q = foo;
% for r = (1:N)
%     for c = (1:N)
%         if ((r == c))
%             M_q(r,c) = simplify(diff(KINETIC_ENERGY, q_dot(r), 2));
%         else
%             K_reduced_qr = simplify(diff(KINETIC_ENERGY, q_dot(c)));
%             K_reduced_qrc = simplify(diff(K_reduced_qr, q_dot(r)));
%             M_q(r,c) = simplify(K_reduced_qrc);
%         end
%     end
% end
% disp("M(q) Inertia Matrix")
% M_q
% 
% 
% 
% %% CORIOLIS, CENTRIFUGAL c(q, q_dot) AND GRAVITY g(q)
% 
% % Computation of coriolis and centrifugal term
% c_q_q_dot = []; 
% 
% for i = (1:N)
%     dMi_dq = jacobian(M_q(:,i), q);
%     dM_dqi = diff(M_q, q(i));
%     Ci = 1/2*(dMi_dq + transpose(dMi_dq) - dM_dqi);
%     ci = simplify(transpose(q_dot)*Ci*q_dot);
%     c_q_q_dot = [c_q_q_dot;ci];
% end
% c_q_q_dot = simplify(expand(simplify(c_q_q_dot)))
% 
% % Computation Potential energy 
% U = 0;
% for i = (1:N)
%     U_i = simplify(- m(i)*(transpose(g))*W_CoM{i}(1:3));
%     U = simplify(U + U_i);
% end
% U;
% 
% % Computation of gravity term g(q)
% disp("Gravity term")
% g_q = simplify(expand(gradient(U, q)))

