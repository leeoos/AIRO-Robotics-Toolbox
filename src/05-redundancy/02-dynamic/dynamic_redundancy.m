%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo, Paolo, Francesco
% This script compute the minimum torque solution for a redundant robot

clc
clear all
close all

%% INPUTS

N = 2; % number of joints

syms q [1 N] % Generalized coordinates
syms q_dot_ [1 N] % Joint Velocities
syms L [1 N] % Link Lenghts
syms m [1 N] % Mass of each link
syms d [1 N] % Dynamic distances
syms g0 a b c r_ddot_x r_ddot_y r_ddot_x

assume(a, 'real')
assume(b, 'real')
assume(c, 'real')

% Acceleration task vector
r_ddot = [
    r_ddot_y
];

% Write the position of center of mass in RF0
o_r_i_com = {
    [0;q1-d1;0],...
    [d2*cos(q2);q1+d2*sin(q2);0], ...
};

% You should change the gravity component depend on you RF0 frame
g = [0,-g0,0];

% Jacobian Matrix
J = [1, L2*cos(q2)];

% Evaluate the Jacobian with your values!!!
configuration = [1, pi/2];
disp("Jacobian Matrix evaluated: ")
J = subs(J, q, configuration)

% pseudoinverse of evaluated Jacobian 
pinvJ = pinv(J);
simplify(pinvJ)

% Jacobian Derivative
J_dot = [
    0, -L2*sin(q2)*q_dot_2
];

% Actual q_dot definition
q_dot_eval = [
    0;
    0;
];

% Inertia matrix M(q)
M = [   
    a,     b*cos(q2);
     b*cos(q2),    c    
];

%% END OF INPUTS

% Coriolis e Gravity Term
c_g_terms = dynamic_cg(M, o_r_i_com, g, N, q, q_dot_, m);
n = c_g_terms{1} + c_g_terms{2};
disp("Dynamic Term n(q,q_dot) = c(q,q_dot)+g(q):")
n

% Evaluate the Inertia Matrix with your values:
M = subs(M, q, configuration);
inv_M = M^-1;
disp("Inverse of M(q) matrix:")
inv_M

pinvJM = pinv(J*inv_M)
disp("Pseudoinverse:")
pinvJM

% Evaluation of n term in actual condition
n = subs(n, q, q);
n = subs(n, transpose(q_dot_), q_dot_eval);
disp("Evaluated corilis, centrifugal and gravity terms")
n

% Minimum norm
tau_1 = simplify(pinvJM*(r_ddot - J_dot*q_dot_eval + J*inv_M*n));
disp("Computation of minimum torque norm solution")
tau_1

%% COMPUTATION OF MINIMUM (SQUARED INVERSE INERTIA WEIGHTED) TORQUE NORM
% Squared inverse inertia
tau_2 = simplify(M*pinvJ*(r_ddot - J_dot*q_dot_eval + J*inv_M*n));
disp("Computation of minimum (squared inverse inertia weighted) torque norm")
tau_2

%% COMPUTATION OF MINIMUM (INVERSE INERTIA WEIGHTED) TORQUE NORM
% Transpose Jacobian Matrix
fprintf("This is the transpose of Jacobian Matrix: \n")
Jt = transpose(J);

% Inverse inertia
tau_3 = simplify(Jt*(J*inv_M*Jt)^-1*(r_ddot - J_dot*q_dot_eval + J*inv_M*n));
disp("Computation of minimum (inverse inertia weighted) torque norm")
tau_3





%% FUNCTIONS SECTION
%-------------------------------------------------------------------
%-------------------------------------------------------------------
%-------------------------------------------------------------------
%-------------------------------------------------------------------
%% COMPUTATION OF CORIOLIS AND CENTRIFUGAL TERM
% This part compute the coriolis and centrifugal terms

function n_term = dynamic_cg(M, o_r_i_com, g, N, q, q_dot_, m)

    c = []; % Coriolis and Centrifugal vector
    
    for i=1:N
        dMidq = jacobian(M(:,i), q);
        dMdqi = diff(M, q(i));
        fprintf("This is the %d Christoffel Symbol: \n", i)
        Ci = simplify(1/2*(dMidq + transpose(dMidq) - dMdqi))
        fprintf("This is the %d Coriolis Term: \n", i)
        ci = expand(simplify(q_dot_*Ci*transpose(q_dot_)))
        c = [c;ci];
    end
    
    fprintf("This is the coriolis and centrifugal term c(q,dq):")
    simplify(c)
    
    %% COMPUTATION OF GRAVITY TERM
   
    % Potential Energy Computation
    U_tot = 0;
    
    for i=1:N
        fprintf("This is the U%d Potential Energy: \n", i)
        Ui = simplify(-m(i)*g*o_r_i_com{i})
        U_tot = U_tot + Ui;
    end
    
    fprintf("This is the gravity term g(q):")
    g_q = transpose(jacobian(U_tot,q))

    n_term = {c,g_q};
end
