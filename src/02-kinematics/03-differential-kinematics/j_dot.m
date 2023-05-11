%% Sopravvivenza AIRO
%% Authors: Massimo Miryam Leonardo Francesco
% This script compute the derivative of a symbolic analytic jacobian

clc
clear all
close all

fprintf("First derivative of a Jacobian \n")
fprintf("REMEMBER to edit the scripts before using them\n\n")

% Define symbols
syms q1 q2 q3 t L;

% Define q as a function of t
q1(t) = symfun(str2sym('q1(t)'), t);
q2(t) = symfun(str2sym('q2(t)'), t);
q3(t) = symfun(str2sym('q3(t)'), t);

fprintf("This is the vectro q(t)")
q_t = [
        q1;
        q2;
        q3
    ];
q = q_t(t)

% Direct kinematics in funtion of t
fprintf("This is the direct kinematics of the robot r = f(q(t))\n")
Px = cos(q1) + cos(q1 +q2) + cos(q1 + q2 + q3)
Py = sin(q1) + sin(q1 +q2) + sin(q1 + q2 + q3)
phi = q1 + q2 + q3

r_t = [   
        Px;
        Py;
        phi;
    ];
r = r_t(t)

fprintf("Press Enter to keep going \n\n")
pause % to check for typos

% Computation of the Analytic Jacobian and singularities:
fprintf ("\nAnalytic Jacobian Analysis: \n")
fprintf ("This is the Jacobian Matrix Jr(q(t)): \n")

J = jacobian(r, q)

% Compute the determinant of Jacobian Matrix
fprintf("Determinant of Jr(q): \n")
det_J = simplify(det(J))

fprintf("Press Enter to get the first derivative \n\n")
pause % to check for typos

% Computation of J dot wrt time
J_dot = sym(zeros(size(J, 1), size(J, 2)));
for r = (1 : size(J, 1))
    for c = (1 : size(J, 2))
        J_dot(r,c) = diff(J(r,c), t);
    end
end

fprintf("First derivative of the Jacobian\n")
J_dot

fprintf("Press Enter to get h(q, q_dot) \n\n")
pause % to check for typos

% Derivative of q wrt time
fprintf("This is the derivative of q \n")
fprintf("Remmber to edit the values\n")
q_dot = [
            diff(q1(t), t)
            diff(q2(t), t)
            diff(q3(t), t)
        ]

fprintf("h(q, qd) \n")
h_q_qd = simplify(J_dot*q_dot)

% Symbols to define to sumplify h
syms D_q1 D_q2 D_q3

h_sym = subs(h_q_qd, {diff(q1(t),t), diff(q2(t),t), diff(q3(t),t)}, {D_q1, D_q2, D_q3})

h_eval = round(vpa(subs(h_sym, {q1, q2, q3, D_q1, D_q2, D_q3}, {pi/4, pi/3,-pi/2, -0.8, 1, 0.2})),4)



