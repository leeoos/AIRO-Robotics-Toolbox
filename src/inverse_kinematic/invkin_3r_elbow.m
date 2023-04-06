%% Sopravvivenza_AIRO! 
%% Author: Massimo, Miryam, Leonardo, Federico, Francesco, Paolo
% This script take as input the values for the direct kinematics of
% a 3R elbow-type robot and compute the possible joint configurations

clc
close all

fprintf("Inverse Kinematic 3R planar\n")
fprintf("REMEMBER to edit the scripts before using them\n\n")

% Define symbols
syms L1 L2 L3 C1 C2 C3 S1 S2 S3 q1 q2 q3

% Define low boundary condition to compare with zero 
eps = 10^-10;

% Values of the direct kinematics
fprintf("Values of the end-effector position: \n")
px = 2
py = 2
pz = 2

fprintf("Links lenghts of the robot: \n")
d1 = 3
l2 = 3
l3 = 2

%% Computation of q3

cos_q3 = (px^2 + py^2 + (pz - d1)^2 - l2^2 - l3^2) / (2*l2*l3);
sin_q3 = sqrt(1 - cos_q3^2);

if (cos_q3 >= -1 && cos_q3 <= 1)
    % computing the 2 possible values for q3

    q3_1 = round(vpa(atan2(sin_q3, cos_q3)), 4);
    q3_2 = round(vpa(atan2(-sin_q3, cos_q3)), 4);

else
    % stop condition when the value for cos(q3) is NOT in [-1,1]

    fprintf("The point p is outside the workspace!!! \n")
    return

end

%% Computation of q1

if (px^2 + py^2 > 0)
    % computing the 2 possible values for q1

    cos_q1 = px/sqrt(px^2 + py^2);
    sin_q1 = py/sqrt(px^2 + py^2);
    q1_1 = round(vpa(atan2(px, cos_q1)), 4);
    q1_2 = round(vpa(atan2(-sin_q1, -cos_q1)), 4);

else
    % stop condition when (px^2 + py^2) < 0

    fprint("Infinite solutions for q1 \n")
    return

end

%% Computation of q2

% system to find cos(q2) and sin(q2)

A = [   l2 + l3*C3, -l3*S3;
        l3*S3, l2 + l3*C3;
    ];

b = [   C1*px + S1*py;
        pz - d1
    ];

x = A\b; % suggested to solve symbolic systems

det_A = px^2 + py^2 + (pz - d1)^2;

if (det_A >= -eps && det_A <= eps)
    % stop condition when Det(A) = 0

    fprintf("Infinite solutions for q2 \n")
    return

else
    % computing the 2 possible values for q2

    % case of all positive
    C1 = cos_q1;
    S1 = sin_q1;

    % case sin(q3) positive
    C3 = cos_q3;
    S3 = sin_q3;
    
    x1 = subs(x);
    q2_1 = round(vpa(atan2(x1(2), x1(1))), 4);

    % case of all positive
    C1 = cos_q1;
    S1 = sin_q1;

    % case sin(q3) negative
    C3 = cos_q3;
    S3 = -sin_q3;
    
    x2 = subs(x);
    q2_2 = round(vpa(atan2(x2(2), x2(1))), 4);

    % case all negative
    C1 = -cos_q1;
    S1 = -sin_q1;

    % case sin(q3) positive
    C3 = cos_q3;
    S3 = sin_q3;

    x3 = subs(x);
    q2_3 = round(vpa(atan2(x3(2), x3(1))), 4);

    % case all negative
    C1 = -cos_q1;
    S1 = -sin_q1;

    % case sin(q3) negative
    C3 = cos_q3;
    S3 = -sin_q3;

    x4 = subs(x);
    q2_4 = round(vpa(atan2(x4(2), x4(1))), 4);

end

% Print the result
fprintf("The four sequence are \n")
first   =   [ q1_1 q2_1 q3_1 ];
second  =   [ q1_1 q2_2 q3_2 ];
third   =   [ q1_2 q2_3 q3_1 ];
fourth  =   [ q1_2 q2_4 q3_2 ];

sol = [first; second; third; fourth];

% Direct kinematic equations to check
% 
for i = (1 : 4)
    
    fprintf("\nSequence " + i + ":\n")
    disp(sol(i,:))

    px_inv = cos(sol(i,1))*(l2*cos(sol(i,2)) + l3*cos(sol(i,2) + sol(i,3)))
    py_inv = sin(sol(i,1))*(l2*cos(sol(i,2)) + l3*cos(sol(i,2) + sol(i,3)))
    pz_inv = d1 + l2*sin(sol(i,2)) + l3*sin(sol(i,2) + sol(i,3))
end


