%% Sopravvivenza_AIRO! 
%% Author: Massimo, Miryam, Leonardo, Federico, Francesco

% This algorithm compute the derivative of an Elementary Rotation Matrix

clc
clear all
close all

%% DEFINITION OF VARIABLES 

syms wx wy wz phi phi_dot

 R = [cos(phi), -sin(phi), 0;
      sin(phi),  cos(phi), 0
         0,          0,    1]

Sw = [0, -wz, wy; 
      wz, 0, -wx; 
     -wy, wx,  0]

%% DIRECT FORMULA

fprintf("Derivative of R in symbolic form: \n")
R_dot = Sw * R;

%% INVERSE FORMULA

fprintf("Vector of angular velocity in base frame: \n")

R_dot_inv = sym('r%d%d', [3 3]);

for i=1:9 
    R_dot_inv(i) = diff(R(i), phi);
end

Sw_inv = eye(3);

Sw_inv = R_dot_inv * transpose(R);

phi_dot * simplify (Sw_inv)


