%% Sopravvivenza_AIRO! 
% Author: Massimo

% This script compute the solution for an LQ Optimization Problem
% Objective Function -> H(x) = 1/2 (dq-dq0)' W (dq-dq0)
% Constraint -> Jdq = dr

% Input: W (Weight Matrix) from H(x),
% dq0 (Preferred Solution) from H(x),
% J (Jacobian Matrix) from the constraint,
% dr (Task Velocity) from the constraint.

clear all
close all
clc

W = [1 -1; -1 3];
dq0 = [0;0];
J = [2 -1];
dr = 1

W_inv = inv(W);
Jt = transpose(J);

JPw = W_inv*Jt*inv(J*W_inv*Jt)

dq = dq0 + JPw*(dr-J*dq0)
