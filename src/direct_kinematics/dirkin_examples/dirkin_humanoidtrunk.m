%% DH transformation matrices and direct kinematics of a serial robot (Humanoid Trunk example)
%% Author Massimo

clear all
close all
clc

%% Define symbolic variables

syms alpha d a theta
syms d0 d2 d4 theta1 theta2 theta3

%% number of joints of Humanoid Trunk

N=3;

%% Insert DH table of parameters of Humanoid Trunk

DHTABLE = [-pi/2, 0, 0, theta1;
           -pi/2, 0, d2, theta2;
           +pi/2, 0,  0, theta3]
         

         
%% Build the general Denavit-Hartenberg trasformation matrix

TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
        sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
          0             sin(alpha)             cos(alpha)            d;
          0               0                      0                   1];

%% Build transformation matrices for each link
% First, we create an empty cell array (an array in which each cell
% contains different data types

A = cell(1,N);

% For every row in 'DHTABLE' we substitute the right value inside
% the general DH matrix
% For example in first iteration (i=1), we have alpha = element(1,1), 
% a = element (1,2) d = element(1,3) and theta = element(1,4) and so on
% Then we substitute these values in TDH matrix and save it in first cell 
% of A array (First 0_A_1 Homogeneus Matrix)

for i = 1:N
    alpha = DHTABLE(i,1);
    a = DHTABLE(i,2);
    d = DHTABLE(i,3);
    theta = DHTABLE(i,4);
    A{i} = subs(TDH);
    fprintf("%d_A_%d", i-1, i)
    A{i}
end

%% Direct kinematics

disp('Direct kinematics of UR10 robot in symbolic form (simplifications may need some time)')

disp(['Number of joints N=', num2str(N)])

% Note: 'simplify' may need some time
% In this section we make products of all homogeneus matricies using first
% an identity matrix, and simplify trigonometric expression each time

T = eye(4);

for i=1:N 
    T = T*A{i};
    fprintf("%d_T_%d", i-1, i)
    T = simplify(T)
end

% output TN matrix
% This is the homogeneus transformation between last frame RFN 
% and first frame RF0

T0N = T

% output ON position
% We print the first 3 rows of last column (position of Origin of RFN w.r.t
% Origin of RF0 Frame

p = T(1:3,4)

%% End Effector Axes

% In this case we print each column of Rotation Matrix that describe the
% orientation of RFN w.r.t RF0 Frame (n -> xN Axis, s -> yN Axis, a -> zN Axis) 

% output xN axis
n=T(1:3,1)

% output yN axis
s=T(1:3,2)

% output zN axis
a=T(1:3,3)

%% Final product

TW0 = [0,   0,  1,  0 ;
      -1,  0,  0,   0;
      0, -1, 0,    d0;
      0,  0,  0,   1 ];

T3E = [1, 0,  0,  0 ;
      0,  1,  0,  0;
      0,  0,  1,  d4;
      0,  0,  0,   1];

T_fin = TW0*T0N*T3E

%end 