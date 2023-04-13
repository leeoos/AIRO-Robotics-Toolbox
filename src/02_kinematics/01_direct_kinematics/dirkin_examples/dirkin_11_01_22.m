%% Sopravvivenza_AIRO! 
%% Authors: Massimo, Miryam, Leonardo, Francesco, Paolo
% This script take as input DH table and compute
% the related homogeneous transformation matrix.

%% Copyrights to: A. De Luca) 5 Nov 2012 DH frames 
% assigned as in lecture slides 09_DirectKinematics.

clc
close all

%% Symbolical evaluation 

fprintf("Direct Denavit Hartenberg\n")
fprintf("Remember to edit the scripts before using them\n\n")

% Define symbolic variables
syms alpha a d theta 

% N is the number of joints of the robots
N = input("Insert the number of joints: ");


% Insert DH table of parameters.
fprintf("DH table of parameters \n")
fprintf("Notice that it is not unique!!! \n")

DHTABLE_SYM = [  sym('pha1') sym('a1') sym('d1') sym('t1');
                 sym('pha2') sym('a2') sym('d2') sym('t2');
                 sym('pha3') sym('a3') sym('d3') sym('t3');
                 sym('pha4') sym('a4') sym('d4') sym('t4');
              ];

% DH table values
fprintf("Remember to check the numbers in the script\n")
fprintf("Press Enter to keep going \n\n")
pause

pha1 = pi;
pha2 = -pi/2;
pha3 = -pi/2;
pha4 = 0;
pha5 = 0;
pha6 = 0;

a1 = sym('a'); 
a2 = 0;
a3 = 0;
a4 = sym('b');
a5 = 0;
a6 = 0;

d1 = 0;
d2 = 0;
d3 = sym('q3');
d4 = 0;
d5 = -115.7;
d6 = 92.2;

t1 = sym('q1');
t2 = sym('q2');
t3 = pi;
t4 = sym('q4');
t5 = 0;
t6 = 0;
           

         
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

q1 = pi/2;
q2 = -1;
q3 = 0;
K = 1;
L = 1;
M=1;

subs(T0N)

pause

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



 
