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

% Insert DH table of parameters.
fprintf("DH table of parameters \n")
%"Notice that it is not unique!!! \n"

DHTABLE_SYM = [  sym('alpha1') sym('a1') sym('d1') sym('t1');
                 sym('alpha2') sym('a2') sym('d2') sym('t2');
                 sym('alpha3') sym('a3') sym('d3') sym('t3');
              ]

% N is the number of joints of the robots
N = size(DHTABLE_SYM,1);

% DH table values
fprintf("Remember to check the numbers in the script\n")
fprintf("Press Enter to keep going \n\n")
pause

alpha1 = pi/2;
alpha2 = 0;
alpha3 = 0;
alpha4 = pi/2;
alpha5 = pi/2;
alpha6 = pi/2;
alpha7 = 0;
alpha8 = 0;

a1 = 0; 
a2 = sym('l2');
a3 = sym('l3');
a4 = 0;
a5 = 0;
a6 = 0;
a7 = 0;
a8 = 0;

d1 = sym('l1');
d2 = 0;
d3 = 0;
d4 = sym('d4');
d5 = 0;
d6 = sym('d6');
d7 = 0;
d8 = 0;

t1 = sym('q1');
t2 = sym('q2');
t3 = sym('q3');
t4 = sym('q4');
t5 = sym('q5');
t6 = sym('q6');
t7 = sym('q7');
t8 = sym('q8');

fprintf("\nDH table of parameters \n")
fprintf("Notice that it is not unique!!! \n")
DHTABLE = subs(DHTABLE_SYM)

fprintf("Press Enter to keep going \n\n")
pause % to check for typos
     
% Build the general Denavit-Hartenberg trasformation matrix
TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
        sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
          0             sin(alpha)             cos(alpha)            d;
          0               0                      0                   1];

% Build transformation matrices for each link
% First, we create an empty cell array
A = cell(1,N);

% For every row in 'DHTABLE' substitute the right value 
% inside the general DH matrix
for i = 1:N
    alpha = DHTABLE(i,1);
    a = DHTABLE(i,2);
    d = DHTABLE(i,3);
    theta = DHTABLE(i,4);
    A{i} = subs(TDH);
    fprintf(" %d-A-%d",i-1, i)
    A{i}
% Direct kinematics
end

fprintf("Direct kinematics in symbolic form (simplifications may need some time)\n")
fprintf("Number of joints N = %d\n", N)

% Accumulation matrix
T = eye(4);

% Note: 'simplify' may need some time
% 



for i=1:N
    T = T*A{i};
    T = simplify(T);
    fprintf(" 0-T-%d", i)
    T
end
p = T(1:4,4)

