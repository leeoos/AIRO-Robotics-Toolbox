% This code computes the inverse kinematic od a polar RRP robot

clc
clear all
close all

eps = 10^-10;
%% DATA: write your values

Px = 7;
Py = 5;
Pz = 7;
d1 = 1;

%% Computation of the angles 
%% First sequence

q3_1 = sqrt(Px^2+Py^2+(Pz-d1)^2);
q3_2 = -sqrt(Px^2+Py^2+(Pz-d1)^2);

if (q3_1<=eps) && (q3_1>=-eps)
    disp('q1 and q2 of the are undefined: we are in a singular case')
    disp('the values of q3 are')
    q3_first = q3_1
    q3_second = q3_2
    return
end


if (Px^2+Py^2<=eps) && (Px^2+Py^2>=-eps)
    disp('q1 is undefined in the first sequence: we are in a singular case')
    disp('The values of q2 and q3 for the first sequence are')
    q2_1 = atan2((Pz-d1)/q3_1, sqrt(Px^2+Py^2)/q3_1);
    q_first = [q2_1; q3_1]    
else
    q2_1 = atan2((Pz-d1)/q3_1, sqrt(Px^2+Py^2)/q3_1); 
    q1_1 = atan2(Py/cos(q2_1), Px/cos(q2_1));
    fprintf("This is the first solution: \n")
    q_first = [q1_1;q2_1;q3_1]
end

%% Second sequence
if (Px^2+Py^2<=eps) && (Px^2+Py^2>=-eps)
    disp('q1 is undefined in the second sequence: we are in a singular case')
    disp('The values of q2 and q3 for the second sequence are')
    q2_2 = atan2((Pz-d1)/q3_2, sqrt(Px^2+Py^2)/q3_2);
    q_second = [q2_2; q3_2]    
    return
else
    q2_2 = atan2((Pz-d1)/q3_2, sqrt(Px^2+Py^2)/q3_2);
    q1_2 = atan2(Py/cos(q2_2), Px/cos(q2_2));
    fprintf("This is the second solution: \n")
    q_second = [q1_2;q2_2;q3_2] 
end 


