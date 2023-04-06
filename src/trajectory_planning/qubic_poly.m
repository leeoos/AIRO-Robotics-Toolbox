%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo

% This script take as input two configurations of the robot (initial and
% final configuration of joints) and interpolate them using cubic
% polynomial interpolation. Initially the algorithm compute all
% symbolically, finally you can use your project value to decide
% initial, final velocities and accelerations.

clc
close all

syms tau t DQ q_initial

% Evaluation of your value

T = 2;
q_start = [3.14; 0] 
q_goal = [1.55; -1.11]
q_dot_start = [0;0]
q_dot_goal = [-0.46;0.20]
qt = [sym('0');sym('0')]
%qt = sym(zeros(6))

% First quintic polynomial
for i=(1:2)
    q_start_i = q_start(i);
    q_goal_i = q_goal(i);
    deltaQ = q_goal(i)-q_start(i);
    q_dot_start_i = q_dot_start(i);
    q_dot_goal_i = q_dot_goal(i);

    if q_start_i ~= q_goal_i
       d = 0
       a = round(vpa(((T*q_dot_start_i + T*q_dot_goal_i)/deltaQ) - 2),3)
       b = round(vpa((-(2*T*q_dot_start_i + T*q_dot_goal_i)/deltaQ) + 3),3)
       c = round(vpa((T*q_dot_start_i)/deltaQ),3)
       
       fprintf('Quintic Polynomial n. %d', i)
       str = strcat(string(q_start_i), " + ", string(deltaQ), " * (", string(a*tau^3 + b*tau^2 + c*tau + d), ")")
       qt(i) =  deltaQ*((a/T^3)*t^3 + (b/T^2)*t^2 + (c/T)*t + d) + vpa(q_start_i)

    else 
        fprintf("The %d quintic polynomial is constant: %f \n" , i, q_start_i)
        qt(i) = 0*t + vpa(q_start_i);
    end
end

% Joints Positions
qt1= qt(1);
qt2= qt(2);

% Joints Velocities

qt1_dot = diff(qt1,t)
qt2_dot = diff(qt2,t)

% Joints Accelerations

qt1_ddot = diff(qt1_dot,t)
qt2_ddot = diff(qt2_dot,t)

% Plot of Positions, Velocities 

 t=[0:0.001:T];
 qt1_plot= 0*t + subs(qt1);
 qt2_plot= 0*t + subs(qt2);
 
 qt1_dot_plot= 0*t + subs(qt1_dot);
 qt2_dot_plot= 0*t + subs(qt2_dot);

 
 figure
 hold on
 plot(t,qt1_plot,t,qt2_plot);grid; title('position');xlabel('time [s]');ylabel('[rad]')
 hold off

 figure
 hold on
 plot(t,qt1_dot_plot,t,qt2_dot_plot);grid; title('velocity');xlabel('time [s]');ylabel('[rad/s]')
 hold off

 grid on
