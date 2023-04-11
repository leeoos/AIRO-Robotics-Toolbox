%% Sopravvivenza_AIRO! 
% Author: Massimo, Leonardo
% This script take as input two configurations of the robot (initial and
% final configuration of joints) and interpolate them using cubic
% polynomial interpolation. Initially the algorithm compute all
% symbolically, finally you can use your project value to decide
% initial, final velocities and accelerations.

syms tau t DQ q_initial

% Evaluation of your value

T = 2;
q_start = [-0.7854; 0.7854; 0.7854] 
q_goal = [0;0;0.7854]
q_dot_start = [2.8284;-8.4853;0]
q_dot_goal = [0;0;0]
q_double_dot_start = [0;0;0]
q_double_dot_goal = [0;0;0]
qt = [sym('0');sym('0');sym('0')]

% First quintic polynomial
for i=(1:3)
    q_start_i = q_start(i);
    q_goal_i = q_goal(i);
    deltaQ = q_goal(i)-q_start(i);
    q_dot_start_i = q_dot_start(i);
    q_dot_goal_i = q_dot_goal(i);
    q_double_dot_start_i = q_double_dot_start(i);
    q_double_dot_goal_i = q_double_dot_goal(i);

    if q_start_i ~= q_goal_i
       f = 0
       e = round(vpa((T/deltaQ)*q_dot_start_i),3)
       d = round(vpa((T^2 / 2*deltaQ)*q_double_dot_start_i),3)
       a = round(vpa((12*deltaQ - 6*T*q_dot_goal_i - 6*T*q_dot_start_i + T^2*q_double_dot_goal_i + 5*T^2*q_double_dot_start_i)/(2*deltaQ)),3)
       b = round(vpa(-(15*deltaQ - 7*T*q_dot_goal_i - 8*T*q_dot_start_i + T^2*q_double_dot_goal_i + 6*T^2*q_double_dot_start_i)/(deltaQ)),3)
       c = round(vpa((20*deltaQ - 8*T*q_dot_goal_i - 12*T*q_dot_start_i + T^2*q_double_dot_goal_i + 7*T^2*q_double_dot_start_i)/(2*deltaQ)),3)
       
       fprintf('Quintic Polynomial n. %d', i)
       str = strcat(string(q_start_i), " + ", string(deltaQ), " * (", string(a*tau^5 + b*tau^4 + c*tau^3 + d*tau^2 + e*tau + f), ")")
       qt(i) =  deltaQ*((a/T^5)*t^5 + (b/T^4)*t^4 + (c/T^3)*t^3 + (d/T^2)*t^2 + (e/T)*t) + f + vpa(q_start_i)

    else 
        fprintf("The %d quintic polynomial is constant: %f \n" , i, q_start_i)
        qt(i) = 0*t + vpa(q_start_i);
    end
end

% Joints Positions
qt1= qt(1);
qt2= qt(2);
qt3 = qt(3);

% Joints Velocities

qt1_dot = diff(qt1,t)
qt2_dot = diff(qt2,t)
qt3_dot = diff(qt3,t)

% Joints Accelerations

qt1_ddot = diff(qt1_dot,t)
qt2_ddot = diff(qt2_dot,t)
qt3_ddot = diff(qt3_dot,t)

% Plot of Positions, Velocities and Accelerations of Joints

 t = [0:0.1:T];
 qt1_plot= 0*t + subs(qt1);
 qt2_plot= 0*t + subs(qt2);
 qt3_plot = 0*t + subs(qt3);
 qt1_dot_plot= 0*t + subs(qt1_dot);
 qt2_dot_plot= 0*t + subs(qt2_dot);
 qt3_dot_plot = 0*t + subs(qt3_dot);
 qt1_ddot_plot= 0*t + subs(qt1_ddot);
 qt2_ddot_plot= 0*t + subs(qt2_ddot);
 qt3_ddot_plot = 0*t + subs(qt3_ddot);
 figure
 hold on
 plot(t,qt1_plot,t,qt2_plot,t,qt3_plot);grid; title('position');xlabel('time [s]');ylabel('[rad]')
 hold off

 figure
 hold on
 plot(t,qt1_dot_plot,t,qt2_dot_plot,t,qt3_dot_plot);grid; title('velocity');xlabel('time [s]');ylabel('[rad/s]')
 hold off

 figure
 hold on
 plot(t,qt1_ddot_plot, t,qt2_ddot_plot, t,qt3_ddot_plot);grid; title('acceleration');xlabel('time [s]');ylabel('[rad/s^2]')

 grid on

