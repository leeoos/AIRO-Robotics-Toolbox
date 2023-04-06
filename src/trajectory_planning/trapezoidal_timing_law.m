%% Sopravvivenza_AIRO! 
%% Author: Massimo, Miryam, Leonardo, Federico, Francesco

clc 
close all

syms t1 t2 t3


% Data

q_i = 0.64;

q_f = 2.35;

sign_L = q_f - q_i;

L = q_f - q_i

v_max = 2 

a_max = 4

%% COMPUTATION TS & T

x =(v_max)^2 / a_max;

if L > x

    T = (L*a_max + v_max^2)/(a_max*v_max)
    
    Ts = v_max/a_max

elseif L == x

    T = 2*v_max/a_max

    Ts = T/2

elseif L < T

    T = sqrt(4*L/a_max)

    Ts = T/2

 else 

    disp('Sei un coglione :)');

end 

if sign_L < 0

    % Position profile
    qt1 = vpa(-a_max*t1^2/2)
    qt2 = vpa(-v_max*t2 + ((v_max^2)/(2*a_max)))
    qt3 = vpa((a_max*(t3-T)^2)/2 - v_max*T + (v_max^2/a_max))
    
    % Velocity profile
    qt1_dot = vpa(diff(qt1, t1))
    qt2_dot = vpa(diff(qt2, t2))
    qt3_dot = vpa(diff(qt3, t3))
    
    % Acceleration profile
    qt1_ddot = vpa(diff(qt1_dot, t1))
    qt2_ddot = vpa(diff(qt2_dot, t2))
    qt3_ddot = vpa(diff(qt3_dot, t3))

elseif sign_L > 0
    
    %Position profile
    qt1 = vpa(a_max*t1^2/2)
    qt2 = vpa(v_max*t2 - ((v_max^2)/(2*a_max)))
    qt3 = vpa((-a_max*(t3-T)^2)/2 + v_max*T - (v_max^2/a_max))
    
    % Velocity profile
    qt1_dot = vpa(diff(qt1, t1))
    qt2_dot = vpa(diff(qt2, t2))
    qt3_dot = vpa(diff(qt3, t3))
    
    % Acceleration profile
    qt1_ddot = vpa(diff(qt1_dot, t1))
    qt2_ddot = vpa(diff(qt2_dot, t2))
    qt3_ddot = vpa(diff(qt3_dot, t3))
end

%% PLOT SECTION

t1=[0:0.001:Ts];
t2=[Ts:0.001:T-Ts];
t3=[T-Ts:0.001:T];

qt1_eval = subs(qt1);
qt2_eval = subs(qt2);
qt3_eval = subs(qt3);

qt1_dot_eval = 0*t1 + subs(qt1_dot);
qt2_dot_eval = 0*t2 + subs(qt2_dot);
qt3_dot_eval = 0*t3 + subs(qt3_dot);

qt1_ddot_eval = 0*t1 + subs(qt1_ddot);
qt2_ddot_eval = 0*t2 + subs(qt2_ddot);
qt3_ddot_eval = 0*t3 + subs(qt3_ddot);


figure
hold on
plot(t1,qt1_eval,t2,qt2_eval,t3,qt3_eval);grid; title('position');xlabel('time [s]');ylabel('[rad]')
hold off

figure
hold on
plot(t1,qt1_dot_eval,t2,qt2_dot_eval,t3,qt3_dot_eval);grid; title('velocity');xlabel('time [s]');ylabel('[rad/s]')
hold off

figure
hold on
plot(t1,qt1_ddot_eval,t2,qt2_ddot_eval,t3,qt3_ddot_eval);grid; title('acceleration');xlabel('time [s]');ylabel('[rad/s^2]')
hold off

grid on