% Iterative learning values for PID 

syms theta(t) I m d g

% I_0 = 0;
% 
% m = 10;
% 
% g_0 = 9.81;
% 
% d = 1;
% 
% Kp = 500;
% 
% Kd = 45;
% 
% gm = 1;


u = (I + m*d^2)*diff(theta(t),t,2) + m*d*g*sin(theta(t))


sys = tf(1,u)