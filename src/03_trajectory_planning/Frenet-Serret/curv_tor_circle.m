%% Sopravvivenza_AIRO! 
% Author: Massimo, Miryam, Leonardo, Federico, Francesco, Paolo

% This script given the parametrization of a curve in symbolic form, give
% as output the computation of curvature k(s) and torsion t(s) of a curve

syms s r x y

%% PARAMETRIZATION OF THE CURVE

% Parametrization of an helix (!! WRITE YOUR PARAMETRIZATION!!)

disp ('This is the curve :)')

x = r*cos(s);
y = r*sin(s);
z = 0; 

p = transpose([x,y,z])

%% COMPUTE THE CURVATURE

p_der_1 = diff(p,s);

p_der_2 = diff(p_der_1,s);

vect_prod = cross(p_der_1, p_der_2);

vect_prod_2 = simplify(vect_prod);

k_num = sqrt((vect_prod_2(1))^2 + (vect_prod_2(2))^2 + (vect_prod_2(3))^2);

k_num = simplify(k_num);

k_den = sqrt( (p_der_1(1))^2 + (p_der_1(2))^2 + (p_der_1(3))^2);

k_den = simplify(k_den);
 
disp ('This is the curvature of this curve :)')

k_s = simplify(k_num/(k_den)^3)

%% COMPUTE TORSION

p_der_3 = diff(p_der_2, s);

vect_prod_2 = cross(p_der_2, p_der_3);

t_num = (p_der_1(1)*vect_prod_2(1)) + (p_der_1(2)*vect_prod_2(2)) + (p_der_1(3)*vect_prod_2(3));

t_num = simplify(t_num);

t_den = k_num^2;

disp ('This is the torsion of the curve :)')

t_s = simplify(t_num/t_den)




