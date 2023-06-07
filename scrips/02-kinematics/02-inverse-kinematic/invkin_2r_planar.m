
% You must be change this values with your data problem
Px = 0.37;
Py = 0.67;
l1 = 0.5;
l2 = 0.4;

%% COMPUTATION OF q2

c2 = (Px^2 + Py^2 - (l1^2 + l2^2))/(2*l1*l2)

if c2>=-1 && c2<=1
    s2_1 = +sqrt(1-c2^2);
    s2_2 = -sqrt(1-c2^2);

    q2_1 = atan2(s2_1,c2);
    q2_2 = atan2(s2_2,c2);
else
    ('Impossible task for your planar Robot, pay attention!')
    return
end


%% COMPUTATION OF q1
det = l1^2+l2^2+2*l1*l2*c2;
eps = 10^-10

if det>=-eps && det<=eps
    disp('Singular case: infinity to the power of 1 solutions')
    disp('This means that q1 is every value!')
else
    s1_1 = (Py*(l1+l2*c2) - Px*l2*s2_1)/det;
    s1_2 = (Py*(l1+l2*c2) - Px*l2*s2_2)/det;
    c1_1 = (Px*(l1+l2*c2) + Py*l2*s2_1)/det;
    c1_2 = (Px*(l1+l2*c2) + Py*l2*s2_2)/det;
    
    q1_1 = atan2(s1_1,c1_1);
    q1_2 = atan2 (s1_2, c1_2);
end

fprintf("This is the first solution: \n")

q_first = [q1_1;q2_1] 

fprintf("This is the second solution: \n")

q_second = [q1_2;q2_2] 

