syms phi

R = [cos(phi), -sin(phi), 0;
      sin(phi),  cos(phi), 0
         0,          0,    1]

w = [1;1;1]

diff_rotation_matrix(R,w)



M = [
    Ixx3 + Iyy2 + Izz1 + L1^2*m2 + L1^2*m3 + d1^2*m1 - Ixx3*cos(q3)^2 + Iyy3*cos(q3)^2 + d3^2*m3*cos(q3)^2 + 2*L1*d3*m3*cos(q2)*cos(q3), Ixx3 + Iyy2 - Ixx3*cos(q3)^2 + Iyy3*cos(q3)^2 + d3^2*m3*cos(q3)^2 + L1*d3*m3*cos(q2)*cos(q3), -L1*d3*m3*sin(q2)*sin(q3);
                                       Ixx3 + Iyy2 - Ixx3*cos(q3)^2 + Iyy3*cos(q3)^2 + d3^2*m3*cos(q3)^2 + L1*d3*m3*cos(q2)*cos(q3),                            Ixx3 + Iyy2 - Ixx3*cos(q3)^2 + Iyy3*cos(q3)^2 + d3^2*m3*cos(q3)^2,                         0;
                                                                                                          -L1*d3*m3*sin(q2)*sin(q3),                                                                                            0,            m3*d3^2 + Izz3]

function R_dot = diff_rotation_matrix(R, w)

    syms wx wy wz
    w_sym = [wx;wy;wz];
    
    Sw = [  
            0, -wz, wy; 
            wz, 0, -wx; 
            -wy, wx,  0
    ];
    Sw = subs(Sw, w_sym, w);
    R_dot = Sw * R;
end