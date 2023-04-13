syms phi

R = [cos(phi), -sin(phi), 0;
      sin(phi),  cos(phi), 0
         0,          0,    1]

w = [1;1;1]

diff_rotation_matrix(R,w)

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