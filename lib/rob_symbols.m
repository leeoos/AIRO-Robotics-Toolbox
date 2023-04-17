%% SYMBOLIC VARIABLES

function void = rob_symbols(args)
    args = split(args);
    eval(args(1))
    eval(args(2));
    
    % Joints
    syms q [1 N]
    syms q_dot [1 N]
    syms q_d_dot [1 N]
    
    % Kinematics parameters
    syms l [1 N]
    syms L [1 N]
    
    % Dynamic parameters
    syms g0
    syms m [1 N]
    
    % Inertia Matrices for single links
    % X
    syms Ixx [1 N]
    syms Ixy [1 N] 
    syms Ixz [1 N]
    % Y
    syms Iyx [1 N]
    syms Iyy [1 N]
    syms Iyz [1 N]
    % Z
    syms Izz [1 N]
    syms Izx [1 N]
    syms Izy [1 N]
    
    I = cell(1,3);
    if (I_diag)
        for i =(1:N)
            I{i} = [
                Ixx(i), 0, 0;
                0, Iyy(i), 0;
                0, 0, Izz(i);
            ];
        end
    else 
        for i =(1:N)
            I{i} = [
                Ixx(i), Ixy(i), Ixz(i);
                Ixy(i), Iyy(i), Iyz(i);
                Ixz(i), Iyz(i), Izz(i);
            ];
        end
    end

end



