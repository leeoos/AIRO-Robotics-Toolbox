%% DATASHEET FOR A PLANAR 4R 
% Insert in the following file all the inputs necessary to compute the
% dynamic model of a planar 4R spatial robot

% Direct kinematics.
DHTABLE = [        
    0   L1   0   q1;
    0   L2   0   q2;
    0   L3   0   q3;
    0   L4   0   q4;
];

% Vectors of CoM relative to Reference Frame i
R_CoM = {
    [
        -L1+d1;
        0;
        0;
        1;
    ], ... 
    [
        -L2+d2;
        0;
        0;
        1;
    ], ...
    [
        -L3+d3;
        0;
        0;
        1;
    ], ... 
    [
        -L4+d4;
        0;
        0;
        1;
    ]
};
