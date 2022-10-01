%% main
clc;
clear;
close;

%% initializations
sample_freq = 10; % Hz
dt = 1; % s

rho_b = 5;  % (m) boundaries
eta_b = 10; % (m) boundaries
rho = 2;    % (m) agent field radius
eta = 10;    % (m) agent eta

L = [20 20]; % x y (m) search area limits
% pos  r1
pos = [0;  % x (m)
       0;  % y (m)    
       0]; % z (m) 
ids = {'r1'};
% euler angle   r1 
orientations = [90; % yaw (degrees)
                0;  % pitch (degrees)    
                0]; % roll (degrees) 

% search area boundaries
bounds = {[ 0    0  ; L(1)  0  ], ... 
          [L(1)  0  ; L(1) L(2)], ...
          [L(1) L(2);  0   L(2)], ...
          [ 0   L(2);  0    0  ]};

%% simulation
ground = Environment(L, bounds, eta_b, rho_b);

robot1 = Agent(ids{1}, ground.potentials, sample_freq, dt, eta, rho);

ground.add_agent(robot1, pos(:,1), orientations(:,1));

%% plotting / testing

% add new obstacle in the middle
points = [9 10; 10 9; 11 10; 10 11];
interp = 'linear';
type = 'obstacle';
eta_o = 10;
rho_o = 4;

ground.add_potential(1, points, interp, eta_o, rho_o, type);
ground.agents{1}.map.add_potential(1, points, interp, eta_o, rho_o, type);

% go along the diagonal
points = [5 0 ; % start
          15 15];   % end
res = 50;

x = linspace(points(1,1), points(2,1), res);
y = linspace(points(1,2), points(2,2), res);

for i = 1:res
    F = ground.agents{1}.calculate_forces();
    ground.plot();
    ground.agents{1}.control([x(i); y(i); 0]);
    pause(0.1);
end

