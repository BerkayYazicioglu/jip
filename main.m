%% main
clc;
clear;
%close;

%% initializations
use_saved = false; % use saved workspace 

filename = 'ground.mat'; % sim env save file

if use_saved == false
    create_setup(filename)
end

load('ground.mat');

%% testing
% agents
ids = {'r1'};
sample_freq = 10; % Hz
dt = 0.1; % s

% pos  r1
pos = [8;   % x (m)
       2;]; % y (m) 

% euler angle   r1 
orientations = [90; % yaw (degrees)
                0;  % pitch (degrees)    
                0]; % roll (degrees) 

robot1 = Agent(ids{1}, sample_freq, dt);
ground.add_agent(robot1, pos(:,1), orientations(:,1));

% set goal
goal = [18;
        19];   

steps = 100;

ground.agents{1}.set_goal(goal);

%% plots

fig_pot = figure(1);
ax_pot = axes('Parent', fig_pot);

fig_plots = figure(2);
ax_laser_m = axes('Parent', fig_plots);
ax_laser_p = axes('Parent', fig_plots);

ax_laser_m = subplot(2, 1, 1, ax_laser_m);
ax_laser_p = subplot(2, 1, 2, ax_laser_p);

%view(ax_pot, [0 -1 5]);
view(ax_pot, 2);

pause(2);
for k = 1:length(ground.agents)
    for i = 1:steps
        ground.agents{k}.calculate_forces(ground);
    
        ground.plot_laser_measurements(k, fig_plots, ax_laser_m);
        ground.plot_laser_potentials(k, fig_plots, ax_laser_p);
        ground.animate(fig_pot, ax_pot);
    
        ground.agents{k}.control();
        pause(0.2);
    end
    
    ground.agents{k}.calculate_forces(ground);
       
    ground.plot_laser_measurements(k, fig_plots, ax_laser_m);
    ground.plot_laser_potentials(k, fig_plots, ax_laser_p);
    ground.animate(fig_pot, ax_pot);
end