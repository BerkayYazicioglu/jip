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
robots = {};

ids = {'r1'};

sample_freq = 10; % Hz
dt = 0.1; % s
steps = 100;

% pos   x  y
pos = [10  1 ;  
       2  10 ]; 

% goal  x   y
goal = [10 19;
        19 10];   

% euler angle   yaw  pitch roll
orientations = [90    0     0; 
                0     0     0]; 

for i = 1:length(ids)
    robots{i} = Agent(ids{i}, sample_freq, dt);
    ground.add_agent(robots{i}, pos(i, :)', orientations(i, :)');
    ground.agents{i}.set_goal(goal(i, :)');
end

%% plots

fig = figure(1);
ax = {};
for i = 1:3
    ax{i} = axes('Parent', fig);
end
ax{1} = subplot(2, 2, [1 3], ax{1});
ax{2} = subplot(2, 2, 2, ax{2});
ax{3} = subplot(2, 2, 4, ax{3});

view(ax{1}, [0 -1 5]);
%view(ax{1}, 2);
pause(10);

for i = 1:steps+1
    for k = 1:length(ground.agents)
        ground.agents{k}.update_neighbors(ground.agents);
        ground.agents{k}.calculate_forces(ground);
        
        ground.animate(fig, ax{1});
        ground.plot_laser_measurements(1, fig, ax{2});
        ground.plot_laser_potentials(1, fig, ax{3});

        if i <= steps
            ground.agents{k}.control();
        end
        pause(0.001);
    end
end