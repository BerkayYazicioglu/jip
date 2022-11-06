%% main
clc;
clear all;
close all;

%% initializations
use_saved = true; % use saved workspace 

filename = 'ground.mat'; % sim env save file

if use_saved == false
    create_setup(filename)
end

load('ground.mat');


%% testing
% agents
robots = {};

ids = {'r1', 'r2', 'r3'};

sample_freq = 10; % Hz
dt = 0.1; % s
steps = 300;

% pos   x  y
pos = [1  1;  
       1  49;
       49  1;
       49  49]; 

% euler angle   r1   r2   r3
orientations = [0    0   0   0  0]; 

for i = 1:length(ids)
    robots{i} = Agent(ids{i}, ground.L, sample_freq, dt);
    ground.add_agent(robots{i}, pos(i, :)', orientations(i));
    ground.agents{i}.pos_log = [pos(i, :)'];
    %ground.agents{i}.goal = goals(i, :)';
end

%% plots

fig = figure(1);
ax = {};
for i = 1:4
    ax{i} = axes('Parent', fig);
end
ax{1} = subplot(3, 4, [1 2 5 6], ax{1});
ax{2} = subplot(3, 4, [3 4 7 8], ax{2});
ax{3} = subplot(3, 4, [9 10], ax{3});
ax{4} = subplot(3, 4, [11 12], ax{4});


simulate(ax, ground, steps);

final_map = ground.agents{1}.heatmap;
explored = checkOccupancy(final_map) ~= -1;
[frontiers, ~, ~, ~] = bwboundaries(explored, 4, 'noholes');

% preprocessing, bunch of edge cases because of bwboundaries
% ¯\_(ツ)_/¯
for k = 1:length(frontiers)
    frontiers{k}(:, 1) = 200 - frontiers{k}(:, 1);
    frontiers{k} = fliplr(frontiers{k});
    frontiers{k} = frontiers{k} ./ 4;
end
f = frontiers;
frontiers = cell2mat(frontiers); % world coordinates


polyarea(frontiers(:,1), frontiers(:,2))


