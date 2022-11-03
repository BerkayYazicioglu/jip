%% Environment to survey in
% x,y is defined on the lower left corner on the boundary

classdef Environment < handle
    %% Variables
    properties
        L                 % dimensions
        agents            % cell array of agents on the field
        t                 % time (s)
        obstacles         % line segments denoting the environment layout
        boundaries        % line segments defining boundaries
        lines             % combined obstacles and boundaries
    end
    
    %% Methods
    methods
        %% Constructor
        % eta_a, rho_a -> agent properties
        function obj = Environment(L, obstacles, boundaries)
            obj.obstacles = obstacles;
            obj.boundaries = boundaries;
            lines = [];
            for j = 1:length(obstacles)
                lines = [lines; obstacles{j}; NaN NaN];
            end
            for j = 1:length(boundaries)
                lines = [lines; boundaries{j}; NaN NaN];
            end
            obj.lines = lines;
            
            obj.agents = {};
            obj.t = 0;
            obj.L = L;         
        end

        %% Add agent to the field
        function add_agent(obj, agent, pos, heading)
            agent.pos = pos;
            agent.heading = heading;
            obj.agents{end+1} = agent;
        end
        
        %% Plot agents with forces
        function animate(obj, ax)
            persistent h_animate;
            if nargin > 1
                hold(ax, 'on');
                % plot obstacles and boundaries
                for i = 1:length(obj.boundaries)
                    line(ax, obj.boundaries{i}(:,1), obj.boundaries{i}(:,2), 'Color', 'k', 'LineWidth', 3);
                end 
                for i = 1:length(obj.obstacles)
                    patch(ax, obj.obstacles{i}(:,1), obj.obstacles{i}(:,2), 'black'); 
                end
                xlim(ax, [0 obj.L(1)]);
                ylim(ax, [0 obj.L(2)]);
                
                h_animate = {};
                C = {'k','b','r','g','y',[.5 .6 .7],[.8 .2 .6]};

                for i = 1:length(obj.agents)
                    h_animate{i} = struct();
                    agent = obj.agents{i};
                    h_animate{i}.pos = plot(ax, agent.pos(1), agent.pos(2), ...
                                            'Color', C{mod(i, length(C)) + 1}, ...
                                            'Marker', '.', 'MarkerSize', 50); 
                    h_animate{i}.goal = plot(ax, agent.goal(1), agent.goal(2), ...
                                            'Color', C{mod(i, length(C)) + 1}, ...
                                            'Marker', '*', 'MarkerSize', 10);
                    h_animate{i}.laser = patch(ax, 'XData', 0, 'YData', 0, ...
                                                   'EdgeColor', 'green', ...
                                                   'FaceColor', 'green', ...
                                                   'FaceAlpha', 0.3);
                end
                hold(ax, 'off');
            end
            
            % plot agents and acting forces
            for i = 1:length(obj.agents)
                % plot agent and its goal
                agent = obj.agents{i};
                set(h_animate{i}.pos, 'XData', agent.pos(1), 'YData', agent.pos(2));
                set(h_animate{i}.goal, 'XData', agent.goal(1), 'YData', agent.goal(2));
                % sensor radius
                laser = agent.laser_beam;
                x0 = agent.pos(1) ...
                   + laser.placement(1) * cosd(agent.heading) ... 
                   - laser.placement(2) * sind(agent.heading);
                y0 = agent.pos(2) ...
                   + laser.placement(1) * sind(agent.heading) ... 
                   + laser.placement(2) * cosd(agent.heading);
                angle = agent.heading ...
                      + laser.placement(3) ...
                      + linspace(laser.angle_range(1), laser.angle_range(2), 20);
                
                x = x0 + laser.max_range .* cosd(angle);
                y = y0 + laser.max_range .* sind(angle);
                
                set(h_animate{i}.laser, 'XData', [x0; x(:); x0], ...       
                                        'YData', [y0; y(:); y0]);
            end
        end

        %% Plot agent laser readings
        function plot_laser_measurements(obj, agent_idx, ax)
            persistent h_dist;
            if nargin > 2
                agent = obj.agents{agent_idx};
                h_dist = stem(ax, 0, 0, 'filled', 'Marker', '.', 'Color', 'k', 'LineWidth', 2);
                yline(ax, agent.laser_thresh);
                ylim(ax, [0 agent.laser_range]);
                title(ax, sprintf('Agent %s laser beam measurements', agent.id));
                xlabel(ax, 'Angles (degrees)');
                ylabel(ax, 'Distances (m)');
                axis(ax, 'tight');
            end
            agent = obj.agents{agent_idx};
            angles = wrapTo180(agent.laser_beam.theta_laser + agent.laser_beam.placement(3));
            set(h_dist, 'XData', angles, 'YData', agent.laser_beam.measurements);
        end

        %% Plot potential calculations
        function plot_laser_potentials(obj, agent_idx, ax)
            persistent h_att h_rep h_tot h_opt;
            if nargin > 2
                agent = obj.agents{agent_idx};
                hold(ax, 'on');
                h_att = plot(ax, 0, 0, 'Color', 'b'); 
                h_rep = plot(ax, 0, 0, 'Color', 'r');
                h_tot = plot(ax, 0, 0, 'Color', 'k'); 
                h_opt = xline(ax, 0);
                title(ax, sprintf('Agent %s potential fields', agent.id));
                xlabel(ax, 'Angles (degrees)');
                ylabel(ax, 'Likelihood estimation');
                grid(ax, 'on'); axis(ax, 'tight');
                hold(ax, 'off');
            end
            agent = obj.agents{agent_idx};
            angles = wrapTo180(agent.laser_beam.theta_laser + agent.laser_beam.placement(3));
            [~, idx] = min(agent.f_total);
            set(h_att, 'XData', angles, 'YData', agent.f_att);
            set(h_rep, 'XData', angles, 'YData', agent.f_rep);
            set(h_tot, 'XData', angles, 'YData', agent.f_total);
            set(h_opt, 'Value', angles(idx));
        end

        %% Plot Occupancy Grid updates
        function plot_occupancy_grid(obj, agent_idx, ax)
            cla(ax)
            agent = obj.agents{agent_idx};
            show(agent.map, 'Parent', ax, 'FastUpdate', 1);
            hold(ax, 'on');
            for k = 1:length(obj.agents{agent_idx}.f)
               boundary = obj.agents{agent_idx}.f{k};
               plot(ax, boundary(:, 1), boundary(:, 2), 'r.', 'LineWidth', 2);
            end
            plot(ax, agent.goal(1), agent.goal(2), 'r*', 'MarkerSize', 10);
            hold(ax, 'off');
        end       
    end
end