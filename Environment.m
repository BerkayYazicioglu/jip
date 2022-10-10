%% Environment to survey in
% x,y is defined on the lower left corner on the boundary

classdef Environment < handle
    %% Variables
    properties
        L                 % dimensions (m)
        V                 % 2D potential map 
        X,Y               % 2D coordinate map
        agents            % cell array of agents on the field
        t                 % time (s)
        potentials        % an array of Potential objects 
        lines             % line segments denoting the environment layout
        eta_a, rho_a
        resolution = 0.1; % (m)
    end
    
    %% Methods
    methods
        %% Constructor
        % eta_a, rho_a -> agent properties
        function obj = Environment(L, lines, eta_a, rho_a)
            obj.lines = lines;
            obj.agents = {};
            obj.potentials = {};
            obj.t = 0;
            obj.L = L;
            obj.eta_a = eta_a;
            obj.rho_a = rho_a;

            % calculate potential field 
            n = floor(L/obj.resolution);
            [obj.X, obj.Y] = meshgrid(linspace(0, L(1), n(1)), ...
                                      linspace(L(2), 0, n(2)));  
            obj.V = zeros(length(obj.X), length(obj.Y));
        end

        %% Add agent to the field
        function add_agent(obj, agent, pos, orientation)
            agent.pos = pos;
            agent.orientation = orientation;
            obj.agents{end+1} = agent;
        end
        
        %% Add new potential to the environment
        function add_potential(obj, id, points, eta, rho, type)
            P = Potential(id, points, eta, rho, type);
            obj.potentials{end+1} = P;
            obj.V = P.add_to_field(obj.V, obj.X, obj.Y);
        end
        
        %% Plot agents with forces
        function animate(obj, fig, ax)
            cla(ax);
            % plot obstacles and boundaries
            for i = 1:length(obj.potentials)
                P = obj.potentials{i};
                if isequal(P.type, 'boundary')
                    line(ax, P.points(:,1), P.points(:,2), ...
                        'Color', 'k', 'LineWidth', 3);
                    hold(ax, 'on');
                else
                    patch(ax, P.points(:,1), P.points(:,2), 'black'); 
                    hold(ax, 'on');
                end
            end
            
            V_0 = obj.V;
            % plot agents and acting forces
            for i = 1:length(obj.agents)
                % plot agent and its goal
                agent = obj.agents{i};
                plot(ax, agent.pos(1), agent.pos(2), '.r', 'MarkerSize', 50); 
                hold(ax, 'on');
                plot(ax, agent.goal(1), agent.goal(2), '*r', 'MarkerSize', 8);
                hold(ax, 'on');
                % sensor radius
                laser = agent.laser_beam;
                x0 = agent.pos(1) ...
                   + laser.placement(1) * cosd(agent.orientation(1)) ... 
                   - laser.placement(2) * sind(agent.orientation(1));
                y0 = agent.pos(2) ...
                   + laser.placement(1) * sind(agent.orientation(1)) ... 
                   + laser.placement(2) * cosd(agent.orientation(1));
                angle = agent.orientation(1) ...
                      + laser.placement(3) ...
                      + linspace(laser.angle_range(1), laser.angle_range(2), 20);
                
                x = x0 + laser.max_range .* cosd(angle);
                y = y0 + laser.max_range .* sind(angle);
                
                patch(ax, 'XData', [x0; x(:); x0], ...       
                          'YData', [y0; y(:); y0], ...
                          'EdgeColor', 'green', ...
                          'FaceColor', 'green', ...
                          'FaceAlpha', 0.3);
                hold(ax, 'on');

                % arrow for the heading
                x = [agent.pos(1) agent.pos(1) + cosd(agent.orientation(1))];
                y = [agent.pos(2) agent.pos(2) + sind(agent.orientation(1))];
                drawArrow(fig,ax, x, y, [0 obj.L(1)], [0 obj.L(2)], ...
                                {'LineWidth', 1, 'Color','r'}); 
                hold(ax, 'on');
                
                % dynamically add agent potentials
                agent_pot = Potential(i, agent.pos(1:2)', obj.eta_a, obj.rho_a, 'agent');
                V_0 = agent_pot.add_to_field(V_0, obj.X, obj.Y);
            end

            % plot potential field
            surf(ax, obj.X, obj.Y, V_0 - max(V_0, [], 'all'), ...
                'FaceAlpha', 0.6, 'EdgeColor', 'none'); 
            hold(ax, 'off');
            xlim(ax, [0 obj.L(1)]);
            ylim(ax, [0 obj.L(2)]);
        end

        %% Plot agent laser readings
        function plot_laser_measurements(obj, agent_idx, fig, ax)
            cla(ax);
            agent = obj.agents{agent_idx};
            angles = wrapTo180(agent.laser_beam.theta_laser + agent.laser_beam.placement(3));
            stem(ax, angles, agent.laser_beam.measurements, ...
                'filled', 'Marker', '.', 'Color', 'k', 'LineWidth', 2);
            yline(ax, agent.laser_thresh);
            ylim(ax, [0 agent.laser_range]);
            axis(ax, 'tight');
        end

        %% Plot potential calculations
        function plot_laser_potentials(obj, agent_idx, fig, ax)
            cla(ax);
            agent = obj.agents{agent_idx};
            angles = wrapTo180(agent.laser_beam.theta_laser + agent.laser_beam.placement(3));
            [~, idx] = min(agent.f_total);
            
            plot(ax, angles, agent.f_att, 'Color', 'b'); hold(ax, 'on');
            plot(ax, angles, agent.f_rep, 'Color', 'r'); hold(ax, 'on');
            plot(ax, angles, agent.f_total, 'Color', 'k'); hold(ax, 'on');
            xline(ax, angles(idx)); 
            hold(ax, 'off'); 
            grid(ax, 'on');
            axis(ax, 'tight');
        end
    end
end