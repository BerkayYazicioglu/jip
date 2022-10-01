%% Environment to survey in
% x,y is defined on the lower left corner on the boundary

classdef Environment < handle
    %% Variables
    properties
        V                 % 2D potential map 
        X,Y               % 2D coordinate map
        agents            % cell array of agents on the field
        t                 % time (s)
        potentials        % an array of Potential objects 
        calc_V = true;    % bool for numerically calculating V
        resolution = 0.1; % m
    end
    
    %% Methods
    methods
        %% Constructor
        function obj = Environment(L, bounds, eta_bound, rho_bound)
            obj.agents = {};
            obj.potentials = {};
            obj.t = 0;

            % calculate potential field (might be inefficient)
            if obj.calc_V
                n = floor(L/obj.resolution);
                [obj.X, obj.Y] = meshgrid(linspace(0, L(1), n(1)), ...
                                          linspace(L(2), 0, n(2)));  
                obj.V = zeros(length(obj.X), length(obj.Y));
                % add boundaries
                for i = 1:length(bounds)
                    obj.add_potential(i, bounds{i}, 'linear', ...
                                       eta_bound, rho_bound, ...
                                       'boundary');
                end
            end
        end

        %% Add agent to the field
        function add_agent(obj, agent, pos, orientation)
            agent.pos = pos;
            agent.orientation = orientation;
            agent.potential.points = [pos(1) pos(2)];
            obj.agents{end+1} = agent;
        end
        
        %% Add new potential to the environment
        function add_potential(obj, id, points, interp, eta, rho, type)
            P = Potential(id, points, interp, eta, rho, type);
            obj.potentials{end+1} = P;
            
            % add to the static potential field
            if obj.calc_V && ~isequal(P.type, 'agent')
                obj.V = P.add_to_field(obj.V, obj.X, obj.Y);
            end
        end
        
        %% Plot agents with forces
        function plot(obj)
            cla;
            drawArrow = @(x,y,varargin) quiver(x(1),y(1),x(2)-x(1),y(2)-y(1),0, varargin{:});
            % plot obstacles and boundaries
            for i = 1:length(obj.potentials)
                P = obj.potentials{i};
                if isequal(P.interp, 'linear')
                    if isequal(P.type, 'obstacle')
                        patch(P.points(:,1), P.points(:,2), 'black'); hold on;
                    else
                        line(P.points(:,1), P.points(:,2), ...
                            'Color', 'k', 'LineWidth', 3); hold on;
                    end
                else
                    patch(P.points(:,1), P.points(:,2), 'black'); hold on;
                end
            end
            % plot agents and acting forces
            for i = 1:length(obj.agents)
                agent = obj.agents{i};
                plot(agent.pos(1), agent.pos(2), '.r', 'MarkerSize', 50); hold on;
                % arrow for the net force
                F = agent.Fnet + agent.pos(1:2, :);
                x = [agent.pos(1) F(1)];
                y = [agent.pos(2) F(2)];
                drawArrow(x, y, 'LineWidth', 1, 'MaxHeadSize', 5, ...
                                'AutoScale','off', 'Color','k'); hold on
                % arrow for the heading
                x = [agent.pos(1) agent.pos(1) + cosd(agent.orientation(1))];
                y = [agent.pos(2) agent.pos(2) + sind(agent.orientation(1))];
                drawArrow(x, y, 'LineWidth', 3, 'MaxHeadSize', 5, ...
                                'AutoScale','off', 'Color','r'); hold on
            end
            if obj.calc_V
                % dynamically add agent potentials
                V_0 = obj.V;
                for i = 1:length(obj.agents)
                    V_0 = obj.agents{i}.potential.add_to_field(V_0, obj.X, obj.Y);
                end
                % plot potential field
                surf(obj.X, obj.Y, V_0 - max(V_0, [], 'all'), ...
                    'FaceAlpha', 0.6, 'EdgeColor', 'none'); hold off;
                xlim([0 max(obj.X, [], 'all')]);
                ylim([0 max(obj.Y, [], 'all')]);
            end
        end
    end
end