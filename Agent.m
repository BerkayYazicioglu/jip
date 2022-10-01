%% Modelling the agent properties
% Assumptions:
% 1- locations are known at all times
% 2- no measurement errors
% 3- search area starts by (0, 0) coordinates on left bottom [x,y]

classdef Agent < handle
    %% Variables
    properties
        id           % agent id
        v            % linear velocity
        w            % angular velocity
        pos          % position [x; y; z]
        d            % radius of POV
        potential    % potential of the agent
        orientation  % attitude (euler angles?)
        neighbors    % neighboring agents
        sample_freq  % sensor sampling Hz
        dt           % control period
        t            % time (s)
        map          % Map instance 
        Fnet         % net virtual forces on the agent
    end
    %% Constants
    properties(Constant)
        max_v = 3            % m/s
        comm_range = inf     % m
        neighbor_range = inf % m
        pot_range = inf      % m
        diameter = 0.25      % m
    end

    %% Methods
    methods
        %% Constructor
        function obj = Agent(id, init_pot, sample_freq, dt, eta, rho)
            obj.id = id;
            obj.sample_freq = sample_freq;
            obj.potential = Potential(id, [], 'linear', eta, rho, 'agent');
            obj.dt = dt;
            obj.t = 0;
            obj.w = 0;
            obj.v = 0;
            obj.map = Map(init_pot);
            obj.neighbors = {};
        end

        %% Update neighbor positions
        function update_neighbors(obj, agent_list)
            obj.neighbors = {};
            for agent = agent_list
                if agent.id ~= obj.id
                    if sum((agent.pos - obj.pos).^2)^0.5 <= obj.neighbor_range
                        obj.neighbors{end+1} = agent;
                    end
                end
            end
        end
        
        %% Move agent (WIP, very simple position change for now)
        function control(obj, new_pos)
            vec = new_pos - obj.pos;
            obj.pos = new_pos;
            obj.orientation(1) = atan2d(vec(2), vec(1));
            
            obj.t = obj.t + obj.dt;
            obj.potential.points = [obj.pos(1) obj.pos(2)];
        end
        
        %% Calculate forces acting on the robot
        % potential spikes are truncated 
        % angle at NaN is the reverse of current heading angle

        function Fnet = calculate_forces(obj)
            % can add a limit on the number of boundaries considered
            % calculate distances to the points
            x = obj.pos(1);
            y = obj.pos(2);
            distances = [];
            magnitudes = [];
            p0 = [];
            Fnet = 0;
            max_mag = 0;
            
            % include neighbor agent potentials
            potentials = obj.map.potentials;
            for i = 1:length(obj.neighbors)
                potentials{end+1} = obj.neighbors{i}.potential;
            end

            for i = 1:length(potentials)
                P = potentials{i};
                [xy, dist, ~] = distance2curve(P.points, [x y], P.interp);
                % look for inside potential detection range
                if (dist < P.rho) && (dist < obj.pot_range)
                    distances = [distances ; dist];
                    p0 = [p0 ; xy];
                    magnitudes = [magnitudes ; P.eta/dist^2 * (1/dist - 1/P.rho)];
                    % correction for potential spikes 
                    % sooo, maybe for 0 <= rho < 1?
                    max_mag = max(max_mag, P.eta * (1 - 1/P.rho));
                end
            end
            
            if ~isempty(distances) 
                % add angles
                vx = (x - p0(:,1)) ./ ...
                    ((x - p0(:,1)).^2 + (y - p0(:,2)).^2).^0.5;
    
                vy = (y - p0(:,2)) ./ ...
                    ((x - p0(:,1)).^2 + (y - p0(:,2)).^2).^0.5;
    
                % find NaN angles ¯\_(ツ)_/¯ and reverse direction?
                vx(isnan(vx)) = cosd(180 - obj.orientation(1));
                vy(isnan(vy)) = sind(180 - obj.orientation(1));
    
                Fx = magnitudes .* vx;
                Fy = magnitudes .* vy;
                Fnet = [sum(Fx) ; sum(Fy)];
                Fnet(isnan(Fnet)) = inf;
               
                if abs(Fnet) > max_mag
                    Fnet = sign(Fnet) * max_mag;
                end
            end 
            obj.Fnet = Fnet;
        end 
    end
end