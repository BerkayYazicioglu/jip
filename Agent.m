%% Modelling the agent properties
% Assumptions:
% 1- locations are known at all times
% 2- no measurement errors
% 3- body frame, moving pos is x axis

classdef Agent < handle
    %% Variables
    properties
        id           % agent id
        v            % linear velocity
        w            % angular velocity
        pos          % position [x; y]
        goal         % (x; y) 
        heading      % attitude 
        neighbors    % neighboring agents

        sample_freq  % sensor sampling Hz
        dt           % control period
        t            % time (s)

        f_rep        % repulsive field wrt agent frame
        f_att        % attractive field wrt agent frame
        f_total      % f_rep + f_att
        angle_net    % angle correction calculated from the net virtual forces 

        laser_beam   % 2d laser beam sensor 
        map          % 2d occupancy map 
        heatmap      % 2d exploration heatmap
        
        f            % BSO frontiers
        clusters     % BSO clusters
        w_clusters   % BSO cluster weights
    end
    %% Constants
    properties(Constant)
        max_v = 2                  % m/s
        max_w = 90                 % degrees/s
        
        comm_range = inf           % m
        neighbor_range = inf       % m
        
        diameter = 0.25            % m
        
        laser_pos = [0 0 0];       % body frame, (x y theta)
        laser_angle = [-90 90];    % angle range of the laser beam (degrees)
        laser_K = 101;             % number of beams on the laser
        laser_range = 5;           % range of the laser beam (m)
        laser_thresh = 2;          % threshold of calculating force (m)

        detect_thresh = 5;         % sherd detection radius (m)
        
        gamma = 1;                 % attractive field gain
        goal_error = 1;            % margin of error for reaching a goal (m)
        
        fov = 4;                   % occupancy grid field of vision
    end

    %% Methods
    methods
        %% Constructor
        function obj = Agent(id, grid_dims, sample_freq, dt)
            obj.id = id;
            obj.sample_freq = sample_freq;
            obj.dt = dt;
            obj.t = 0;
            obj.w = 0;
            obj.v = 0;
            obj.neighbors = {};
            obj.f = {};
            obj.clusters = [];
            obj.w_clusters = [];

            obj.laser_beam = Laser(obj.laser_pos, obj.laser_angle, ...
                                   obj.laser_K, obj.laser_range);
            obj.map = occupancyMap(grid_dims(1), grid_dims(2), obj.fov);

            obj.heatmap = occupancyMap(grid_dims(1), grid_dims(2), obj.fov);
        end
        
        %% Update neighbor positions
        function update_neighbors(obj, agent_list)
            obj.neighbors = {};
            for i = 1:length(agent_list)
                agent = agent_list{i};
                if strcmp(agent.id, obj.id) == false
                    if sum((agent.pos - obj.pos).^2)^0.5 <= obj.neighbor_range
                        obj.neighbors{end+1} = agent;
                        syncWith(obj.map, agent.map);
                        syncWith(obj.heatmap, agent.heatmap);
                    end
                end
            end
        end
        
        %% Move agent (WIP, very simple angle change for now)
        function control(obj)
            d = obj.max_v * obj.dt;
            % check if at the goal
            e = sum((obj.goal - obj.pos).^2);
            
            if e > obj.goal_error
                % assume instant heading change
                correction = obj.angle_net;
%                 if abs(correction) >= obj.max_w * obj.dt
%                     correction = sign(correction) * obj.max_w * obj.dt;
%                 end
                obj.heading = wrapTo180(obj.heading + correction);
                vec = [d * cosd(obj.heading);
                       d * sind(obj.heading)];
    
                obj.pos = obj.pos + vec;
            end

            obj.t = obj.t + obj.dt;
        end
        
        %% Set goal
        % BSO algorithm 

        function set_goal(obj)
            % detect the frontiers
            explored = checkOccupancy(obj.heatmap) ~= 0;
            frontiers = bwboundaries(explored, 8, 'noholes');
            d = [];
            for k = 1:length(frontiers)
                frontiers{k}(:, 1) = obj.map.GridSize(2) - frontiers{k}(:, 1);
                frontiers{k} = fliplr(frontiers{k});
                frontiers{k} = frontiers{k} ./ obj.fov;
                frontiers{k} = frontiers{k}(all(frontiers{k}, 2),:);
            end
            obj.f = frontiers;
            frontiers = cell2mat(frontiers); % world coordinates

            N_ = 20;
            p_branch = 0.5;
            p_lvr = 0.5;

            % n closest frontiers as the first cluster
            p = repmat(obj.pos', [size(frontiers, 1) 1]);
            d2 = sqrt(sum((frontiers - p).^2, 2));

            mask = d2 >= obj.detect_thresh;
            d2 = d2(mask);
            frontiers = frontiers(mask, :);

            % TODO - add weights wrt. how far away clusters are from the
            % detected sherds

            w_frontiers = 1 ./ d2;
            [w_candidates, idx] = sort(w_frontiers, 'descend');

            candidates = frontiers(idx(1:N_), :);
            center = candidates(1, :);
            
            if obj.id == 'r1'
                disp(center);
            end
            % generate random value to either use own cluster or merge with others
            obj.goal = center(:);
%             for r = 1:N_
%                 if rand <= p_branch
%                     if rand <= p_lvr
%                         % cluster center is the new candidate
%                         new = center;
%                         i = 1;
%                     else
%                         u = rand;
%                         p_candidates = w_candidates ./ sum(w_candidates);
%                         i = length(p_candidates);
%                         for k = 1:length(p_candidates)
%                             s_p = sum(p_candidates(1:k));
%                             if s_p >= u
%                                 i = k;
%                                 break
%                             end
%                         end
%                         new = candidates(i, :);
% 
%                     end
%                    
%                     else
%                 end
%             end
        end
        
        %% Calculate forces acting on the robot
        % using obstacle-dependent gaussian potential fields

        function calculate_forces(obj, env)
            theta = wrapTo180(obj.laser_beam.theta_laser(:).' + ...
                              obj.laser_beam.placement(3)) ;
            obj.angle_net = 0;
            obj.f_rep = zeros(size(theta));
            obj.f_att = zeros(size(theta));

            lines = env.lines;
            for n = 1:length(obj.neighbors)
                npos = obj.neighbors{n}.pos;
                width = obj.neighbors{n}.diameter;
                lines = [lines; 
                         npos(1)-width npos(2);
                         npos(1) npos(2)-width;
                         npos(1)+width npos(2);
                         npos(1) npos(2)+width;
                         npos(1)-width npos(2);
                         NaN NaN];
            end
            % measure distances
            obj.laser_beam.measure(lines, [obj.pos; obj.heading]);         

            % find measurements below threshold
            d = obj.laser_beam.measurements(:).';
            mask = d < obj.laser_thresh;   
            starts = strfind([false, mask], [0 1]);
            stops = strfind([mask, false], [1 0]);

            % repulsive forces
            if sum(mask) > 0
                theta_full = wrapTo360(theta);
                % average angles and distances to obstacles
                theta_k = wrapTo180((theta(starts) + theta(stops))./ 2); % center angles
                d_k = zeros(size(theta_k));   % average distances
                phi_k = zeros(size(theta_k)); % angle span of obstacles
                for i = 1:length(phi_k)
                    d_k(i) = mean(d(starts(i):stops(i)));
                    phi_k(i) = abs(theta_full(starts(i)) ... 
                                 - theta_full(stops(i)));
                end
                % corrected half angles with agent width added
                sigma_k = atan2d((d_k + 0.001) .* tand(phi_k./2) + obj.diameter, ...
                                  d_k + 0.001);
                for i = 1:length(theta)
                    obj.f_rep(i) = sum((obj.laser_range - d_k) .* exp(1/2) ...
                                   .* exp(-(theta_k - theta(i)).^2 ./ (2.*sigma_k.^2)));
                end
            end

            % convert goal from global to local frame
            R = [cosd(-obj.heading) -sind(-obj.heading); 
                 sind(-obj.heading) cosd(-obj.heading)];
            v_goal = R*(obj.goal - obj.pos);
            theta_goal = atan2d(v_goal(2), v_goal(1));
            % attractive forces
            for i = 1:length(theta)
                obj.f_att(i) = obj.gamma * pi / 180 * abs(wrapTo180(theta_goal - theta(i)));
            end
            obj.f_total = obj.f_rep + obj.f_att;

            % find minimizing angle
            [~, idx] = min(obj.f_total);
            obj.angle_net = theta(idx);
        end 

        %% update the grid (rao-blackwell occupancy grid implementation)
        
        function calculate_grid(obj)
            % Sensor measurements and angles.
            d = obj.laser_beam.measurements(:)';
            theta = obj.laser_beam.theta_laser(:)';
                          
            pose = [obj.pos; deg2rad(obj.heading + obj.laser_beam.placement(3))];
            angles = deg2rad(theta);
            scan = lidarScan(d, angles);
            
            insertRay(obj.map, pose, scan, obj.laser_range);

            rays = [];
            for i = 1:length(angles)
                [ends, mids] = raycast(obj.heatmap, pose, obj.detect_thresh, angles(i));
                rays = [rays; ends; mids];
            end

            % TODO if a sherd is found
            setOccupancy(obj.heatmap, rays, 0, 'grid');
        end
    end
end