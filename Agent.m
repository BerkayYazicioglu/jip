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

        goal_reached % flag for goal selection
        comm_cnt = 1
        pos_log;
    end
    %% Constants
    properties(Constant)
        max_v = 2                  % m/s
        max_w = 90                 % degrees/s
        
        comm_range = inf           % m
        neighbor_range = inf       % m
        
        diameter = 0.25            % m
        
        laser_pos = [0 0 0];       % body frame, (y theta)
        laser_angle = [-90 90];    % angle range of the laser beam (degrees)
        laser_K = 101;             % number of beams on the laser
        laser_range = 5;           % range of the laser beam (m)
        laser_thresh = 2;          % threshold of calculating force (m)

        detect_range = 2;          % sherd detection radius (m)
        repulse_range = 5;         % inter agent repulsion range
        
        gamma = 1;                 % attractive field gain
        goal_error = 2;            % margin of error for reaching a goal (m)
        
        fov = 4;                   % occupancy grid field of vision
        occ_thresh = 0.65;
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
            obj.goal_reached = true;
            obj.laser_beam = Laser(obj.laser_pos, obj.laser_angle, ...
                                   obj.laser_K, obj.laser_range);
            obj.map = occupancyMap(grid_dims(1), grid_dims(2), obj.fov);
            obj.map.OccupiedThreshold = obj.occ_thresh;

            obj.heatmap = occupancyMap(grid_dims(1), grid_dims(2), obj.fov);
        end
        
        %% Update neighbor positions
        function update_neighbors(obj, agent_list)
            t_int = floor(1/obj.dt);

            if obj.comm_cnt == t_int
                obj.neighbors = {};
                for i = 1:length(agent_list)
                    agent = agent_list{i};
                    if strcmp(agent.id, obj.id) == false
                        if sum((agent.pos - obj.pos).^2)^0.5 <= obj.neighbor_range
                            obj.neighbors{end+1} = agent;
                            updateOccupancy(obj.map, agent.map.occupancyMatrix);
                            updateOccupancy(obj.heatmap, agent.heatmap.occupancyMatrix);
                        end
                    end
                end
                obj.comm_cnt = 1;
            else
                obj.comm_cnt = obj.comm_cnt + 1;
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
                obj.goal_reached = true;
            else
                obj.goal_reached = true;
            end
            
            obj.pos_log = [obj.pos_log obj.pos];
            obj.t = obj.t + obj.dt;
        end
        
        %% Set goal
        % BSO algorithm 

        function set_goal(obj)
            % detect the frontiers
            explored = checkOccupancy(obj.map) == 0;
            
            tmp_map = obj.map.copy();
            inflate(tmp_map, obj.fov, 'grid');
            
            occupied = find(checkOccupancy(tmp_map) == 1);
            [ox, oy] = ind2sub(obj.map.GridSize, occupied);
            occupied = [oy(:)  obj.map.GridSize(2) - ox(:)];
            [frontiers, ~, ~, ~] = bwboundaries(explored, 4, 'holes');

            % preprocessing, bunch of edge cases because of bwboundaries
            % ¯\_(ツ)_/¯
            for k = 1:length(frontiers)
                frontiers{k}(:, 1) = obj.map.GridSize(2) - frontiers{k}(:, 1);
                frontiers{k} = fliplr(frontiers{k});
                frontiers{k} = frontiers{k}(all(frontiers{k}(:, 1)-1, 2),:);
                frontiers{k} = frontiers{k}(all(frontiers{k}(:, 2), 2),:);
                frontiers{k} = frontiers{k}(all(frontiers{k}(:, 1) - obj.map.GridSize(1), 2),:);
                frontiers{k} = frontiers{k}(all(frontiers{k}(:, 2) - obj.map.GridSize(2)+1, 2),:);
                frontiers{k} = setdiff(frontiers{k}, occupied, 'rows');
                frontiers{k} = frontiers{k} ./ obj.fov;
            end
            obj.f = frontiers;
            
            t_int = floor(1/obj.dt)/2;
            if obj.comm_cnt < t_int
                return
            end
            
            frontiers = cell2mat(frontiers); % world coordinates

            if obj.goal_reached == false
                return
            end

            lambda_ = 0.9; % importance of cost over utility
            N0_ = 750;
            N_ = 10;
            
            p = repmat(obj.pos', [size(frontiers, 1) 1]);
            d2 = sqrt(sum((frontiers - p).^2, 2));
            
            % cost of a frontier is the normalized distance
            c_frontiers = d2 ./ max(d2);

            % utility of a frontier is num of detected poi around + unexplored
            % cells over a circle around detection radius 
            u_frontiers = zeros(size(c_frontiers));
            
            poi = [];
            unexplored = [];
            [sy, sx] = ind2sub(obj.heatmap.GridSize, find(checkOccupancy(obj.heatmap) == 1));
            [uy, ux] = ind2sub(obj.heatmap.GridSize, find(checkOccupancy(obj.heatmap) == -1));
            if sx
                poi = grid2world(obj.heatmap, [sy sx]);
            end
            if ux
                unexplored = grid2world(obj.heatmap, [uy ux]);
            end
                
            % find the utility of a given point (world coordinates)
            function u = find_utility(p)
                p = p(:)';
                num_poi = 0;
                num_unexplored = 0;
                if poi
                    d_poi = sqrt(sum((repmat(p, [size(poi, 1) 1]) - poi).^2, 2));
                    num_poi = sum(d_poi <= obj.laser_range);
                end
                if unexplored
                    d_unexp = sqrt(sum((repmat(p, [size(unexplored, 1) 1]) - unexplored).^2, 2));
                    num_unexplored = sum(d_unexp <= obj.detect_range);
                end
                u = num_unexplored / size(unexplored, 1) ...
                  + num_poi / (size(poi, 1) + 0.0001);
                u = u/2;
            end
            
            % find utilities
            for k = 1:length(u_frontiers)
                 u_frontiers(k) = find_utility(frontiers(k,:));
            end

            w_frontiers = u_frontiers - lambda_ .* c_frontiers; 

            [w_candidates, idx] = sort(w_frontiers, 'descend');
            
            if N_ > length(frontiers)
                N_ = length(frontiers);
            end
            candidates = frontiers(idx(1:N_), :);
            w_candidates = w_candidates(1:N_, :);
            
            if obj.id == 'r3'
                disp([u_frontiers(idx(1:5))'; lambda_*c_frontiers(idx(1:5))']);
            end

            %center = candidates(1, :);
            %obj.goal = center(:); % greedy algorithm, uncooperative
            
            % LVS for stochasticity
            u = rand;
            w_candidates = w_candidates(1: ceil(N_/2));
            norm_w = (w_candidates - min(w_candidates)) ...
                   / (max(w_candidates) - min(w_candidates));
            p_candidates = norm_w ./ sum(norm_w);
            center_idx = N_;
            sum_p = 0;

            for i = 1:length(w_candidates)
                sum_p = sum_p + p_candidates(i);
                if sum_p >= u
                    center_idx = i;
                    break
                end
            end

            center = candidates(center_idx, :);
            obj.goal = center(:);

            % repulsion between robots
            neighbor_d = zeros(1, length(obj.neighbors));
            for i = 1:length(obj.neighbors)
                neighbor_d(i) = sqrt(sum((obj.neighbors{i}.goal - obj.goal).^2));
            end
            [n_d, idx] = min(neighbor_d);
            
            if idx
                n_pos = obj.neighbors{idx}.goal';
                R = obj.repulse_range^2 / (n_d + obj.repulse_range);
                d_n = sqrt(sum((repmat(n_pos, [size(candidates, 1) 1]) - candidates).^2, 2));
                inR = find(d_n <= R);
                if inR
                    d_n = d_n(inR);
                    candidates = candidates(inR, :);
                    [~, idx] = max(d_n);
                    obj.goal = candidates(idx, :)';
%                 else
%                     [~, idx] = min(d_n);
%                     obj.goal = candidates(idx, :)';
                end
            end
        end


        %% Calculate forces acting on the robot
        % using obstacle-dependent gaussian potential fields

        function calculate_forces(obj)
            theta = wrapTo180(obj.laser_beam.theta_laser(:).' + ...
                              obj.laser_beam.placement(3)) ;
            obj.angle_net = 0;
            obj.f_rep = zeros(size(theta));
            obj.f_att = zeros(size(theta));

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
        
        function calculate_grid(obj, env)
            % Sensor measurements and angles.
            d = obj.laser_beam.measurements(:)';
            theta = obj.laser_beam.theta_laser(:)';
                          
            pose = [obj.pos; deg2rad(obj.heading + obj.laser_beam.placement(3))];
            angles = deg2rad(theta);
            scan = lidarScan(d, angles);
            
            insertRay(obj.map, pose, scan, obj.laser_range);
            rays = [];
            for i = 1:length(angles)
                d_i = d(i);
                if d_i > obj.detect_range
                    d_i = obj.detect_range;
                end
                [ends, mids] = raycast(obj.heatmap, pose, d_i, angles(i));
                rays = [rays; ends; mids];
            end

            % detect poi
            poi = world2grid(obj.heatmap, env.poi);
            poi = rays(ismember(rays, poi, 'rows'), :);

            setOccupancy(obj.heatmap, rays, 0, 'grid');
            if poi
                setOccupancy(obj.heatmap, poi, 1 , 'grid');
            end
        end

        %% measure with lidar and calculate grid
        function measure_and_map(obj, env)
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
            obj.calculate_grid(env);
        end
    end
end