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
        orientation  % attitude (euler angles?)
        neighbors    % neighboring agents

        sample_freq  % sensor sampling Hz
        dt           % control period
        t            % time (s)

        f_rep        % repulsive field wrt agent frame
        f_att        % attractive field wrt agent frame
        f_total      % f_rep + f_att
        angle_net    % angle correction calculated from the net virtual forces 

        laser_beam   % 2d laser beam sensor 
    end
    %% Constants
    properties(Constant)
        max_v = 3                  % m/s
        comm_range = inf           % m
        neighbor_range = inf       % m
        diameter = 0.25            % m
        laser_pos = [0 0 0];       % body frame, (x y theta)
        laser_angle = [-90 90];    % angle range of the laser beam (degrees)
        laser_K = 51;              % number of beams on the laser
        laser_range = 5;           % range of the laser beam (m)
        laser_thresh = 2;          % threshold of calculating force (m)
        gamma = 2;                 % attractive field gain
        goal_error = 1;            % margin of error for reaching a goal (m)
    end

    %% Methods
    methods
        %% Constructor
        function obj = Agent(id, sample_freq, dt)
            obj.id = id;
            obj.sample_freq = sample_freq;
            obj.dt = dt;
            obj.t = 0;
            obj.w = 0;
            obj.v = 0;
            obj.neighbors = {};
            obj.laser_beam = Laser(obj.laser_pos, obj.laser_angle, ...
                                   obj.laser_K, obj.laser_range);
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
        
        %% Move agent (WIP, very simple angle change for now)
        function control(obj)
            d = obj.max_v * obj.dt;
            % check if at the goal
            e = sum((obj.goal - obj.pos).^2);
            
            if e > obj.goal_error
                % assume instant heading change
                obj.orientation(1) = wrapTo180(obj.orientation(1) + obj.angle_net);
                vec = [d * cosd(obj.orientation(1));
                       d * sind(obj.orientation(1))];
    
                obj.pos = obj.pos + vec;
            end
            
            obj.t = obj.t + obj.dt;
        end
        
        %% Set goal
        function set_goal(obj, goal_pos)
            obj.goal = goal_pos(:);
        end

        %% Calculate forces acting on the robot
        % using obstacle-dependent gaussian potential fields
        % maybe add virtual fields later for decision making??

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
                         npos(1)-width npos(2) npos(1) npos(2)-width;
                         npos(1) npos(2)-width npos(1)+width npos(2);
                         npos(1)+width npos(2) npos(1) npos(2)+width;
                         npos(1) npos(2)+width npos(1)-width npos(2)];
            end
            % measure distances
            obj.laser_beam.measure(lines, [obj.pos; obj.orientation(1)]);

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
            R = [cosd(-obj.orientation(1)) -sind(-obj.orientation(1)); 
                 sind(-obj.orientation(1)) cosd(-obj.orientation(1))];
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
    end
end