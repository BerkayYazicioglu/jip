%% Struct for variables defining a potential field 

classdef Potential < handle
    properties
        points % mx2 matrix of points along the source curve
        interp % interpolation type 'linear' 'spline' 
        eta    % potential gain factor
        rho    % potential radius
        type   % source type 'boundary' 'obstacle' 'agent'
        id     % integer id
    end
    methods
        %% Constructor
        function obj = Potential(id, points, interp, eta, rho, type)
            obj.id = id;
            obj.points = points;
            obj.interp = interp;
            obj.eta = eta;
            obj.rho = rho;
            obj.type = type;
        end

        %% Calculate potential field and add to existing one
        % X,Y -> meshgrid matrices

        function V = add_to_field(obj, V_0, X, Y)
            % find the distance of every mesh point to the obstacle
            [~, dist, ~] = distance2curve(obj.points, [X(:) Y(:)], obj.interp);
            
            % get the idx of distances < rho
            idx = find(dist < obj.rho);
            % add selected to the potential map
            V_0(idx) = V_0(idx) + 0.5 .* obj.eta .* (1./dist(idx) - 1./obj.rho).^2;
            % correction for potential spikes 
            idx = find(V_0 > 0.5 * obj.eta);
            V_0(idx) = repmat(0.5 * obj.eta, length(idx), 1);
            V = V_0;
        end
    end
end