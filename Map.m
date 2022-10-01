%% Map data of Environment exploration for Agents
% x,y is defined on the lower left corner on the boundary

classdef Map < handle
    %% Variables
    properties
        potentials % an array of Potential objects 
    end
    
    %% Methods
    methods
        %% Constructor
        function obj = Map(initial_potentials)
            obj.potentials = initial_potentials;
        end
        
        %% Add new potential to the environment
        function add_potential(obj, id, points, interp, eta, rho, type)
            P = Potential(id, points, interp, eta, rho, type);
            obj.potentials{end+1} = P;
        end

        %% Remove a potential
    end
end