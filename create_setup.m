%% Create ground and save for later test runs

function create_setup(file_name)
    etas = [10 10]; % obstacle, agent
    rhos = [5 2];   % (m), obstacle, agent
    
    L = [20 20]; % x y (m) search area limits
    
    % search area boundaries
    bounds = {[ 0    0  ; L(1)  0  ], ... 
              [L(1)  0  ; L(1) L(2)], ...
              [L(1) L(2);  0   L(2)], ...
              [ 0   L(2);  0    0  ]};
    
    % obstacle
    obstacles = {[7 8; 8 8; 8 11; 12 11; 12 8; 13 8; 13 13; 7 13; 7 8]};
    %% simulation
    map = convert(bounds);
    map = [map; convert(obstacles)];
    ground = Environment(L, map, etas(2), rhos(2));
    
    % add boundaries
    for i = 1:length(bounds)
        ground.add_potential(i, bounds{i}, etas(1), rhos(1), 'boundary');
    end
    % add obstacles
    for i = 1:length(obstacles)
        ground.add_potential(i, obstacles{i}, etas(1), rhos(1), 'obstacle');
    end

    save(file_name, 'ground');

    % convert [x y]*m cell array to NaN seperated matrices
    function map = convert(array)
        map = [];
        for j = 1:length(array)
            map = [map; array{j}; NaN NaN];
        end
    end
end

