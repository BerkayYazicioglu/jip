%% Create ground and save for later test runs

function create_setup(file_name)
    L = [50 50]; % x y (m) search area limits
    
    % search area boundaries
    bounds = {[ 0    0  ; L(1)  0  ], ... 
              [L(1)  0  ; L(1) L(2)], ...
              [L(1) L(2);  0   L(2)], ...
              [ 0   L(2);  0    0  ]};
    
    % obstacle
    obstacles = {[8 11; 12 11; 13 13; 7 13; 7 8; 8 8; 8 11]};
    
    %% simulation
    ground = Environment(L, obstacles, bounds);

    save(file_name, 'ground');
end

