%% Create ground and save for later test runs

function create_setup(file_name)
    L = [50 50]; % x y (m) search area limits
    
    % search area boundaries
    bounds = {[ 0    0  ; L(1)  0  ], ... 
              [L(1)  0  ; L(1) L(2)], ...
              [L(1) L(2);  0   L(2)], ...
              [ 0   L(2);  0    0  ]};
    
    R = 3;
    x0 = 25;
    x1 = 35;
    t = linspace(0, 2*pi, 20);
    circ = [x0 + R*cos(t); x1 + R*sin(t)]';
    % obstacle
    %obstacles = {[8 11; 12 11; 13 13; 7 13; 7 8; 8 8; 8 11]};
    obstacles = {[0 45; 0 46; 10 46; 10 45; 0 45], ...
                 circ, ...
                 [5 22; 8 22; 8 26; 5 26; 5 22], ...
                 [10 5; 15 3; 13 10; 10 5], ...
                 [35 45; 37 47; 47 37; 45 35; 35 45]};
    
    % poi
    poi = [[25 25] + 6 .* randn(15, 2);
           [15 10] + 2 .* randn(10, 2);
           [30 10] + 3 .* randn(25, 2)];

    %% simulation
    ground = Environment(L, obstacles, bounds, poi);

    save(file_name, 'ground');
end

