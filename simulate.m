%% simulate the environment

function simulate(ax, ground, steps)
    for i = 1:steps
        for k = 1:length(ground.agents)
            ground.agents{k}.update_neighbors(ground.agents);
            ground.agents{k}.measure_and_map(ground);
            ground.agents{k}.set_goal();
            ground.agents{k}.calculate_forces();
        end
        if i == 1
            ground.animate(ax{1});
            ground.plot_occupancy_grid(1, ax{2});
            ground.plot_laser_measurements(1, ax{3});
            ground.plot_laser_potentials(1, ax{4});
        else
            ground.animate();
            ground.plot_occupancy_grid(1, ax{2});
            ground.plot_laser_measurements(1);
            ground.plot_laser_potentials(1);
            drawnow;
        end
        for k = 1:length(ground.agents)
            ground.agents{k}.control();
        end
    end
end