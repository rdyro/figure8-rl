push!(LOAD_PATH, pwd() * "/sim")
using sim
# Scenario: toy case for testing a FAST RACER

function main()
  # load the graphical context (OpenGL handle, the graphical window, etc.)
  context = vis.setup()

  vis_scaling = 0.01

  # make the road
  path = make_figure8_path()
  road_width = 10.0
  road = Road(path, road_width, 
              vis.make_road(context, path.X, path.Y, road_width))
  road.road.T = vis.scale_mat(vis_scaling, vis_scaling)

  # make the agents
  agent1 = Agent(1, [0.0; 120 / 3.6; 0], vis.make_car(context))
  vis.car_lights!(agent1.car, false)

  agent2 = Agent(1, [0.0; 120 / 3.6; 3], vis.make_car(context, [1.0, 1.0, 0.0]))
  vis.car_lights!(agent2.car, false)

  agent3 = Agent(1, [0.0; 60 / 3.6; -3], vis.make_car(context, [1.0, 0.0, 1.0]))
  vis.car_lights!(agent3.car, false)

  # make the world
  world = World(road, [agent1, agent2, agent3], vis_scaling)

  window = true
  h = 1e-2
  t0 = time_ns()
  oldt = (time_ns() - t0) / 1e9
  while window
    t = (time_ns() - t0) / 1e9

    to_visualize = vis.RenderObject[]
    if world.road.road != nothing
      push!(to_visualize, world.road.road)
    end

    for agent in world.agents
      rk4!(agent.dynamics!, agent.x, Pair(agent, world), oldt, t, h)
      update_renderer(agent, world)
      if agent.car != nothing
        push!(to_visualize, agent.car)
      end
    end

    window = vis.visualize(context, to_visualize)

    oldt = t
  end
end

main()
