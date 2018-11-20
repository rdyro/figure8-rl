dir_path = @__DIR__
push!(LOAD_PATH, dir_path * "/sim")
push!(LOAD_PATH, dir_path * "/vis")
push!(LOAD_PATH, dir_path * "/mdp")
using sim
using vis
using mdp

using Serialization

include("fast_racer_discretization.jl")

# Scenario: Basic scenario template
function main()
  # load the graphical context (OpenGL handle, the graphical window, etc.)
  context = vis.setup()

  vis_scaling = 0.01

  # make the road
  path = make_figure8_path()
  road_width = 20.0
  global road = Road(path, road_width, 
                     vis.make_road(context, path.X, path.Y, road_width))
  road.road.T = vis.scale_mat(vis_scaling, vis_scaling)

  display(maximum(road.path.S))

  # make the agents
  global agent = Agent(1, [0.0; 0.0; 0], vis.make_car(context))
  vis.car_lights!(agent.car, false)


  # make the world
  global world = World(road, [agent], vis_scaling)

  global P = nothing
  global S = nothing

  policy_file_path = dir_path * "/../data/fr_vi_policy.bin"

  if isfile(policy_file_path)
    fp = open(policy_file_path, "r")
    P = deserialize(fp)
    close(fp)

    S = P.S

    for i in 1:100
      print("$(i) -> ")
      display(mdp.iterate!(P))
    end
    fp = open(policy_file_path, "w")
    serialize(fp, P)
    close(fp)
  else
    S = make_MDP(world)
    P = Policy(S, 0.999)
    for i in 1:500
      print("$(i) -> ")
      display(mdp.iterate!(P))
    end
    fp = open(policy_file_path, "w")
    serialize(fp, P)
    close(fp)
  end

  agent.controller! = controllerd!
  agent.dynamics! = dynamicsd!

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
      ls = xd2ls(x2xd(agent.x, world.road))
      agent.custom = P.S[ls].a[P.Aidx[ls]]

      advance!(agent.dynamics!, agent.x, Pair(agent, world), oldt, t, h)
      println(x2xd(agent.x, world.road))

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
