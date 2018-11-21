dir_path = @__DIR__
push!(LOAD_PATH, dir_path * "/sim")
push!(LOAD_PATH, dir_path * "/vis")
push!(LOAD_PATH, dir_path * "/mdp")
using sim
using vis
using mdp

using Serialization

include("racerd.jl")

# Scenario: Racer scenario
function main()
  # load the graphical context (OpenGL handle, the graphical window, etc.)
  context = vis.setup()

  vis_scaling = 0.01

  # make the road
  path = make_figure8_path()
  road_width = 20.0
  road = Road(path, road_width, 
                     vis.make_road(context, path.X, path.Y, road_width))
  road.road.T = vis.scale_mat(vis_scaling, vis_scaling)

  display(maximum(road.path.S))

  # make the agents
  agent = Agent(1, [0.0; 0.0; 0], vis.make_car(context))
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

    println("Input the number of iterations")
    repeat = readline()
    repeat = repeat == "" ? 500 : parse(Int, repeat)

    for i in 1:repeat
      print("$(i) -> ")
      display(mdp.iterate!(P))
    end
    fp = open(policy_file_path, "w")
    serialize(fp, P)
    close(fp)
  else
    S = make_MDP(world)
    P = Policy(S, 0.999)

    println("Input the number of iterations")
    repeat = readline()
    repeat = repeat == "" ? 500 : parse(Int, repeat)
    for i in 1:repeat
      print("$(i) -> ")
      display(mdp.iterate!(P))
    end
    fp = open(policy_file_path, "w")
    serialize(fp, P)
    close(fp)
  end

  agent.controller! = controllerd_interp!
  agent.dynamics! = dynamicsd!

  v1 = vis.make_vector_xy(context, 0.0, 0.0, 0.0, 0.0)

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
      XD = bounding_xd(agent.x, world.road)
      X = [xd2x(xd, world.road) for xd in XD]
      LU = [P.S[ls].a[P.Aidx[ls]] for ls in xd2ls.(XD)]
      U = [ud2u(lu2ud(lu)) for lu in LU]
      U1 = [u[1] for u in U]
      U2 = [u[2] for u in U]

      u1 = interp(X, agent.x, U1)
      u2 = interp(X, agent.x, U2)
      #display(U)
      #display([u1, u2])
      agent.custom = [u1, u2]

      ls = xd2ls(x2xd(agent.x, world.road))
      #println(P.S[ls].a2r[P.Aidx[ls]])
      ##agent.custom = P.S[ls].a[P.Aidx[ls]]

      advance!(agent.dynamics!, agent.x, Pair(agent, world), oldt, t, h)
      #println(agent.x)
      #println(P.S[ls].a[P.Aidx[ls]])
      (x, y) = sim.sp2xy(agent.x[1], agent.x[3], world.road.path)

      th = update_renderer(agent, world) + pi / 2
      vis.update_vector_thr!(v1, vis_scaling * x, vis_scaling * y, th, 
                             0.2 * agent.x[2] / 50.0)

      push!(to_visualize, v1)

      if agent.car != nothing
        push!(to_visualize, agent.car)
      end
    end

    window = vis.visualize(context, to_visualize)

    oldt = t
  end
end

#main()
