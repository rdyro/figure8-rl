dir_path = @__DIR__
push!(LOAD_PATH, dir_path * "/sim") # simulation
push!(LOAD_PATH, dir_path * "/vis") # visualization
push!(LOAD_PATH, dir_path * "/mdp") # value iteration MDP
push!(LOAD_PATH, dir_path * "/dis") # discretization
using sim
using vis
using mdp
using dis

using Serialization
using Printf

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

  # make the world
  global world = World(road, [], vis_scaling)

  # make the agents
  agent = Agent(1, [0.0; 0.0; 0], vis.make_car(context))
  vis.car_lights!(agent.car, false)
  agent.controller! = controllerd_interp!
  agent.dynamics! = dynamicsd!
  push!(world.agents, agent)

  # read in, iterate and store the policy
  global P = nothing
  S = nothing
  global (state_d, ctrl_d) = discretize(world)

  policy_file_path = dir_path * "/../data/fr_vi_policy.bin"

  println("Input the number of iterations")
  repeat = readline()
  repeat = repeat == "" ? 500 : parse(Int, repeat)
  if isfile(policy_file_path)
    fp = open(policy_file_path, "r")
    (P, state_d, ctrl_d) = deserialize(fp)
    close(fp)

    S = P.S
  else
    S = make_MDP(world)
    P = Policy(S, 0.999)
  end
  for i in 1:repeat
    print("$(i) -> ")
    display(mdp.iterate!(P))
  end
  fp = open(policy_file_path, "w")
  serialize(fp, (P, state_d, ctrl_d))
  close(fp)

  # switch agent's trained controller on
  agent.controller! = controllerd_interp!
  agent.custom = (P, state_d, ctrl_d)

  # make diagnostics render objects
  vec1 = vis.make_vector_xy(context, 0.0, 0.0, 0.0, 0.0)
  vec2 = vis.make_vector_xy(context, 0.0, 0.0, 0.0, 0.0)
  txt_pos1 = vis.make_text(context, "", 0.0, 0.0, 0.5)
  txt_vel1 = vis.make_text(context, "", 0.0, 0.0, 0.5)
  txt_p1 = vis.make_text(context, "", 0.0, 0.0, 0.5)
  txt_r1 = vis.make_text(context, "", 0.0, 0.0, 0.5)
  txt_inp1 = vis.make_text(context, "", 0.0, 0.0, 0.5)

  # main loop for rendering and simulation
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
      ## advance one frame in time
      advance!(agent.dynamics!, agent.x, Pair(agent, world), oldt, t, h)

      ## visualize
      (x, y, sx, sy, dx, u) = diagnostic(agent, world, t)
      agent.is_braking = dx[1] * u[1] < 0

      ## get additional information
      dp = dx[3]
      th = atan(sy, sx)
      th_car = atan(dp, agent.x[2]) + atan(sy, sx)

      if agent.car != nothing
        ## update the car
        vis.car_lights!(agent.car, agent.is_braking)
        agent.car.T = (vis.scale_mat(world.vis_scaling, world.vis_scaling) * 
                       vis.translate_mat(x, y) * 
                       vis.rotate_mat(th_car - pi / 2))
        push!(to_visualize, agent.car)

        ## update the velocity vector
        vis.update_vector_thr!(vec1, world.vis_scaling * x, 
                               world.vis_scaling * y, th, 
                               0.2 * agent.x[2] / 50.0)
        push!(to_visualize, vec1)

        ## update the text
        #vis.update_text!(txt1, string(dx[1]), world.vis_scaling * x + 0.2, 
        #                 world.vis_scaling * y + 0.2, 0.5)
        vis.update_text!(txt_pos1, @sprintf("pos = %3.1e", agent.x[1]), 
                         0.7, 0.7, 1.0)
        push!(to_visualize, txt_pos1)
        vis.update_text!(txt_vel1, @sprintf("vel = %3.1e", agent.x[2]), 
                         0.7, 0.6, 1.0)
        push!(to_visualize, txt_vel1)
        vis.update_text!(txt_p1, @sprintf("p = %3.1e", agent.x[3]), 
                         0.7, 0.5, 1.0)
        push!(to_visualize, txt_p1)
        vis.update_text!(txt_inp1, @sprintf("u =  (%+3.1e, %+3.1e)", u[1], 
                                            u[2]), 0.7, 0.4, 1.0)
        push!(to_visualize, txt_inp1)


        ls = dis.x2ls(state_d, agent.x)
        r = P.S[ls].a2r[P.Aidx[ls]]
        vis.update_text!(txt_r1, @sprintf("r = %3.1e", r),
                         0.7, 0.3, 1.0)
        push!(to_visualize, txt_r1)

        ## update the bounds vector
        (xmin, ymin, xmax, ymax) = vis.bounding_box(txt_inp1)
        vis.update_vector_xy!(vec2, xmin, ymin, xmax, ymax)
        push!(to_visualize, vec2)
      end
    end

    window = vis.visualize(context, to_visualize)

    oldt = t
  end
end

#main()
