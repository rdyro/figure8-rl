dir_path = @__DIR__
push!(LOAD_PATH, dir_path * "/sim") # simulation
push!(LOAD_PATH, dir_path * "/vis") # visualization
push!(LOAD_PATH, dir_path * "/mdp") # fully observable MDP
push!(LOAD_PATH, dir_path * "/dis") # discretization
push!(LOAD_PATH, dir_path * "/olm") # online methods
using sim
using vis
using mdp
using dis
using olm

using Serialization
using Printf

# state space
const pts_per_s = 150
const pts_per_ds = 30
const pts_per_p = 20
const min_s = 0.0
const min_p = -12.0
const max_p = 12.0
const min_ds = 0.0
const max_ds = 50.0

# action space
const pts_per_acc = 10
const pts_per_ste = 3
const min_acc = -15
const max_acc = 5
const min_ste = -1e-1
const max_ste = 1e-1

function discretize_vit(world::World)
  max_s = world.road.path.S[end]
  state_d = Discretization([pts_per_s, pts_per_ds, pts_per_p],
                           [min_s, min_ds, min_p],
                           [max_s, max_ds, max_p])
  ctrl_d = Discretization([pts_per_acc, pts_per_ste],
                          [min_acc, min_ste],
                          [max_acc, max_ste])
  return (state_d, ctrl_d)
end

function discretize_fwds(world::World)
  pts_per_acc = 3
  pts_per_ste = 3
  min_acc = -15
  max_acc = 5
  min_ste = -1e-1
  max_ste = 1e-1
  ctrl_d = Discretization([pts_per_acc, pts_per_ste],
                          [min_acc, min_ste],
                          [max_acc, max_ste])
  return ctrl_d
end

function reward(world, agent, x, u, nx)
  return abs(nx[3]) > 0.35 * world.road.width ? -1e9 : nx[2]^3
end

function update_info(info::vis.InfoBox, agent::Agent, world::World, t::Float64)
  (x, y, sx, sy, dx, u) = diagnostic(agent, world, t)

  dp = dx[3]
  th = atan(sy, sx)
  th_car = atan(dp, agent.x[2]) + atan(sy, sx)

  vis.update_vector_thr!(info.ds_vec, info.scaling * x, 
                         info.scaling * y, th, 
                         0.2 * agent.x[2] / 50.0)
  vis.update_vector_thr!(info.u1_vec, info.scaling * x, 
                         info.scaling * y, th, 
                         0.2 * u[1] / 10.0)
  vis.update_vector_thr!(info.u2_vec, info.scaling * x, 
                         info.scaling * y, th + pi / 2, 
                         0.2 * u[2] / 0.1)

  offset = 0.0
  vis.update_text!(info.id_text, @sprintf("Agent %d", agent.id), 
                   info.x, info.y - offset, 1.0)
  (xmin, ymin, xmax, ymax) = vis.bounding_box(info.id_text)
  offset += ymax - ymin
  vis.update_text!(info.s_text, @sprintf("s= %3.1e", agent.x[1]), 
                   info.x, info.y - offset, 1.0)
  (xmin, ymin, xmax, ymax) = vis.bounding_box(info.id_text)
  offset += ymax - ymin
  vis.update_text!(info.ds_text, @sprintf("ds= %3.1e", agent.x[2]), 
                   info.x, info.y - offset, 1.0)
  (xmin, ymin, xmax, ymax) = vis.bounding_box(info.id_text)
  offset += ymax - ymin
  vis.update_text!(info.p_text, @sprintf("p= %3.1e", agent.x[3]), 
                   info.x, info.y - offset, 1.0)
  (xmin, ymin, xmax, ymax) = vis.bounding_box(info.id_text)
  offset += ymax - ymin
  vis.update_text!(info.u1_text, @sprintf("u1= %+3.1e", u[1]),
                   info.x, info.y - offset, 1.0)
  (xmin, ymin, xmax, ymax) = vis.bounding_box(info.id_text)
  offset += ymax - ymin
  vis.update_text!(info.u2_text, @sprintf("u2= %+3.1e", u[2]),
                   info.x, info.y - offset, 1.0)
end

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
  world = World(road, [], vis_scaling)

  # make the agents
  agent = Agent(1, [0.0; 0.0; 0], vis.make_car(context))
  push!(world.agents, agent)

  # Value Iteration Approach ------------------------------------------------ #
  # read in, iterate and store the policy
  #=
  P = nothing
  S = nothing
  (state_d, ctrl_d) = discretize_vit(world) # value iteration

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
    S = mdp.make_MDP(agent, world, reward, state_d, ctrl_d)
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
  agent.controller! = mdp.controllerd_interp!
  agent.custom = (P, state_d, ctrl_d)
  =#
  # ------------------------------------------------------------------------- #

  # Forward Search Approach ------------------------------------------------- #
  ctrl_d = discretize_fwds(world)
  agent.controller! = olm.controller_fwds!
  # ------------------------------------------------------------------------- #

  # make diagnostics render objects
  info = vis.InfoBox(context, 0.75, 0.75, vis_scaling)

  # main loop for rendering and simulation
  window = true
  h = 1e-2
  t0 = time_ns()
  oldt = (time_ns() - t0) / 1e9
  while window
    t = (time_ns() - t0) / 1e9

    to_visualize = []
    if world.road.road != nothing
      push!(to_visualize, world.road.road)
    end

    for agent in world.agents
      # Forward Search Approach --------------------------------------------- #
      @time plan = olm.replan(agent.x, agent, world, reward, ctrl_d, 4)
      agent.custom = plan.value.u
      # --------------------------------------------------------------------- #

      ## advance one frame in time
      advance!(agent.dynamics!, agent.x, Pair(agent, world), oldt, t, h)
      update_info(info, agent, world, t)

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
      end
    end
    push!(to_visualize, info)

    window = vis.visualize(context, to_visualize)

    oldt = t
  end
end
