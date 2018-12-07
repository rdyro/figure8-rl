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
using DelimitedFiles
using Statistics

include("update_info.jl")

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

function reward(x, u, nx, agent, world)
  return nx[2]^3 + (abs(x[3]) > 0.5 * world.road.width ? -1e9 : 0.0)
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
  (state_d, ctrl_d) = discretize_vit(world) # discretize the world

  # make the agents
  x0 = [0.0; 25.0; 0.0]
  agent1 = Agent(1, copy(x0), vis.make_car(context))
  push!(world.agents, agent1)
  agent2 = Agent(2, copy(x0), vis.make_car(context, [1.0, 0.0, 0.0]))
  push!(world.agents, agent2)
  agent3 = Agent(3, copy(x0), vis.make_car(context, [0.0, 1.0, 0.0]))
  push!(world.agents, agent3)

  #=
  IT = Int[]
  FLIP = Int[]
  t1 = time_ns()
  S = mdp.make_MDP(agent1, world, reward, state_d, ctrl_d)
  t2 = time_ns()
  println("Building takes $((t2 - t1) / 1e9) s")

  P = Policy(S, 0.999)
  i = 1
  flip = 1
  t1 = time_ns()
  while flip > 0
    (change, nbchange) = mdp.iterate!(P)
    flip = nbchange

    push!(IT, i)
    push!(FLIP, flip)

    println(flip)

    i += 1
  end
  t2 = time_ns()
  println("Iteration takes $((t2 - t1) / 1e9) s")

  writedlm("../data/vlit_it.txt", [IT FLIP])
  =#


  # Value Iteration Approach ------------------------------------------------ #
  # read in, iterate and store the policy
  P = nothing
  S = nothing

  policy_file_path = dir_path * "/../data/fr_vi_policy.bin"

  fp = open(policy_file_path, "r")
  (P, state_d, ctrl_d) = deserialize(fp)
  close(fp)

  S = P.S

  # switch agent's trained controller on
  agent1.controller! = mdp.controllerd_interp!
  agent1.custom = (P, state_d, ctrl_d)
  # ------------------------------------------------------------------------- #

  # Forward Search Approach ------------------------------------------------- #
  ctrl_d = discretize_fwds(world)
  agent2.controller! = olm.controller_fwds!
  # ------------------------------------------------------------------------- #

  # MCTS Approach ----------------------------------------------------------- #
  ctrl_d = discretize_fwds(world)
  agent3.controller! = olm.controller_mcts!
  # ------------------------------------------------------------------------- #


  # main loop definititons 
  window = true
  h = 1e-2
  t0 = time_ns()
  dt = 1 / 60
  N = round(Int, 20.0 / dt)
  D = 5

  # Accumulated Data -------------------------------------------------------- #
  global T = Array{Float64, 2}(undef, (N, D))

  global Cfwds = Array{Float64, 2}(undef, (N, D))
  global Cmcts = Array{Float64, 2}(undef, (N, D))

  global Vvlit = Array{Float64, 2}(undef, (N, D))
  global Vfwds = Array{Float64, 2}(undef, (N, D))
  global Vmcts = Array{Float64, 2}(undef, (N, D))
  # ------------------------------------------------------------------------- #
  for j in 1:D
    t = 0.0
    agent1.x = copy(x0)
    agent2.x = copy(x0)
    agent3.x = copy(x0)
    for i in 1:N
      # Value Iteration ----------------------------------------------------- #
      Vvlit[i, j] = agent1.x[2]
      # --------------------------------------------------------------------- #

      # Forward Search Approach --------------------------------------------- #
      t1 = time_ns()
      u = olm.plan_fwds(agent2.x, agent2, world, reward, ctrl_d, j)
      t2 = time_ns()
      Cfwds[i, j] = (t2 - t1) / 1e9
      Vfwds[i, j] = agent2.x[2]

      agent2.custom = u
      # --------------------------------------------------------------------- #

      # MCTS Approach ------------------------------------------------------- #
      t1 = time_ns()
      us = olm.plan_mcts(agent3.x, agent3, world, reward, ctrl_d, j)
      t2 = time_ns()
      Cmcts[i, j] = (t2 - t1) / 1e9
      Vmcts[i, j] = agent3.x[2]

      agent3.custom = us
      # --------------------------------------------------------------------- #

      ## advance one frame in time
      advance!(agent1.dynamics!, agent1.x, Pair(agent1, world), 0.0, dt, h)
      advance!(agent2.dynamics!, agent2.x, Pair(agent2, world), 0.0, dt, h)
      advance!(agent3.dynamics!, agent3.x, Pair(agent3, world), 0.0, dt, h)

      T[i, j] = t
      t += dt

      if mod(i, div(N, 10)) == 0
        println("$(j) -> $(t)")
      end
    end
    println("Finished depth = $(j)")
  end
  DATAvlit = fill(0.0, (N, 2 * D))
  DATAfwds = fill(0.0, (N, 3 * D))
  DATAmcts = fill(0.0, (N, 3 * D))
  for i in 1:D
    DATAvlit[:, 2 * (i - 1) + 1] = T[:, i]
    DATAvlit[:, 2 * (i - 1) + 2] = Vvlit[:, i]

    DATAfwds[:, 3 * (i - 1) + 1] = T[:, i]
    DATAfwds[:, 3 * (i - 1) + 2] = Vfwds[:, i]
    DATAfwds[:, 3 * (i - 1) + 3] = Cfwds[:, i]

    DATAmcts[:, 3 * (i - 1) + 1] = T[:, i]
    DATAmcts[:, 3 * (i - 1) + 2] = Vmcts[:, i]
    DATAmcts[:, 3 * (i - 1) + 3] = Cmcts[:, i]
  end

  AVGCfwds = map(i -> mean(DATAfwds[:, 3 * (i - 1) + 3]), 1:D)
  AVGCmcts = map(i -> mean(DATAmcts[:, 3 * (i - 1) + 3]), 1:D)

  AVGVvlit = map(i -> mean(DATAvlit[:, 2 * (i - 1) + 2]), 1:D)
  AVGVfwds = map(i -> mean(DATAfwds[:, 3 * (i - 1) + 2]), 1:D)
  AVGVmcts = map(i -> mean(DATAmcts[:, 3 * (i - 1) + 2]), 1:D)

  writedlm("../data/vlit_data.txt", DATAvlit)
  writedlm("../data/fwds_data.txt", DATAfwds)
  writedlm("../data/mcts_data.txt", DATAmcts)

  writedlm("../data/fwds_avgc.txt", AVGCfwds)
  writedlm("../data/mcts_avgc.txt", AVGCmcts)

  writedlm("../data/vlit_avgv.txt", AVGVvlit)
  writedlm("../data/fwds_avgv.txt", AVGVfwds)
  writedlm("../data/mcts_avgv.txt", AVGVmcts)
end
