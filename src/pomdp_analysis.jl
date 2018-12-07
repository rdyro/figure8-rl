dir_path = @__DIR__
push!(LOAD_PATH, dir_path * "/sim") # simulation
push!(LOAD_PATH, dir_path * "/vis") # visualization
push!(LOAD_PATH, dir_path * "/adv") # adversary policies
push!(LOAD_PATH, dir_path * "/dis") # discretization
push!(LOAD_PATH, dir_path * "/olm") 
push!(LOAD_PATH, dir_path * "/pomdp") 
push!(LOAD_PATH, dir_path * "/tap") 
using sim
using vis
using adv
using dis
using olm
using pomdp
using tap

using Serialization
using Printf
using DelimitedFiles
using Statistics

include("update_info.jl")

function discretize_adv(world::World)
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

# Scenario: Racer scenario
function main(use_tape::Bool=false)
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

	len_rd = path.S[end]

  # forward search agent ---------------------------------------------------- #
  x01 = [len_rd - 50.0; 25.0; 0.0]
  agent1 = Agent(1, copy(x01), vis.make_car(context))
	agent1.controller! = olm.controller_pofs!
  agent1.custom = [0.0, 0.0]
  b0 = fill(1 / length(pomdp.DRIVERS), length(pomdp.DRIVERS))
  b = b0
	ctrl_d = discretize_adv(world)
  # ------------------------------------------------------------------------- #

  v_track = 25.5
	p_track = 0.0
  x02 = [len_rd / 2 - 50.0; v_track; 0]
  agent2 = Agent(2, copy(x02), vis.make_car(context, [1.0, 0.0, 0.0]))
	agent2.controller! = pomdp.adv_controller!
	agent2.custom = [v_track, p_track, pomdp.NOTHING, ctrl_d, pomdp.WEAK]

  push!(world.agents, agent1)
  push!(world.agents, agent2)

  # make diagnostics render objects
  info1 = vis.InfoBox(context, 0.75, 0.75, vis_scaling)
  info2 = vis.InfoBox(context, -0.75, -0.25, vis_scaling)
	
  # main loop for rendering and simulation
  window = true
  h = 1e-2
  dt = 1 / 60

  tf = 4.0
  N = round(Int, tf / dt)
  D = 4
  global T = Array{Float64, 2}(undef, (N, D))
  global Cfwds = Array{Float64, 2}(undef, (N, D))
 
  do_vis = false
  for j in 1:D
    t = 0.0
    agent1.x = copy(x01)
    agent2.x = copy(x02)
    b = b0
    agent2.custom[5] = pomdp.WEAK
    for i in 1:N
      if do_vis == true
        tape = tap.Tape(world)
        tape.info_objs[1] = info1
        tape.info_objs[2] = info2
        tap.add_frame!(tape)
      end

      t1 = time_ns() / 1e9
      (u, ret) = olm.plan_pofs(agent1.x, b, agent1, agent2, world, 
                               pomdp.reward, ctrl_d, j)
      t2 = time_ns() / 1e9
      Cfwds[i, j] = t2 - t1
      agent1.custom = u


      (o, _) = adv.replan_adv(agent2, world)
      agent2.custom[3] = o
      (c, cv) = predict_collision(agent2.x, agent1.x, world)
      b = pomdp.update_belief(b, o, c)

      ## advance one frame in time
      advance!(sim.default_dynamics!, agent1.x, Pair(agent1, world), 0.0, dt, h)
      advance!(agent2.dynamics!, agent2.x, Pair(agent2, world), 0.0, dt, h)

      ## visualize
      if do_vis == true
        diag = diagnostic(agent1, world, 0.0)
        (x, y, sx, sy, dx, u) = diag
        (_, cv) = predict_collision(agent1.x, agent2.x, world)
        tap.add_to_frame!(tape, agent1.x, diag, cv)

        diag = diagnostic(agent2, world, 0.0)
        (x, y, sx, sy, dx, u) = diag
        (_, cv) = predict_collision(agent2.x, agent1.x, world)
        tap.add_to_frame!(tape, agent2.x, diag, cv)
      end

      T[i, j] = t
      t += dt

      if mod(i, div(N, 10)) == 0
        println("$(j) -> $(t)")
      end

      if do_vis == true
        to_vis = tap.visualize_frame(tape, 1)
        window = vis.visualize(context, to_vis)
      end

    end
    println(agent1.x)
    println("Finished depth = $(j)")
  end

  DATAfwds = fill(0.0, (N, 2 * D))
  for i in 1:D
    DATAfwds[:, 2 * (i - 1) + 1] = T[:, i]
    DATAfwds[:, 2 * (i - 1) + 2] = Cfwds[:, i]
  end
  writedlm("../data/pofs_data.txt", DATAfwds)

  AVGCfwds = map(i -> mean(DATAfwds[:, 2 * (i - 1) + 2]), 1:D)
  writedlm("../data/pofs_avgc.txt", AVGCfwds)
end
