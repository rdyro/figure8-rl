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
  #b0 = [0.0, 1.0, 0.0]
  b = b0
	ctrl_d = discretize_adv(world)
  # ------------------------------------------------------------------------- #

  v_track = 25.5
	p_track = 0.0
  x02 = [len_rd / 2 - 50.0; v_track; 0]
  agent2 = Agent(2, copy(x02), vis.make_car(context, [1.0, 0.0, 0.0]))
	agent2.controller! = pomdp.adv_controller!
	agent2.custom = [v_track, p_track, pomdp.NOTHING, ctrl_d, pomdp.STRONG]

  push!(world.agents, agent1)
  push!(world.agents, agent2)

  # make diagnostics render objects
  info1 = vis.InfoBox(context, 0.75, 0.75, vis_scaling)
  info2 = vis.InfoBox(context, -0.75, -0.25, vis_scaling)
	
  # main loop for rendering and simulation
  window = true
  h = 1e-2
  t0 = time_ns()
  oldt = (time_ns() - t0) / 1e9

  if use_tape == true
    tape = tap.Tape(world) # make the tap
    tape.info_objs[1] = info1
    tape.info_objs[2] = info2
    frame_nb = 220
    dt = 1 / 60
    for i in 1:frame_nb
      tap.add_frame!(tape)

      if mod(agent1.x[1], road.path.S[end]) < 150.0 && 
        mod(agent1.x[1], path.S[end]) > 60.0
        agent1.x = copy(x01)
        agent2.x = copy(x02)

        b = b0
        println("RESETTING")
      end

      for agent in world.agents
        cv = nothing
        if agent.id == 1
          (_, cv) = predict_collision(agent1.x, agent2.x, world)
        elseif agent.id == 2
          global (u, ret) = olm.plan_pofs(agent1.x, b, agent1, agent2, world, pomdp.reward,
                            ctrl_d, 3)
          agent1.custom = u

          (o, _) = adv.replan_adv(agent, world)
          agent.custom[3] = o
          (c, cv) = predict_collision(agent2.x, agent1.x, world)
          b = pomdp.update_belief(b, o, c)
          println(b)
        end

        ## advance one frame in time
        advance!(agent.dynamics!, agent.x, Pair(agent, world), 0.0, dt, h)

        ## visualize
        diag = diagnostic(agent, world, 0.0)
        (x, y, sx, sy, dx, u) = diag
        agent.is_braking = dx[1] * u[1] < 0

        ######################################
        tap.add_to_frame!(tape, agent.x, diag, cv)
      end
      #to_vis = tap.visualize_frame(tape, 1)
      #window = vis.visualize(context, to_vis)
    end
    i = 1
    while window
      to_vis = tap.visualize_frame(tape, i)
      window = vis.visualize(context, to_vis)

      i += 1
      i = mod(i - 1, frame_nb) + 1
    end
  else
    while window
      t = (time_ns() - t0) / 1e9

      to_visualize = []
      if world.road.road != nothing
        push!(to_visualize, world.road.road)
      end

      if mod(agent1.x[1], road.path.S[end]) < 150.0 && 
        mod(agent1.x[1], path.S[end]) > 60.0
        agent1.x = copy(x01)
        agent2.x = copy(x02)

        b = fill(1 / length(pomdp.DRIVERS), length(pomdp.DRIVERS))
        println("RESETTING")
      end

      for agent in world.agents
        cv = nothing
        if agent.id == 1
          global (u, ret) = olm.plan_pofs(agent1.x, b, agent1, agent2, world, pomdp.reward,
                            ctrl_d, 3)
          agent1.custom = u
          (_, cv) = predict_collision(agent1.x, agent2.x, world)

          update_info(info1, agent, world, t, cv)
        elseif agent.id == 2
          (o, _) = adv.replan_adv(agent, world)
          agent.custom[3] = o
          (c, cv) = predict_collision(agent2.x, agent1.x, world)
          b = pomdp.update_belief(b, o, c)

          update_info(info2, agent, world, t, cv)
        end

        ## advance one frame in time
        dt = 0.1
        advance!(agent.dynamics!, agent.x, Pair(agent, world), 0.0, dt, h)

        ## visualize
        diag = diagnostic(agent, world, t)
        (x, y, sx, sy, dx, u) = diag
        agent.is_braking = dx[1] * u[1] < 0

        ######################################
        #tap.add_to_frame!(tape, agent.x, diag, cv)
        
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

      push!(to_visualize, info1)
      push!(to_visualize, info2)

      #to_vis = tap.visualize_frame(tape, 1)
      window = vis.visualize(context, to_visualize)
      #window = vis.visualize(context, to_vis)

      oldt = t
    end
  end
end
