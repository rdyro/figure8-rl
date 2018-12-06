dir = @__DIR__
push!(LOAD_PATH, dir * "/../pomdp")
push!(LOAD_PATH, dir * "/../adv")
using adv
using pomdp

mutable struct PofsNode
  x::AbstractArray{Float64, 1}
  u::AbstractArray{Float64, 1}
  b::AbstractArray{Float64, 1}
  adv_x::AbstractArray{Float64, 1}
  c::pomdp.Collision
  r::Float64

  PofsNode() = new(Float64[], Float64[], pomdp.Collision(), 0.0)
  PofsNode(x) = new(x, Float64[], pomdp.Collision(), 0.0)
  PofsNode(x, u) = new(x, u, pomdp.Collision(), 0.0)
end

function plan_pofs(x::AbstractArray{Float64, 1}, agent::Agent, 
                   adv_agent::Agent, world::World, reward::Function, 
                   ctrl_d::Discretization, depth::Int)
  # backup variables
  adv_agent_controller! = adv_agent.controller!
  adv_agent_custom = adv_agent.custom

  # collision from the perspective of the adversary!!!
  value = PofsNode()
  value.x = x
  # uniform prior belief
  value.b = fill(1.0 / length(pomdp.ACTIONS), length(pomdp.ACTIONS)) 
  value.adv_x = adv_agent.x

  adv_agent.controller = pomdp.adv_controller!
  (c, _) = adv.predict_collision(adv_agent.x, agent.x, world)
  value.c = c

  root = GroupTree(value)

  rs = select_action_pofs(root, agent, adv_agent, world, reward, ctrl_d, depth)
  us = Float64[]
  for node_obs in root.next
    for node in node_obs 
      if node.value.r >= rs
        us = node.value.u
      end
    end
  end

  # revert variables
  adv_agent.controller! = adv_agent_controller!
  adv_agent.custom = adv_agent_custom

  if us == Float64[]
    error("Action empty, error in POFS")
  end

  return us
end

function select_action_pofs(node::Tree, agent::Agent, adv_agent::Agent, 
                            world::World, reward::Function, 
                            ctrl_d::Discretization, depth::Int)
  if depth <= 0
    return 0.0
  end

  node.value.x[1] = mod(node.value.x[1], world.road.path.S[end])

  # allocate next level of the tree
  la_len = ctrl_d.thr[end] * ctrl_d.pt[end] # number of actions to survey
  node.next = Array{Array{GroupTree, 1}, 1}(undef, la_len)
  for aidx in 1:la_len
    node.next[aidx] = Array{GroupTree, 1}(undef, length(ACTIONS))
  end

  # predict adv agent doing three possible actions
  adv_NX = Array{Array{Float64, 1}, 1}(undef, length(ACTIONS))
  for o in ACTIONS
    oidx = Int(o)

    adv_nx = copy(node.value.adv_x)
    adv_u = fill(0.0, length(u))
    adv_agent.custom[3] = o
    sim.advance!(sim.default_dynamics!, adv_nx, Pair(adv_agent, world), 0.0,
                 olm_dt, olm_h)
    adv_nx[1] = mod(adv_nx[1], world.road.path.S[end])

    adv_NX[oidx] = adv_nx
  end

  # for each action, evaluate
  max_r = -Inf
  for la in 0:(la_len - 1)
    u = dis.ls2x(ctrl_d, la)
    agent.custom = u
    nx = copy(node.value.x)
    sim.advance!(sim.default_dynamics!, nx, Pair(agent, world), 0.0, olm_dt, 
                 olm_h)
    nx[1] = mod(nx[1], world.road.path.S[end])

    ra = 0.0
    for o in ACTIONS
      oidx = Int(o)
      adv_nx = adv_NX[oidx]
      (nc, _) = adv.predict_collision(nx, adv_nx, world)
      r = reward(vcat(node.value.x, nc.d, nc.t), u, nx, agent, world)
      bp = pomdp.update_belief(node.value.b, o, node.value.c)

      value = PofsNode()
      value.x = nx
      value.u = u
      value.b = bp
      value.adv_x = adv_nx
      value.nc = nc

      next_node = GroupTree(value)
      next_r = select_action_pofs(next_node, agent, adv_agent, world, reward, 
                                  ctrl_d, depth - 1)
      r += olm_gamma * next_r
      next_node.r = r
      node.next[la + 1][oidx] = next_node

      ra += r * reduce(+, map(i -> b[i] * P_adv(o, value.c, pomdp.DRIVERS[i]), 
                              1:length(pomdp.DRIVERS)))
    end

    if ra > max_r
      max_r = ra
    end
  end

  return max_r
end

function controller_pofs!(u::AbstractArray{Float64}, 
                          x::AbstractArray{Float64}, 
                          dx::AbstractArray{Float64}, 
                          agent_world::Pair{Agent, World}, t::Float64)
  agent = agent_world.first
  u[1] = agent.custom[1]
  u[2] = agent.custom[2]
end
