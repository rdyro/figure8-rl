dir = @__DIR__
push!(LOAD_PATH, dir * "../pomdp")
using pomdp

mutable struct PofsNode
  x::AbstractArray{Float64, 1}
  u::AbstractArray{Float64, 1}
  b::AbstractArray{Float64, 1}
  adv_x::AbstractArray{Float64, 1}
  c::Collision
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
  value.b = fill(1.0 / length(ACTIONS), length(ACTIONS)) 
  value.adv_x = adv_agent.x

  adv_agent.controller = adv_controller!
  adv_agent.custom[3] = NOTHING
  (d, t, ctype) = predict_collision(adv_agent.x, adv_agent, agent.x, agent, 
                                    world)
  c = Collision(d, t, ctype)
  value.c = c

  root = GroupTree(value)

  rs = select_action_pofs(root, agent, adv_agent, world, reward, ctrl_d, depth)
  us = Float64[]

  # revert variables
  adv_agent.controller! = adv_agent_controller!
  adv_agent.custom = adv_agent_custom

  return us
end

function select_action_pofs(node::Tree, agent::Agent, world::World, 
                            reward::Function, ctrl_d::Discretization, 
                            depth::Int)
  if depth <= 0
    return 0.0
  end

  max_r = -Inf
  us = Float64[]

  node.value.x[1] = mod(node.value.x[1], world.road.path.S[end])

  la_len = ctrl_d.thr[end] * ctrl_d.pt[end] # number of actions to survey
  # allocate next level of the tree
  node.next = Array{Array{GroupTree, 1}, 1}(undef, la_len)
  for aidx in 1:la_len
    node.next[aidx] = Array{GroupTree, 1}(undef, length(ACTIONS))
  end
  adv_NX = Array{Array{Float64, 1}, 1}(undef, length(ACTIONS))
  adv_U = Array{Array{Float64, 1}, 1}(undef, length(ACTIONS))
  for o in ACTIONS
    oidx = Int(o)

    adv_nx = copy(node.value.adv_x)
    adv_u = fill(0.0, length(u))

    adv_agent.custom[3] = o
    sim.advance!(sim.default_dynamics!, adv_nx, Pair(adv_agent, world), 0.0,
                 olm_dt, olm_h)
    adv_nx[1] = mod(adv_nx[1], world.road.path.S[end])

    adv_NX[oidx] = adv_nx
    adv_U[oidx] = adv_u
  end
  for la in 0:(la_len - 1)
    # take an action
    u = dis.ls2x(ctrl_d, la)
    adv_u = 
    agent.custom = u
    nx = copy(node.value.x)
    sim.advance!(sim.default_dynamics!, nx, Pair(agent, world), 0.0, olm_dt, 
                 olm_h)
    nx[1] = mod(nx[1], world.road.path.S[end])
    for o in ACTIONS
      oidx = Int(o)
      adv_nx = adv_NX[oidx]
      adv_u = fill(0.0, length(u))
      adv_agent.custom[3] = adv_u
      sim.advance!(sim.default_dynamics!, adv_nx, Pair(adv_agent, world), 0.0,
                   olm_dt, olm_h)
      adv_nx[1] = mod(adv_nx[1], world.road.path.S[end])

      nc = predict_collision(nx, adv_nx, world)
      r = reward(vcat(node.value.x, nc.d, nc.t), u, nx, agent, world)

      bp = update_belief(node.value.b, o, node.value.c)
    end

    r = map(adv_a -> reward(adv_a)) .* 
              map(adv_a -> sum(node.value.b * P(adv_a, collision_analysis)), A)

    value = PofsNode(nx, u)
    node.next[la + 1] = Tree(value)
    next_r = select_action_pofs(node.next[la + 1], agent, world, reward, 
                                ctrl_d, depth - 1)
    r = reward(node.value.x, u, nx, agent, world) + olm_gamma * next_r
    node.next[la + 1].value.r = r

    if r > max_r
      max_r = r
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
