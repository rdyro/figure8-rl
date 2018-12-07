const olm_pomc_c = 1e5 # exploration parameter for MCTS
const olm_pomc_iterations = 300 # exploration parameter for MCTS
const olm_pomc_dt = 0.5 # exploration parameter for MCTS

mutable struct PomcNode
  x::AbstractArray{Float64, 1}
  u::AbstractArray{Float64, 1}
  b::AbstractArray{Float64, 1}
  adv_x::AbstractArray{Float64, 1}
  c::pomdp.Collision
  q::Float64
  N::Int
  Ns::Int

  PomcNode() = new(Float64[], Float64[], Float64[], Float64[], 
                   pomdp.Collision(), 0.0, 1, 1)
end

function plan_pomc(x::AbstractArray{Float64, 1}, b::AbstractArray{Float64, 1},
                   agent::Agent, adv_agent::Agent, world::World, 
                   reward::Function, ctrl_d::Discretization, depth::Int)
  #backup variables
  adv_agent_controller! = adv_agent.controller!
  adv_agent_custom = adv_agent.custom

  x = copy(x)
  x[1] = mod(x[1], world.road.path.S[end])

  value = PomcNode()
  value.x = x
  value.b = b
  value.adv_x = adv_agent.x
  adv_agent.controller! = pomdp.adv_controller!
  (c, _) = adv.predict_collision(adv_agent.x, agent.x, world)
  value.c = c
  node = GroupTree(value)

  num_iterations = round(Int, olm_pomc_iterations / 4 * depth)

  visited = Set{PomcNode}()
  for i in 1:num_iterations
    simulate_pomc(node, agent, adv_agent, world, reward, 
                  ctrl_d, visited, depth)
  end

  us = Float64[]
  qs = -Inf
  ret = nothing
  (adv_c, _) = predict_collision(adv_agent.x, x, world)
  P = map(o -> reduce(+, map(i -> node.value.b[i] * 
                             pomdp.P_adv(o, adv_c, pomdp.DRIVERS[i]), 
                             1:length(pomdp.DRIVERS))), pomdp.ACTIONS)
  for a in 1:length(node.next)
    q = reduce(+, map(o -> node.next[a][o].value.q * P[o], 
                      1:length(pomdp.ACTIONS)))
    if q > qs
      qs = q
      us = node.next[a][1].value.u
      ret = node
    end
  end

  # revert variables
  adv_agent.controller! = adv_agent_controller!
  adv_agent.custom = adv_agent_custom

  return (us, node)
end

function simulate_pomc(node::GroupTree, agent::Agent, adv_agent::Agent, 
                       world::World, reward::Function, ctrl_d::Discretization, 
                       visited::Set{PomcNode}, depth::Int)
  if depth <= 0
    return 0.0
  end

  node.value.x[1] = mod(node.value.x[1], world.road.path.S[end])
  (adv_c, _) = predict_collision(node.value.adv_x, node.value.x, world)

  if !(node.value in visited)
    push!(visited, node.value)


    # predict adv agent doing three possible actions
    adv_NX = Array{Array{Float64, 1}, 1}(undef, length(pomdp.ACTIONS))
    for o in pomdp.ACTIONS
      oidx = Int(o)

      adv_nx = copy(node.value.adv_x)
      adv_agent.custom[3] = o
      sim.advance!(sim.default_dynamics!, adv_nx, Pair(adv_agent, world), 0.0,
                   olm_pomc_dt, olm_h)
      adv_nx[1] = mod(adv_nx[1], world.road.path.S[end])

      adv_NX[oidx] = adv_nx
    end

    # allocate next level of the tree
    la_len = ctrl_d.thr[end] * ctrl_d.pt[end] # number of actions to survey
    node.next = Array{Array{GroupTree, 1}, 1}(undef, la_len)
    for aidx in 1:la_len
      node.next[aidx] = Array{GroupTree, 1}(undef, length(pomdp.ACTIONS))
    end
    
    for la in 0:(la_len - 1)
      u = dis.ls2x(ctrl_d, la)
      agent.custom = u
      nx = copy(node.value.x)
      sim.advance!(sim.default_dynamics!, nx, Pair(agent, world), 0.0, 
                   olm_pomc_dt, olm_h)
      nx[1] = mod(nx[1], world.road.path.S[end])

      ra = 0.0
      for o in pomdp.ACTIONS
        oidx = Int(o)
        adv_nx = adv_NX[oidx]
        (nc, _) = adv.predict_collision(nx, adv_nx, world)

        custom = agent.custom
        agent.custom = nc
        r = reward(node.value.x, u, nx, agent, world)
        agent.custom = custom

        bp = pomdp.update_belief(node.value.b, o, adv_c)

        value = PomcNode()
        value.x = nx
        value.adv_x = adv_nx
        value.u = u
        value.b = bp
        value.c = nc
        value.q = r

        next_node = GroupTree(value)
        node.next[la + 1][oidx] = next_node
      end
    end
    return 0.0
  end

  # probability of each observation
  P = map(o -> reduce(+, map(i -> node.value.b[i] * 
                             pomdp.P_adv(o, adv_c, pomdp.DRIVERS[i]), 
                             1:length(pomdp.DRIVERS))), pomdp.ACTIONS)
  # find the best action for value and exploration
  as = -1
  vs = -Inf
  for a in 1:length(node.next)
    # expected reward
    ra = reduce(+, map(o -> node.next[a][o].value.q * P[o], 
                       1:length(pomdp.ACTIONS)))
    v = ra + olm_pomc_c * sqrt(log(node.value.Ns) / 
                               reduce(+, map(o -> node.next[a][o].value.N, 
                                             1:length(pomdp.ACTIONS))))
    if v > vs
      vs = v
      as = a
    end
  end

  # sample a transition
  cP = cumsum(P)
  rnd = rand()
  oidx = findfirst(i -> cP[i] >= rnd, 1:length(cP))

  x = node.value.x
  adv_nx = node.next[as][oidx].value.adv_x
  u = node.next[as][oidx].value.u
  nx = node.next[as][oidx].value.x
  bp = node.next[as][oidx].value.b

  (nc, _) = adv.predict_collision(nx, adv_nx, world)
  custom = agent.custom
  agent.custom = nc
  r = reward(node.value.x, u, nx, agent, world)
  agent.custom = custom

  q = r + olm_gamma * simulate_pomc(node.next[as][oidx], agent, adv_agent, 
                                    world, reward, ctrl_d, visited, depth - 1)

  # update Qsa for fuck's sake ---------------------------------------------- #
  dq = (q - node.next[as][oidx].value.q)
  node.next[as][oidx].value.q += dq / node.next[as][oidx].value.N

  # this works best BY FAR
  #node.next[as].value.q = max(q, node.next[as].value.q)

  #alpha = 0.5
  #Qsa = node.next[as].value.q
  #Qsa = Qsa * (1 - alpha) + q * alpha
  #node.next[as].value.q = Qsa
  # ------------------------------------------------------------------------- #

  node.next[as][oidx].value.N += 1
  node.value.Ns += 1

  return q
end

function rollout(x::AbstractArray{Float64, 1}, 
                 adv_x::AbstractArray{Float64, 1},
                 b::AbstractArray{Float64, 1}, 
                 agent::Agent, adv_agent::Agent,
                 world::World, reward::Function, 
                 ctrl_d::Discretization, depth::Int)
  if depth <= 0
    return 0.0
  end

  return 0.0

  # choose middle action, generally reasonable
end

function controller_pomc!(u::AbstractArray{Float64}, 
                          x::AbstractArray{Float64}, 
                          dx::AbstractArray{Float64}, 
                          agent_world::Pair{Agent, World}, t::Float64)
  agent = agent_world.first
  u[1] = agent.custom[1]
  u[2] = agent.custom[2]
end
