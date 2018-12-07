const olm_mcts_c = 1e5 # exploration parameter for MCTS
const olm_mcts_iterations = 300 # exploration parameter for MCTS

mutable struct MctsNode
  x::AbstractArray{Float64, 1}
  u::AbstractArray{Float64, 1}
  q::Float64
  N::Int
  Ns::Int

  MctsNode() = new(Float64[], Float64[], 0.0, 1, 1)
  MctsNode(x, u, r) = new(x, u, r, 1, 1)
end

function plan_mcts(x::AbstractArray{Float64, 1}, agent::Agent, world::World, 
                   reward::Function, ctrl_d::Discretization, depth::Int)
  x = copy(x)
  x[1] = mod(x[1], world.road.path.S[end])
  x = copy(x)
  value = MctsNode()
  value.x = x
  node = Tree(value)

  num_iterations = round(Int, olm_mcts_iterations / 4 * depth)

  visited = Set{MctsNode}()
  for i in 1:num_iterations
    simulate_mcts(node, agent, world, reward, 
                  ctrl_d, visited, depth)
  end
  
  us = Float64[]
  qs = -Inf
  for a in 1:length(node.next)
    if node.next[a].value.q > qs
      qs = node.next[a].value.q
      us = node.next[a].value.u
    end
  end

  return us
end

function simulate_mcts(node::Tree, agent::Agent, world::World, 
                       reward::Function, ctrl_d::Discretization, 
                       visited::Set{MctsNode}, depth::Int)
  if depth <= 0
    return 0.0
  end

  if !(node.value in visited)
    push!(visited, node.value)

    la_len = ctrl_d.thr[end] * ctrl_d.pt[end] # number of actions to survey
    node.next = Array{Tree, 1}(undef, la_len)
    for la in 0:(la_len - 1)
      u = dis.ls2x(ctrl_d, la)
      agent.custom = u
      nx = copy(node.value.x)
      sim.advance!(sim.default_dynamics!, nx, Pair(agent, world), 0.0, olm_dt, 
                   olm_h)
      nx[1] = mod(nx[1], world.road.path.S[end])

      r = reward(node.value.x, u, nx, agent, world)

      value = MctsNode(nx, u, r)
      node.next[la + 1] = Tree(value)
    end

    return rollout(node.value.x, agent, world, reward, ctrl_d, depth)
  end

  # find the best action for value and exploration
  as = -1
  vs = -Inf
  for a in 1:length(node.next)
    v = node.next[a].value.q + olm_mcts_c * sqrt(log(node.value.Ns) / 
                                                 node.next[a].value.N)
    if v > vs
      vs = v
      as = a
    end
  end

  # transition to the new state deterministically
  x = node.value.x
  u = node.next[as].value.u
  nx = node.next[as].value.x
  r = reward(x, u, nx, agent, world)

  q = r + olm_gamma * simulate_mcts(node.next[as], agent, world, reward, 
                                    ctrl_d, visited, depth - 1)

  # update Qsa for fuck's sake ---------------------------------------------- #
  #dq = (q - node.next[as].value.q)
  #node.next[as].value.q += dq / node.next[as].value.N

  # this works best BY FAR
  node.next[as].value.q = max(q, node.next[as].value.q)

  #alpha = 0.5
  #Qsa = node.next[as].value.q
  #Qsa = Qsa * (1 - alpha) + q * alpha
  #node.next[as].value.q = Qsa
  # ------------------------------------------------------------------------- #

  node.next[as].value.N += 1
  node.value.Ns += 1

  return q
end

function rollout(x::AbstractArray{Float64, 1}, agent::Agent, 
                 world::World, reward::Function, 
                 ctrl_d::Discretization, depth::Int)
  if depth <= 0
    return 0.0
  end

  # choose middle action, generally reasonable
  ad = fill(0, ctrl_d.dim)
  for i in 1:length(ad)
    ad[i] = div(ctrl_d.pt[i], 2)
  end
  u = dis.xd2x(ctrl_d, ad)
  agent.custom = u
  nx = copy(x)
  sim.advance!(sim.default_dynamics!, nx, Pair(agent, world), 0.0, olm_dt, 
               olm_h)
  r = reward(x, u, nx, agent, world)

  return r + olm_gamma * rollout(nx, agent, world, reward, ctrl_d, depth - 1)
end

function controller_mcts!(u::AbstractArray{Float64}, 
                          x::AbstractArray{Float64}, 
                          dx::AbstractArray{Float64}, 
                          agent_world::Pair{Agent, World}, t::Float64)
  agent = agent_world.first
  u[1] = agent.custom[1]
  u[2] = agent.custom[2]
end
