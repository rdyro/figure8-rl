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
const min_acc = -10
const max_acc = 10
const min_ste = -1e-1
const max_ste = 1e-1

function discretize(world::World)
  max_s = world.road.path.S[end]
  state_d = dis.Discretization([pts_per_s, pts_per_ds, pts_per_p],
                               [min_s, min_ds, min_p],
                               [max_s, max_ds, max_p])
  ctrl_d = dis.Discretization([pts_per_acc, pts_per_ste],
                              [min_acc, min_ste],
                              [max_acc, max_ste])
  return (state_d, ctrl_d)
end



function controllerd_interp!(u::AbstractArray{Float64},
                             x::AbstractArray{Float64},
                             dx::AbstractArray{Float64},
                             agent_world::Pair{Agent, World}, t::Float64)
  agent = agent_world.first
  world = agent_world.second

  P = agent.custom[1]
  state_d = agent.custom[2]
  ctrl_d = agent.custom[3]

  x = copy(agent.x)
  x[1] = mod(x[1], state_d.mx[1])
  dis.clamp_x!(state_d, x)

  # interpolate control
  X = dis.bounding_X(state_d, x)
  U1 = fill(0.0, length(X))
  U2 = fill(0.0, length(X))
  for i in 1:length(X)
    ls = dis.x2ls(state_d, X[i])
    lu = P.S[ls].a[P.Aidx[ls]]
    uv = dis.ls2x(ctrl_d, lu)
    U1[i] = uv[1]
    U2[i] = uv[2]
  end
  u[1] = dis.interp(X, x, U1)
  u[2] = dis.interp(X, x, U2)

  #=
  ls = dis.x2ls(state_d, x)
  lu = P.S[ls].a[P.Aidx[ls]]
  uv = dis.ls2x(ctrl_d, lu)
  u[1] = uv[1]
  u[2] = uv[2]
  =#
end

function controllerd_train!(u::AbstractArray{Float64},
                            x::AbstractArray{Float64},
                            dx::AbstractArray{Float64},
                            agent_world::Pair{Agent, World}, t::Float64)
  agent = agent_world.first
  world = agent_world.second

  ctrl_d = agent.custom[3]
  lu = agent.custom[4]
  agent_u = dis.ls2x(ctrl_d, lu)

  u[1] = agent_u[1]
  u[2] = agent_u[2]
end

function dynamicsd!(dx::AbstractArray{Float64},
                    x::AbstractArray{Float64},
                    agent_world::Pair{Agent, World}, t::Float64)

  agent = agent_world.first
  world = agent_world.second


  state_d = agent.custom[2]
  sim.default_dynamics!(dx, x, agent_world, t)

  x[1] = mod(x[1], state_d.mx[1])
  dis.clamp_x!(state_d, x)
end

function reward(world, agent, x, a, nx)
  return abs(nx[3]) > 0.35 * world.road.width ? -1e9 : nx[2]^3
end


const dt = 1.5 # time increment for planning
const step_h = 1e-1 # dt for numerical integration
function make_MDP(world::World)
  @assert length(world.agents) == 1
  agent = world.agents[1]
  agent.controller! = controllerd_train!

  (state_d, ctrl_d) = discretize(world)

  S = Dict{Int, DetState}()
  for s in 0:(prod(state_d.pt) - 1)
    a = collect(0:(prod(ctrl_d.pt) - 1))
    a2r = fill(0.0, length(a))
    ns = fill(-1, length(a))

    xd = dis.ls2xd(state_d, s)
    x = dis.xd2x(state_d, xd)

    for i in 1:length(a)
      nx = copy(x)
      lu = a[i]
      agent.custom = (nothing, state_d, ctrl_d, lu)
      #println("Before = $(nx)")
      advance!(agent.dynamics!, nx, Pair(agent, world), 0.0, dt, step_h)
      #println("After  = $(nx)")

      # enforce max values
      dis.clamp_x!(state_d, nx)
      nxd = dis.x2xd(state_d, nx)
      ns[i] = dis.xd2ls(state_d, nxd)

      a2r[i] = reward(world, agent, x, a[i], nx)
    end

    S[s] = DetState(a, a2r, ns)
  end

  agent.controller! = sim.default_controller!

  return S
end
