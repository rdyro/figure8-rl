dir = @__DIR__
push!(LOAD_PATH, dir * "../sim")
push!(LOAD_PATH, dir * "../dis")

using sim
using dis

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

const vit_dt = 1.5 # time increment for planning
const vit_h = 1e-1 # dt for numerical integration
function make_MDP(agent::Agent, world::World, reward::Function, 
                  state_d::Discretization, ctrl_d::Discretization)
  agent.controller! = controllerd_train!

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
      advance!(agent.dynamics!, nx, Pair(agent, world), 0.0, vit_dt, vit_h)

      # enforce max values
      dis.clamp_x!(state_d, nx)
      nxd = dis.x2xd(state_d, nx)
      ns[i] = dis.xd2ls(state_d, nxd)
      u = dis.ls2x(ctrl_d, lu)

      a2r[i] = reward(x, u, nx, agent, world)
    end

    S[s] = DetState(a, a2r, ns)
  end

  return S
end

