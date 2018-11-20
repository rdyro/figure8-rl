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

# time increment
const dt = 0.5
const step_h = 1e-1

# map linear state to state vector
function ls2xd(ls::Int)
  pd = div(ls, pts_per_s * pts_per_ds)
  ls = rem(ls, pts_per_s * pts_per_ds)
  dsd = div(ls, pts_per_s)
  sd = rem(ls, pts_per_s)

  return [sd, dsd, pd]
end

function xd2ls(xd::AbstractArray{<: Real})
  sd = xd[1]
  dsd = xd[2]
  pd = xd[3]

  return sd + (dsd + pd * pts_per_ds) * pts_per_s
end

# map state vector to a discrete state vector
function x2xd(x::AbstractArray{<: Real}, road::Road)
  max_s = road.path.S[end]

  # enforce position within max_s
  x[1] = mod(x[1], max_s)

  s = x[1]
  ds = x[2]
  p = x[3]

  sd = round(Int, (pts_per_s - 1) * (s - min_s) / (max_s - min_s))
  dsd = round(Int, (pts_per_ds - 1) * (ds - min_ds) / (max_ds - min_ds))
  pd = round(Int, (pts_per_p - 1) * (p - min_p) / (max_p - min_p))

  return [sd, dsd, pd]
end

function xd2x(xd::AbstractArray{<: Real}, road::Road)
  max_s = road.path.S[end]

  sd = xd[1]
  dsd = xd[2]
  pd = xd[3]

  s = (sd / (pts_per_s - 1)) * (max_s - min_s) + min_s
  ds = (dsd / (pts_per_ds - 1)) * (max_ds - min_ds) + min_ds
  p = (pd / (pts_per_p - 1)) * (max_p - min_p) + min_p

  return [s, ds, p]
end

# map linear input to vector input
function lu2ud(lu::Int)
  sted = div(lu, pts_per_acc)
  accd = rem(lu, pts_per_acc)

  return [accd, sted]
end

function ud2lu(ud::AbstractArray{<: Real})
  accd = ud[1]
  sted = ud[2]

  return accd + sted * pts_per_acc
end

# map input vector to a discrete input vector
function u2ud(u::AbstractArray{<: Real})
  acc = u[1]
  ste = u[2]

  accd = round(Int, (pts_per_acc - 1) * (acc - min_acc) / (max_acc - min_acc))
  sted = round(Int, (pts_per_ste - 1) * (ste - min_ste) / (max_ste - min_ste))

  return [accd, sted]
end

function ud2u(ud::AbstractArray{<: Int})
  accd = ud[1]
  sted = ud[2]

  acc = (accd / (pts_per_acc - 1)) * (max_acc - min_acc) + min_acc
  ste = (sted / (pts_per_ste - 1)) * (max_ste - min_ste) + min_ste

  return [acc, ste]
end


function controllerd!(u::AbstractArray{Float64},
                      x::AbstractArray{Float64},
                      dx::AbstractArray{Float64},
                      agent_world::Pair{Agent, World}, t::Float64)
  agent = agent_world.first
  world = agent_world.second

  ud = lu2ud(agent.custom)
  agent_u = ud2u(ud)

  u[1] = agent_u[1]
  u[2] = agent_u[2]

  return
end

function dynamicsd!(dx::AbstractArray{Float64},
                    x::AbstractArray{Float64},
                    agent_world::Pair{Agent, World}, t::Float64)

  agent = agent_world.first
  world = agent_world.second

  max_s = world.road.path.S[end]

  sim.default_dynamics!(dx, x, agent_world, t)

  # enforce max values
  x[1] = x[1] > max_s ? max_s : x[1]
  x[1] = x[1] < min_s ? min_s : x[1]

  x[2] = x[2] > max_ds ? max_ds : x[2]
  x[2] = x[2] < min_ds ? min_ds : x[2]

  x[3] = x[3] > max_p ? max_p : x[3]
  x[3] = x[3] < min_p ? min_p : x[3]

  return
end

function make_MDP(world::World)
  @assert length(world.agents) == 1
  agent = world.agents[1]
  agent.controller! = controllerd!

  S = Dict{Int, DetState}()

  max_s = world.road.path.S[end]

  for s in 0:(pts_per_s * pts_per_ds * pts_per_p)
    a = collect(0:(pts_per_acc * pts_per_ste - 1))
    a2r = fill(0.0, length(a))
    ns = fill(-1, length(a))

    xd = ls2xd(s)
    x = xd2x(xd, world.road)

    for i in 1:length(a)
      nx = copy(x)
      agent.custom = a[i]
      advance!(agent.dynamics!, nx, Pair(agent, world), 0.0, dt, step_h)

      # enforce max values
      nx[1] = nx[1] > max_s ? max_s : nx[1]
      nx[1] = nx[1] < min_s ? min_s : nx[1]

      nx[2] = nx[2] > max_ds ? max_ds : nx[2]
      nx[2] = nx[2] < min_ds ? min_ds : nx[2]

      nx[3] = nx[3] > max_p ? max_p : nx[3]
      nx[3] = nx[3] < min_p ? min_p : nx[3]

      nxd = x2xd(nx, world.road)

      ns[i] = xd2ls(nxd)
      a2r[i] = abs(nxd[3]) > 0.7 * world.road.width ? -1e5 : nxd[2]^2
    end

    S[s] = DetState(a, a2r, ns)
  end

  agent.controller! = sim.default_controller!

  return S
end
