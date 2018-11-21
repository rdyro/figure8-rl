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

function xd2ls(xd::AbstractArray{Int})
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

function bounding_xd(x::AbstractArray{<: Real}, road::Road)
  dim = 3
  @assert length(x) == 3

  xd = x2xd(x, road)
  nx = xd2x(xd, road)

  XD = [copy(xd) for i in 1:2^dim]

  for i in 1:dim
    add = fill(0, dim)
    add[i] = 1
    use_add = 0

    if nx[i] >= x[i]
      add .*= -1
      use_add = 1
    end

    j = 0
    while j < length(XD)
      for k in 1:2^(i - 1)
        XD[j + k] += use_add * add
      end
      j += 2^(i - 1)
      use_add = use_add == 0 ? 1 : 0
    end
  end

  return XD
end

function bounding_x(x::AbstractArray{<: Real}, road::Road)
  XD = bounding_xd(x, road)
  X = [xd2x(xd, road) for xd in XD]
  return X
end

function interp(X::AbstractArray, x::AbstractArray, v::AbstractArray)
  dim = Int(log2(length(X)))

  xi = fill(0.0, dim)
  for i in 1:dim
    xi[i] = (x[i] - X[1][i]) / (X[end][i] - X[1][i])
  end

  c = v # no need to copy
  for i in 1:dim
    len = 2^(dim - i)
    nc = fill(0.0, len)
    for j in 1:len
      nc[j] = c[2 * (j - 1) + 1] * (1.0 - xi[i]) + c[2 * (j - 1) + 2] * xi[i]
    end
    c = nc
  end

  return c[]
end


function controllerd_interp!(u::AbstractArray{Float64},
                             x::AbstractArray{Float64},
                             dx::AbstractArray{Float64},
                             agent_world::Pair{Agent, World}, t::Float64)
  agent = agent_world.first
  world = agent_world.second

  #=
  ud = lu2ud(agent.custom)
  agent_u = ud2u(ud)

  u[1] = agent_u[1]
  u[2] = agent_u[2]
  =#

  u[1] = agent.custom[1]
  u[2] = agent.custom[2]

  return
end

function controllerd_train!(u::AbstractArray{Float64},
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
  agent.controller! = controllerd_train!

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
      a2r[i] = abs(nx[3]) > 0.35 * world.road.width ? -1e5 : (nx[1] - x[1])^3
    end

    S[s] = DetState(a, a2r, ns)
  end

  agent.controller! = sim.default_controller!

  return S
end
