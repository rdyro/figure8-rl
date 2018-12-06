module pomdp
dir_path = @__DIR__
push!(LOAD_PATH, dir_path * "/../sim") # simulation
using sim

@enum CollisionType NO_COLLISION=0 HITTING=1 BEING_HIT=2
@enum DriverType WEAK=1 MEDIUM=2 STRONG=3
@enum DriverAction BRAKE=1 ACC=2 NOTHING=3

const ACTIONS = [BRAKE, ACC, NOTHING]
const DRIVERS = [WEAK, MEDIUM, STRONG]

struct Collision
  d::Float64
  t::Float64
  ctype::Enum

  Collision(d, t, ctype) = new(d, t, ctype)
  Collision() = new(0.0, 0.0, NO_COLLISION)
end

const max_t = 5.0
const max_d = 10.0
function reward(x, u, nx, agent, world)
  c = agent.custom

  base_r = abs(x[3]) > 0.35 * world.road.width ? -1e9 : x[2]^3
  coll_r = 0.0
  if c.ctype != NO_COLLISION
    t = c.t < 1.0 ? 1.0 : c.t
    coll_r += -1e9 * (max_d - c.d) / 10.0 * (max_t - t)
  end

  return base_r + coll_r
  #return coll_r
end

function P_adv(o::Enum, c::Collision, s::Enum)
  if c.ctype != NO_COLLISION
    P = 0.0
    if s == WEAK
      P = o == BRAKE ? 0.9 : P
      P = o == ACC ? 0.05 : P
      P = o == NOTHING ? 0.05 : P
    elseif s == MEDIUM
      if c.ctype == HITTING
        P = o == BRAKE ? 0.9 : P
        P = o == ACC ? 0.05 : P
        P = o == NOTHING ? 0.05 : P
      elseif c.ctype == BEING_HIT
        P = o == BRAKE ? 0.05 : P
        P = o == ACC ? 0.9 : P
        P = o == NOTHING ? 0.05 : P
      end
    elseif s == STRONG
      P = o == BRAKE ? 0.05 : P
      P = o == ACC ? 0.05 : P
      P = o == NOTHING ? 0.9 : P
    end

    return P
  else
    if o == BRAKE || o == ACC
      return 0.0
    elseif o == NOTHING
      return 1.0
    end
  end
end

function update_belief(b::AbstractArray{Float64, 1}, o::Enum, c::Collision)
  bp = similar(b)
  bp_sum = 0.0
  for i in 1:length(b)
    #println("i -> $(i), b[i] -> $(b[i]), P -> $(P_adv(o, c, DRIVERS[i]))")
    bp[i] = b[i] * P_adv(o, c, DRIVERS[i])
    bp_sum += bp[i]
  end
  bp ./= sum(bp)

  return bp_sum != 0.0 ? bp : b
end

function sample_adv_a(dt::Enum, c::Collision)
  if c.ctype != NO_COLLISION
    # set cP
    cP = Float64[]
    if dt == WEAK
      cP = [0.9, 0.95, 1.0]
    elseif dt == MEDIUM
      if c.ctype == HITTING
        cP = [0.9, 0.95, 1.0]
      elseif c.ctype == BEING_HIT
        cP = [0.05, 0.95, 1.0]
      end
    elseif dt == STRONG
      cP = [0.05, 0.10, 1.0]
    end

    # sample
    r = rand()
    for i in 1:length(ACTIONS)
      if cP[i] >= r
        return ACTIONS[i]
      end
    end
  else
    return NOTHING
  end
end

function adv_controller!(u::AbstractArray{Float64, 1},
                         x::AbstractArray{Float64, 1}, 
                         dx::AbstractArray{Float64, 1}, 
                         agent_world::Pair{Agent, World},
                         t::Float64)
  agent = agent_world.first

  v_track = agent.custom[1]
  p_track = agent.custom[2]
  o = agent.custom[3]
  ctrl_d = agent.custom[4]

  u[1] = 0.1 * (v_track - x[2])
  u[2] = 1.0 * (p_track - x[3])
  if o == BRAKE
    u[1] = sign(x[2]) * ctrl_d.mi[1]
  elseif o == ACC
    u[1] = sign(x[2]) * ctrl_d.mx[1]
  elseif o == NOTHING
  end
end

end
