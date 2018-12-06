module pomdp

@enum CollisionType HITTING=1 BEING_HIT=2
@enum DriverType WEAK=1 MEDIUM=2 STRONG=3
@enum DriverAction BRAKE=1 ACC=2 NOTHING=3

const ACTIONS = [BRAKE, ACC, NOTHING]
const DRIVERS = [WEAK, MEDIUM, STRONG]

struct Collision
  d::Float64
  t::Float64
  ctype::Enum
end

const max_t = 5.0
const max_d = 10.0
function reward(x, u, nx, agent, world)
  d = nx[4]
  t = nx[5]

  base_r = abs(x[3]) > 0.35 * world.road.width ? -1e9 : x[2]^3
  coll_r = 0.0
  if d < max_d && t >= 0.0 && t <= max_t
    t = t < 1.0 ? 1.0 : t
    coll_r += -2.5e3 * (d / 10.0) * (max_t - t)
  end

  return base_r + coll_r
end

function P_adv(o::Enum, c::Collision, s::Enum)
  if c.d <= max_d && c.t >= 0.0 && c.t <= max_t
    P = 0.0
    if s == WEAK
      P = o == BRAKE ? 0.9 : 0.0
      P = o == ACC ? 0.05 : 0.0
      P = o == NOTHING ? 0.05 : 0.0
    elseif s == MEDIUM
      if c.ctype == HITTING
        P = o == BRAKE ? 0.9 : 0.0
        P = o == ACC ? 0.05 : 0.0
        P = o == NOTHING ? 0.05 : 0.0
      elseif c.ctype == BEING_HIT
        P = o == BRAKE ? 0.05 : 0.0
        P = o == ACC ? 0.9 : 0.0
        P = o == NOTHING ? 0.05 : 0.0
      end
    elseif s == STRONG
      P = o == BRAKE ? 0.05 : 0.0
      P = o == ACC ? 0.05 : 0.0
      P = o == NOTHING ? 0.9 : 0.0
    end
  else
    if o == BRAKE || o == ACC
      return 0.0
    elseif o == NOTHING
      return 1.0
    end
  end
end

function update_belief(b::AbstractArray{Float64, 1}, o::Enum,
                       c::Collision, s::Enum)
  bp = similar(b)
  for i in 1:length(b)
    bp[i] = b[i] * P_adv(o, c, s)
  end
  bp ./= sum(bp)

  return bp
end

function sample_adv_a(dt::Enum, c::Collision)
  if c.d <= max_d && c.t >= 0.0 && c.t <= max_t
    # set cP
    cP = Float64[]
    if s == WEAK
      cP = [0.9, 0.95, 1.0]
    elseif s == MEDIUM
      if c.ctype == HITTING
        cP = [0.9, 0.95, 1.0]
      elseif c.ctype == BEING_HIT
        cP = [0.05, 0.95, 1.0]
      end
    elseif s == STRONG
      cP = [0.05, 0.10, 1.0]
    end

    # sample
    r = rand()
    for i in 1:length(ACTIONS)
      if r >= cP[i]
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
  v_track = agent.custom[1]
  p_track = agent.custom[2]
  o = agent.custom[3]
  ctrl_d = agent.custom[4]

  u[1] = 0.1 * (v_track - x[2])
  u[2] = 1.0 * (p_track - x[3])
  if o == BRAKE
    u[2] = sign(x[2]) * ctrl_d.mi[1]
  elseif o == ACC
    u[2] = sign(x[2]) * ctrl_d.mx[1]
  elseif o == NOTHING
  end
end

end
