@enum CollisionType HITTING=1 BEING_HIT=2
@enum DriverType WEAK=1 MEDIUM=2 STRONG=3
@enum DriverAction BRAKE=1 ACC=2 NOTHING=3

struct POMDP
  S::AbstractArray{Enum, 1}
  O::AbstractArray{Enum, 1}
  Uadv::AbstractArray{AbstractArray{Float64, 1}, 1}
end

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

function P_adv(o::Enum, s::Enum, collision::Collision)
  if collision.d <= max_d && collision.t >= 0.0 && collision.t <= max_t
    P = 0.0
    if s == WEAK
      P = o == BRAKE ? 0.9 : 0.0
      P = o == ACC ? 0.05 : 0.0
      P = o == NOTHING ? 0.05 : 0.0
    elseif s == MEDIUM
      if collision.ctype == HITTING
        P = o == BRAKE ? 0.9 : 0.0
        P = o == ACC ? 0.05 : 0.0
        P = o == NOTHING ? 0.05 : 0.0
      elseif collision.ctype == BEING_HIT
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
