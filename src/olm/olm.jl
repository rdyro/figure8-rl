module olm
dir = @__DIR__

push!(LOAD_PATH, dir * "../sim")
push!(LOAD_PATH, dir * "../dis")

using sim
using dis

const olm_dt = 1.0 # time increment for planning step
const olm_h = 1e-1 # time increment for numerical integration
const olm_mcts_c = 0.1 # exploration parameter for MCTS
const olm_gamma = 0.9999 # discount factor

mutable struct Tree
  value::Any
  next::AbstractArray{Tree, 1}

  Tree() = new(nothing, Tree[])
  Tree(value) = new(value, Tree[])
end

include(dir * "/olm_mcts.jl")
include(dir * "/olm_fwds.jl")

end
