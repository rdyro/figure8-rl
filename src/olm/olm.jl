module olm
dir = @__DIR__

push!(LOAD_PATH, dir * "../sim")
push!(LOAD_PATH, dir * "../dis")

using sim
using dis

const olm_dt = 1.0 # time increment for planning step
const olm_h = 1e-1 # time increment for numerical integration
const olm_gamma = 0.9999 # discount factor

mutable struct Tree
  value::Any
  next::AbstractArray{Tree, 1}

  Tree() = new(nothing, Tree[])
  Tree(value) = new(value, Tree[])
end

mutable struct GroupTree
  value::Any
  next::AbstractArray{AbstractArray{GroupTree, 1}, 1}

  GroupTree() = new(nothing, Array{GroupTree, 1}[])
  GroupTree(value) = new(value, Array{GroupTree, 1}[])
end

include(dir * "/olm_mcts.jl")
include(dir * "/olm_fwds.jl")

end
