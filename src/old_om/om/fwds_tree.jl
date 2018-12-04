module fwds_tree
export StateNode

mutable struct StateNode
  s::Array{Float64,1}
  v::Float64
  parent::Union{StateNode,Nothing}
  children::Union{Vector{Tuple{StateNode,Int}},Nothing}

  StateNode(state,parent) = new(state,0.0,parent,nothing)
end

end
