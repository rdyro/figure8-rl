module mdp
using DelimitedFiles 
export State, DetState, Policy, RandomPolicy

# Type definitions ############################################################
struct State
  a::Array{Int, 1}
  a2r::Array{Float64, 1}
  Pa2ns::Array{Float64, 2}
  cPa2ns::Array{Float64, 2}
  ns::Array{Int, 1}
end

struct DetState
  a::Array{Int, 1}
  a2r::Array{Float64, 1}
  ns::Array{Int, 1}
end

mutable struct Policy
  S::Union{Dict{Int, State}, Dict{Int, DetState}}
  U::Dict{Int, Float64}
  Aidx::Dict{Int, Int}
  drate::Float64
end

function Policy(S::Union{Dict{Int, State}, Dict{Int, DetState}}, 
                drate::Float64=1.0)
  U = Dict{Int, Float64}()
  sizehint!(U, length(S))

  Aidx = Dict{Int, Int}()
  sizehint!(Aidx, length(S))

  for (id, s) in S
    (U[id], Aidx[id]) = findmax(s.a2r)
  end

  return Policy(S, U, Aidx, drate)
end

function RandomPolicy(S::Union{Dict{Int, State}, Dict{Int, DetState}}, 
                      drate::Float64=1.0)
  U = Dict{Int, Float64}()
  sizehint!(U, length(S))

  Aidx = Dict{Int, Int}()
  sizehint!(Aidx, length(S))

  for (id, s) in S
    Aidx[id] = rand(1:length(s.a))
    U[id] = s.a2r[Aidx[id]]
  end

  return Policy(S, U, Aidx, drate)
end
###############################################################################


# Utility Functions ###########################################################
function optimal_policy(S::Dict{Int, Union{State, DetState}}, drate::Float64)
  P = Policy(S, drate)

  ret = 1.0
  k = 40
  while ret > 0.0 || k > 0
    ret = iterate!(P)
    println(ret)
    k = ret == 0.0 ? k - 1 : k
  end

  return P
end

function read_data(name::AbstractString)::Array{Int, 2}
  # read the data and variable names in
  full_data = readdlm("data/" * name * ".csv", ',')
  D = Int.(full_data[2:end, :])

  return D
end

function write_data(name::AbstractString, D::AbstractArray{Int, 2})
  # read the data and variable names in
  full_data = [["s" "a" "r" "ns"]; D]
  writedlm("data/" * name * ".csv", full_data, ',')
end

function MLE(D::AbstractArray{Int, 2})
  # count the number of states and actions, next states for each state -------#
  us = unique(D[:, 1])
  s2ns = Dict{Int, Set{Int}}()
  s2a = Dict{Int, Set{Int}}()
  for s in us
    s2ns[s] = Set{Int}()
    s2a[s] = Set{Int}()
  end

  for i in 1:size(D, 1)
    sid = D[i, 1]
    a = D[i, 2]
    r = D[i, 3]
    ns = D[i, 4]
    push!(s2ns[sid], ns)
    push!(s2a[sid], a)
  end
  #---------------------------------------------------------------------------#

  # maximum likelihood estimate of state transistions and reward --------------#
  counts = Dict{Int, Array{Int, 2}}()
  rewards_total = Dict{Int, Array{Float64, 1}}()
  rewards_count = Dict{Int, Array{Int, 1}}()
  A = Dict{Int, Array{Int, 1}}()
  NS = Dict{Int, Array{Int, 1}}()
  for id in keys(s2ns)
    A[id] = sort(collect(s2a[id]))
    NS[id] = sort(collect(s2ns[id]))

    counts[id] = fill(0, (length(s2a[id]), length(s2ns[id])))
    rewards_total[id] = fill(0.0, length(s2a[id]))
    rewards_count[id] = fill(0, length(s2a[id]))
  end

  for i in 1:size(D, 1)
    sid = D[i, 1]
    a = D[i, 2]
    r = D[i, 3]
    ns = D[i, 4]

    idxa = 1
    for j in 1:length(A[sid])
      idxa = j
      A[sid][j] == a &&  break
    end

    idxns = 1
    for j in 1:length(NS[sid])
      idxns = j
      NS[sid][j] == ns && break
    end

    counts[sid][idxa, idxns] += 1

    rewards_total[sid][idxa] += r
    rewards_count[sid][idxa] += 1
  end
  #---------------------------------------------------------------------------#

  # allocate the model -------------------------------------------------------#
  chain = Dict{Int, State}()
  for id in keys(s2ns)
    a2r = rewards_total[id] ./ rewards_count[id]

    Pa2ns = convert.(Float64, counts[id])
    for i in 1:size(Pa2ns, 1)
      tot = sum(Pa2ns[i, :]) == 0.0 ? 1.0 : sum(Pa2ns[i, :])
      Pa2ns[i, :] /= tot
    end

    cPa2ns = cumsum(Pa2ns, dims=2)

    chain[id] = State(A[id], a2r, Pa2ns, cPa2ns, NS[id])
  end
  #---------------------------------------------------------------------------#

  return chain
end

function determinateMLE(D::AbstractArray{Int, 2})
  states = MLE(D)

  states_det = Dict{Int, DetState}()
  for (id, s) in states
    a = copy(s.a)
    idx = [cart_idx[2] for cart_idx in findmax(s.Pa2ns, dims=2)[2]][:]
    ns = copy(s.ns[idx])

    states_det[id] = DetState(a, s.a2r, ns)
  end

  return states_det
end

function iterate!(p::Policy)
  nbchange = 0
  change = 0.0

  det = typeof(p.S) == Dict{Int, DetState}

  SID = collect(keys(p.S))
  EUr = fill(0.0, 0)
  for k in 1:length(SID)
    sid = SID[k]
    s = p.S[sid]

    if length(EUr) < length(s.a2r)
      EUr = fill(0.0, length(s.a2r))
    end
    for i in 1:length(EUr)
      EUr[i] = 0.0
    end

    for i in 1:length(s.a2r)
      EUr[i] = 0.0
      if det
        if s.ns[i] > 0 # check if action is not final
          EUr[i] = p.drate * p.U[s.ns[i]]
        end
      else
        for j in 1:length(s.ns)
          if s.ns[j] > 0 # check if action is not final
            EUr[i] += p.drate * s.Pa2ns[i, j] * p.U[s.ns[j]]
          else
            println("Action is final")
          end
        end
      end
    end

    oldaidx = p.Aidx[sid]
    oldu = p.U[sid]

    maxAidx = 1
    maxU = -Inf
    for i in 1:length(s.a2r)
      if s.a2r[i] + EUr[i] >= maxU
        maxU = s.a2r[i] + EUr[i]
        maxAidx = i
      end
    end
    #(p.U[sid], p.Aidx[sid]) = findmax(s.a2r + EUr)
    p.U[sid] = maxU
    p.Aidx[sid] = maxAidx


    change += abs(oldu - p.U[sid])
    if oldaidx != p.Aidx[sid]
      nbchange += 1
    end
  end

  return (change, nbchange)
end

function estimate_score(p::Policy, N::Int=10)
  det = typeof(p.S) == Dict{Int, DetState}

  score_tot = 0.0
  for i in 1:N
    score_part = 0.0
    sid = rand(keys(p.S))

    M = p.drate != 1.0 ? ceil(log(eps()) / log(p.drate)) : (1 << 31)
    #M = 100
    for j in 1:M
      aidx = p.Aidx[sid]
      score_part += p.drate^(j - 1) * p.S[sid].a2r[aidx]
      nsid = 1


      if det
        nsid = p.S[sid].ns[aidx]
      else
        r = rand()
        for k in 1:length(p.S[sid].ns)
          if p.S[sid].cPa2ns[aidx, k] > r
            nsid = p.S[sid].ns[k]
            break
          end
        end
      end

      if nsid < 0 # check if action was final
        #println("Stopping after $(j) actions")
        break
      else
        sid = nsid
      end
    end
    #println("####################################################")

    score_tot += score_part
  end

  return score_tot / N
end

function generate_data(name::AbstractString, S::Dict{Int, State}, N::Int=10^5)
  fp = open("data/" * name * ".csv", "w")
  write(fp, "s,a,r,ns\n")

  k = keys(S)
  for i in 1:N
    s = rand(k)
    aidx = rand(1:length(S[s].a))
    a = S[s].a[aidx]
    r = round(Int, S[s].a2r[aidx])
    ns = -1

    rnd = rand()
    for j in 1:length(S[s].ns)
      if S[s].cPa2ns[aidx, j] > rnd
        ns = S[s].ns[j]
        break
      end
    end

    write(fp, "$(s),$(a),$(r),$(ns)\n")
  end

  close(fp)
end

function write_policy(name::AbstractString, p::Policy)
  fp = open("data/" * name * ".policy", "w")

  for s in 1:maximum(keys(p.S))
    if haskey(p.S, s) && haskey(p.Aidx, s)
      write(fp, string(p.S[s].a[p.Aidx[s]]))
    else
      write(fp, string(1))
    end
    write(fp, "\n")
  end

  close(fp)
end
###############################################################################

end
