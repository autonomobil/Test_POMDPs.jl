using POMDPs
using Distributions: Normal
using Random
import POMDPs: initialstate_distribution, actions, gen, discount, isterminal
Random.seed!(1);

mutable struct LightDark1D <: POMDPs.POMDP{Float64,Int,Int}
    discount_factor::Float64
    correct_r::Float64
    incorrect_r::Float64
    step_size::Int
    movement_cost::Float64
end
LightDark1D() = LightDark1D(0.9, 10, -10, 1, 0)
discount(p::LightDark1D) = p.discount_factor
isterminal(::LightDark1D, s::Float64) = isnan(s);

noise(x) = ceil(Int, abs(x - 5) / sqrt(2) + 1e-2)

function gen(m::LightDark1D, s::Float64, a::Int, rng::AbstractRNG)
    # generate next state
    sp = iszero(a) ? NaN : s + a
    # generate observation
    if isnan(sp)
        o = 0
    else
        n = noise(sp)
        o = round(Int, sp) + rand(rng, -n:n)
    end
    # generate reward
    r = iszero(a) ? (abs(s) < 1 ? m.correct_r : m.incorrect_r) : 0.0
    
    return (sp = sp, o = o, r = r)
end;

actions(::LightDark1D) = [-1, 0, 1] # Left Stop Right

initialstate_distribution(pomdp::LightDark1D) = Normal(2.0, 3.0);

using BasicPOMCP
using POMDPSimulators

solver = POMCPSolver(tree_queries = 10000)
pomdp = LightDark1D()
planner = solve(solver, pomdp);

b = initialstate_distribution(pomdp)

struct LDObsDist
    x::Int
    noise::Int
end

function POMDPs.pdf(d::LDObsDist, x::Int)
    if abs(x-d.x) <= d.noise
        return 1/(2*d.noise+1)
    else
        return 0.0
    end
end

function POMDPs.observation(p::LightDark1D, a::Int, sp::Float64)
    if isnan(sp)
        return LDObsDist(0, 0)
    else
        return LDObsDist(round(Int, sp), noise(sp))
    end
end

using ParticleFilters

filter = SIRParticleFilter(pomdp, 1000)
for (s,a,r,sp,o) in stepthrough(pomdp, planner, filter, "s,a,r,sp,o")
    @show (s,a,r,sp,o)
end


