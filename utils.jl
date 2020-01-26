function getAcclerationValues(a::Symbol)
    acc_lon = 0.0
    acc_lat = 0.0
    if a == :faster_left
        acc_lon = action_matrix[1, 1][1]
        acc_lat = action_matrix[1, 1][2]
    elseif a == :faster
        acc_lon = action_matrix[1, 2][1]
        acc_lat = action_matrix[1, 2][2]
    elseif a == :faster_right
        acc_lon = action_matrix[1, 3][1]
        acc_lat = action_matrix[1, 3][2]
    elseif a == :neutral_left
        acc_lon = action_matrix[2, 1][1]
        acc_lat = action_matrix[2, 1][2]
    elseif a == :neutral
        acc_lon = action_matrix[2, 2][1]
        acc_lat = action_matrix[2, 2][2]
    elseif a == :neutral_right
        acc_lon = action_matrix[2, 3][1]
        acc_lat = action_matrix[2, 3][2]
    elseif a == :slower_left
        acc_lon = action_matrix[3, 1][1]
        acc_lat = action_matrix[3, 1][2]
    elseif a == :slower
        acc_lon = action_matrix[3, 2][1]
        acc_lat = action_matrix[3, 2][2]
    elseif a == :slower_right
        acc_lon = action_matrix[3, 3][1]
        acc_lat = action_matrix[3, 3][2]
    else
        error("invalid action: $a")
    end
    return (acc_lon, acc_lat)
end


# https://github.com/JuliaPOMDP/POMDPExamples.jl/blob/master/notebooks/Defining-a-POMDP-with-the-Generative-Interface.ipynb 
# Initial state distribution

# The final task for describing the POMDP is specifying the initial state distribution, which can be accomplished by implementing a method of initialstate_distribution.

# For this example, the baby always starts in the the "not hungry" state (false). This is implemented using the Deterministic distribution from POMDPModelTools. The distribution returned by initialstate_distribution can be any object that implements all or part of the POMDPs.jl distribution interface.

# Alternatively, one may implement initialstate(m::BabyPOMDP, rng::AbstractRNG), to generate initial states, but many solvers expect at least a sampleable model of the initial state distribution, so initialstate_distribution is preferred.
# In [4]:

# POMDPs.initialstate_distribution(m::BabyPOMDP) = Deterministic(false)

# function POMDPs.initialstate(m::myPOMDP, rng::AbstractRNG) 
#     example_ego_state = SVector(280.34234, 1.1, 13.2)
#     example_obj_states = @SMatrix [ 20. -0.5 1.5 1.;
#                                 15. 0.5 12. 2.;
#                                 11. -0.6 11. 1. ;
#                                 10. 0.3 10. 0.5;
#                                 0. 0. 0. 0.;
#                                 0. 0. 0. 0.;
#                                 0. 0. 0. 0.;
#                                 0. 0. 0. 0.;
#                                 0. 0. 0. 0.]
#     example_state = State(example_ego_state, example_obj_states)

#     # return Deterministic(example_state)
#     return example_state
#     # return Deterministic(false)
# end
