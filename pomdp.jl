#!/usr/bin/env julia
module ika_decision_making

using POMDPs
using POMDPModelTools
using ParticleFilters
using ARDESPOT
# using POMCPOW
# using POMDPPolicies

using StaticArrays
using Distributions
using Random
using LinearAlgebra
# using Parameters
# using Plot

include("msg_import.jl")

## Lanelet2 stuff
# using Cxx
# cxx""" #include "/home/cremer/git/automated-driving-platform/ros/src/perception/processing/route_planning/lanelet2/lanelet2_core/include/lanelet2_core/primitives/Lanelet.h" """
# Does not work because of include eigen, boost, etc. in header of lanelet source files... so calculating frenet-coordinates manually?
# route is available as vector of 3D-points(geometry_msgs.msg:Point)
time_step_ = 0.25        # seconds - default
planning_horizon_ = 40   # steps - default
##############
# State space
const EgoState = SVector{3,Float64} # s, d, v
const ObjStates = SMatrix{(10, 4),Float64}  # s, d, v, r  | states for 10 nearest objects

struct State{EgoState,ObjStates}
    ego::EgoState
    objs::ObjStates
end

##############
## Observation space
const EgoObserv = SVector{3,Float64} # s, d, v
const ObjObservs = SMatrix{(10, 3),Float64} # x, y, v  | observations for 10 nearest objects

struct Observation{EgoObserv,ObjObservs}
    ego::EgoObserv
    objs::ObjObservs
end


##############
struct myPOMDP <: POMDP{State,Symbol,Observation} end
include("actions.jl")
##############

# Generative Interface
# example:
# function POMDPs.gen(m::MyPOMDP, s, a, rng)
#   sp = s+a
#   r = s^2
#   o = s+a+randn(rng)
#   return (sp, r, o)
# end

function POMDPs.gen(m::myPOMDP, s::State, a::Symbol, rng)
    # suppose we plan with 0.25s steps
    acc_lon = 0.0
    acc_lat = 0.0
    acc_lon, acc_lat = getAcclerationValues(a)

    ### TRANSITION MODEL
    ## next State sp
    # ego state
    # TODO: this is not accurate because v_r can point away from route, but for ok now
    s_r = s.ego[1] + time_step_ * s.ego[3] + 0.5 * time_step_ * time_step_ * acc_lon # distance along route
    d_r = s.ego[2] + 0.5 * time_step_ * time_step_ * acc_lat                         # distance perpendicular to route
    v_r = s.ego[3] + time_step_ * sqrt(acc_lon * acc_lon + acc_lat * acc_lat)  # velocity on route

    new_state_ego = SVector(s_r, d_r, v_r) # s, d, v

    # object states
    new_state_objs = @MMatrix zeros(size(s.objs, 1), 4) # s, d, v, r  | states for 10 nearest objects
    for i in 1:size(s.objs, 1)
        if (s.objs[i, 1] != 0.0 &&
            s.objs[i, 2] != 0.0 && 
            s.objs[i, 3] != 0.0 && 
            s.objs[i, 4] != 0.0)
            # TODO: get conflict-zone-interaction based acceleration a_k 
            a_k = 0.0
            new_state_objs[i, 1] = s.objs[i, 1] + time_step_ * s.objs[i, 3] + 0.5 * time_step_ * time_step_ * a_k
            new_state_objs[i, 2] = s.objs[i, 2]
            new_state_objs[i, 3] = s.objs[i, 3] + time_step_ * a_k

            # TODO: Somehow calculate route probability via features (position error, v-vref error)
            new_state_objs[i, 4] = s.objs[i, 4]
        end
    end

    sp = State(new_state_ego, new_state_objs)


    ### OBSERVATION MODEL
    ## ego observation = ego state for now
    obser_ego = SVector(new_state_ego[1], new_state_ego[2], new_state_ego[3]) # s, d, v # no noise yet

    # objects observation
    # TODO: Somehow generate x,y from route id and s and d 
    # -> Probably Lanelet2-functions needed, fromArccordinates(route, ...)
    # 0 for now
    obser_objs = @MMatrix zeros(size(sp.objs, 1), 3)
    for i in 1:size(sp.objs, 1)
        # obser_objs[i, 1] = obj.fPosX 
        # obser_objs[i, 2] = obj.fPosY
        # obser_objs[i, 3] = sqrt(obj.fAbsVelX * obj.fAbsVelX + obj.fAbsVelY * obj.fAbsVelY)
    end

    o = Observation(obser_ego, obser_objs)

    # reward model
    r = 0.0
    # TODO: check Collision! --> negative reward if collision or very near object
    # TODO: v_max is only from currentLanelet, also needs v_max from future Lanelets
    v_ref = route_status_.v_max / 3.6

    # if v is not v_ref
    r -= abs(v_ref - new_state_ego[3]) * 0.5

    # if we go in direction of destination
    r += new_state_ego[1] - s.ego[1]

    # if we are not in the middle line 
    r -= abs(new_state_ego[2]) * 0.2
    
    # for every accleration : punish a little
    r -= abs(acc_lon) * 0.02 + abs(acc_lat) * 0.02 

    # destination reached!
    if (route_status_.s_dest - new_state_ego[1]) < 0.5 && d_r < 0.5 && new_state_ego[3] < 0.5
        r += 10.0
    end

    return (sp = sp, o = o, r = r)
end

function POMDPs.initialstate_distribution(m::myPOMDP)
    example_ego_state = SVector(0.0, 0.0, 0.0)
    example_obj_states = @MMatrix zeros(10, 4)
    example_state = State(example_ego_state, example_obj_states)
    return Deterministic(example_state)
end


# https://github.com/JuliaPOMDP/POMDPModelTools.jl/blob/master/src/obs_weight.jl
# function POMDPModelTools.obs_weight(m::myPOMDP, sp::State, o::Observation)
# function POMDPModelTools.obs_weight(m::myPOMDP, a::Action, sp::State, o::Observation)
# TODO: implement this! 
# See Page 6:https://www.researchgate.net/publication/322279328_Automated_Driving_in_Uncertain_Environments_Planning_With_Interaction_and_Uncertain_Maneuver_Prediction
function POMDPModelTools.obs_weight(m::myPOMDP, s::State, a::Symbol, sp::State, o::Observation)
    return 0.9
end

function POMDPs.isterminal(m::myPOMDP, s::State)
    if route_status_.s_dest - s.ego[1] < 0.5
        return true
    else
        return false # TODO implement
    end
end

POMDPs.discount(pomdp::myPOMDP) = 0.95 # necessary?
my_pomdp = myPOMDP()

## construct and config the pomdp solver
# DESPOT
# D = tree height/planning horizon
# T_max = max time for one planning step
solver = DESPOTSolver(bounds = (-20.0, 20.0), T_max = 1.0, default_action = :neutral, D = planning_horizon_) # can work with generative interface

## POMCPOW
# solver = POMCPOWSolver(max_depth = planning_horizon_, max_time = 1.0) # needs explicit interface - TODO

sampletype(::Type{State}) = State

planner = solve(solver, my_pomdp) # construct the pomdp planner

##############
# Belief
belief_updater = updater(planner)
# N = 1000 # 1000 particles
# belief_updater = SIRParticleFilter(my_pomdp, N)

# const NormalEgoStateDist = SMatrix{3, 2} # s mean, s std, d mean, d std, v mean, v std
# const NormalObjStatesDist = SArray{Tuple{10, 4, 2},Float64} #s mean, s std, d mean, d std, v mean, v std, r mean, r std

# struct NormalStateDist
#     ego::NormalEgoStateDist
#     objs::NormalObjStatesDist
# end

# # example belief:
# ego_b0 = NormalEgoStateDist(SMatrix{3,2}([1. 3. ; 2. 4.; 0.5 0.2]))
# objs_b0 = @MArray zeros(10, 4, 2)

# b0 = NormalStateDist(ego_b0, objs_b0)

# example_ego_state = SVector(280.34234, 1.1, 13.2)
# example_obj_states = @SMatrix [
#     20.0 -0.5 1.5 1.0
#     15.0 0.5 12.0 2.0
#     11.0 -0.6 11.0 1.0
#     10.0 0.3 10.0 0.5
#     0.0 0.0 0.0 0.0
#     0.0 0.0 0.0 0.0
#     0.0 0.0 0.0 0.0
#     0.0 0.0 0.0 0.0
#     0.0 0.0 0.0 0.0
# ]
# rand(rng::AbstractRNG, d::NormalStateDist) = State(example_ego_state, example_obj_states)

# https://github.com/JuliaPOMDP/POMDPModels.jl/blob/master/src/LightDark.jl
# sampletype(::Type{LDNormalStateDist}) = LightDark1DState
# rand(rng::AbstractRNG, d::LDNormalStateDist) = LightDark1DState(0, d.mean + randn(rng)*d.std)
# initialstate_distribution(pomdp::LightDark1D) = LDNormalStateDist(2, 3)
# observation(p::LightDark1D, sp::LightDark1DState) = Normal(sp.y, p.sigma(sp.y))
################################################

# global variabels
route_status_ = IkaRouteStatus()
ego_state_ = IkaEgoState()
object_list_ = IkaObjectList()
predicted_object_list_ = IkaObjectListPrediction()
# b_obs_received = [false, false, false, false]
b_obs_received = [false, false, false]

function callbackEgoState(msg::IkaEgoState)
    global b_obs_received[1] = true
    global ego_state_ = msg
end

function callbackRouteStatus(msg::IkaRouteStatus)
    global b_obs_received[2] = true
    global route_status_ = msg
    # println(string("route_status_.v_max:", route_status_.v_max))
end

function callbackObjectList(msg::IkaObjectList)
    global b_obs_received[3] = true
    global object_list_ = msg
end

function callbackObjectListPrediction(msg::IkaObjectListPrediction)
    # global b_obs_received[4] = true
    global predicted_object_list_ = msg
end

function convertMeasurements2ObservationSpace()
    #  object observation
    obs_objs = @MMatrix zeros(size(object_list_.objects, 1), 3)
    if b_obs_received[3]
        obj = get(object_list_.objects, 1, 0)
        if obj != 0
            # valid objects
            for i in 1:size(object_list_.objects, 1)
                obj = get(object_list_.objects, i, 0)
                obs_objs[i, 1] = obj.fPosX
                obs_objs[i, 2] = obj.fPosY
                obs_objs[i, 3] = sqrt(obj.fAbsVelX * obj.fAbsVelX + obj.fAbsVelY * obj.fAbsVelY)
            end
        end
    end

    println(string("route_status_.s:", route_status_.s))
    
    # ego observation
    if b_obs_received[1] && b_obs_received[2] 
        obs_ego = EgoObserv(route_status_.s, route_status_.d, ego_state_.fVelocity)
        return Observation(obs_ego, obs_objs)
    else
        return 0
    end
end

dist = initialstate_distribution(my_pomdp)
belief = initialize_belief(belief_updater, dist)
belief_old = belief


function loop(pub_action)

    loop_rate = Rate(0.5) # min 1 Hz

    println("\n************ Decision Making ready ! \n")

    while !is_shutdown()
        observation = convertMeasurements2ObservationSpace()
        # How to convert observation to belief? belief_updater?
        println(string("b_obs_received? ego state, route status, object list: ", b_obs_received))
        if all(b_obs_received)
            ## TODO: Belief updater
            ## TODO: update belief from Observation
            # Something like this?
            a = POMDPs.action(planner, belief)
            global belief = POMDPs.update(belief_updater, belief_old, a, observation)
            global belief_old = belief
            # get action sequence from planner DESPOTSolver?

            # a_sequence = show_tree_and_best_sequence(planner, belief)

            # global belief = POMDPs.update(belief_updater, belief_old, a_sequence[0], observation)
            println(string("action:", a))

            ## TODO: get Action sequence instead of one action

            ## Dummy action sequence
            a_seq = IkaActionSequence()
            a_seq.header.stamp = RobotOS.now()
            a_seq.header.frame_id = "map"
            a_seq.actions = ones(Int, planning_horizon_) * 5 # neutral

            vel = ego_state_.fVelocity * 1.0

            for i in 1:planning_horizon_
                # println(vel)
                if vel < route_status_.v_max / 3.6
                    a_seq.actions[i] = 2
                    vel = vel + time_step_ * action_matrix[1, 2][1]
                end

                if vel > route_status_.v_max / 3.6 * 1.1
                    a_seq.actions[i] = 8
                    vel = vel + time_step_ * action_matrix[3, 2][1]
                end
            end
            publish(pub_action, a_seq)
        end
        rossleep(loop_rate)
    end
end


function main()
    init_node("POMDP")

    pub_action =
        Publisher{IkaActionSequence}("/ika_decision_making/actions", queue_size = 20)

    sub_ego_state = Subscriber{IkaEgoState}("/localization/ego_state",
        callbackEgoState,
        queue_size = 20)

    sub_object_list = Subscriber{IkaObjectList}("/fusion/ikaObjectList",
        callbackObjectList,
        queue_size = 20,
    )

    sub_predicted_object_list = Subscriber{IkaObjectListPrediction}("/object_list_prediction/object_list_prediction",
        callbackObjectListPrediction,
        queue_size = 20,
    )

    sub_route_status = Subscriber{IkaRouteStatus}("/route_planning/routeStatus",
        callbackRouteStatus,
        queue_size = 20,
    )

    global time_step_ = get_param("/ika_decision_making/time_step")
    global planning_horizon_ = get_param("/ika_decision_making/planning_horizon")

    loop(pub_action)
end

if !isinteractive()
    main()
end

end
