#!/usr/bin/env julia
module ika_decision_making
include("utils.jl")

using Conda
using RobotOS # ROS stuff for julia

# import ros messages
@rosimport geometry_msgs.msg:Point, Pose2D
@rosimport definitions.msg:IkaEgoState,
IkaObject,
IkaObjectList,
IkaRouteStatus,
IkaLaneletRoute,
IkaActionSequence,
IkaObjectListPrediction,
IkaObjectPrediction
rostypegen(@__MODULE__)
import .geometry_msgs.msg: Point, Pose2D
import .definitions.msg:
    IkaEgoState,
    IkaObject,
    IkaObjectList,
    IkaRouteStatus,
    IkaLaneletRoute,
    IkaActionSequence,
    IkaObjectListPrediction,
    IkaObjectPrediction

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

## Lanelet2 stuff
# using Cxx
# cxx""" #include "/home/cremer/git/automated-driving-platform/ros/src/perception/processing/route_planning/lanelet2/lanelet2_core/include/lanelet2_core/primitives/Lanelet.h" """
# Does not work because of include eigen, boost, etc. in header of lanelet source files... so calculating frenet-coordinates manually?
# route is available as vector of 3D-points(geometry_msgs.msg:Point)

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
# Action space
#  3x3 lat/lon acc + 1 neutral = 10 available actions
#     1           2        3
#     4           5        6
#     7           8        9
# 
# +1.5|-0.5    1.5|0    1.5|+0.5
#    0|-0.5      0|0      0|+0.5
# -1.5|-0.5   -1.5|0   -1.5|+0.5

action_matrix = @SMatrix [
    (1.5, -0.5) (1.5, 0.0) (1.5, 0.0)
    (0.0, -0.5) (0.0, 0.0) (0.0, 0.5)
    (-1.5, -0.5) (-1.5, 0.0) (-1.5, 0.5)
]

# ? const action_sequence = SVector{40,Action} # 40 actions @ 4Hz = 10 seconds planning horizon

struct myPOMDP <: POMDP{State,Symbol,Observation} end

POMDPs.actions(pomdp::myPOMDP) = [
    :faster_left,
    :faster,
    :faster_right,
    :neutral_left,
    :neutral,
    :neutral_right,
    :slower_left,
    :slower,
    :slower_right,
]

function POMDPs.actionindex(pomdp::myPOMDP, a::Symbol)
    if a == :faster_left
        return 1
    elseif a == :faster
        return 2
    elseif a == :faster_right
        return 3
    elseif a == :neutral_left
        return 4
    elseif a == :neutral
        return 5
    elseif a == :neutral_right
        return 6
    elseif a == :slower_left
        return 7
    elseif a == :slower
        return 8
    elseif a == :slower_right
        return 9
    end
    error("invalid action: $a")
end

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
    s_r = s.ego[1] + 0.25 * s.ego[3] + 0.5 * 0.25 * 0.25 * acc_lon # distance along route
    d_r = s.ego[2] + 0.5 * 0.25 * 0.25 * acc_lat                         # distance perpendicular to route
    v_r = s.ego[3] + 0.25 * sqrt(acc_lon * acc_lon + acc_lat * acc_lat)  # velocity on route

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
            new_state_objs[i, 1] = s.objs[i, 1] + 0.25 * s.objs[i, 3] + 0.5 * 0.25 * 0.25 * a_k
            new_state_objs[i, 2] = s.objs[i, 2]
            new_state_objs[i, 3] = s.objs[i, 3] + 0.25 * a_k

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
    # TODO: get current v_ref of lanelet, from route_status_.v_max 
    # v_ref = 50.0 / 3.6
    v_ref = route_status_.v_max

    # if v is not v_ref
    r -= abs(v_ref - new_state_ego[3])

    # if we go in direction of destination
    r += new_state_ego[1] - s.ego[1]

    # if we are not nn the middle line 
    r -= abs(new_state_ego[2]) * 0.2

    return (sp = sp, o = o, r = r)
end

function POMDPs.initialstate_distribution(m::myPOMDP)
    example_ego_state = SVector(0.0, 1.0, 0.5)
    example_obj_states = @MMatrix zeros(10, 4)
    example_state = State(example_ego_state, example_obj_states)
    return Deterministic(example_state)
end

function POMDPModelTools.obs_weight(m::myPOMDP, sp::State, o::Observation)
    return 0.9
end


function POMDPs.isterminal(m::myPOMDP, s::State)
    return false # TODO implement
end

POMDPs.discount(pomdp::myPOMDP) = 0.95 # necessary?
my_pomdp = myPOMDP()


## construct and config the pomdp solver
# DESPOT
# D = tree height/planning horizon
# T_max = max time for one planning step
solver = DESPOTSolver(bounds = (-20.0, 20.0), T_max = 1.0, default_action = :neutral, D = 40) # can work with generative interface

## POMCPOW
# solver = POMCPOWSolver(max_depth = 40, max_time = 1.0) # needs explicit interface

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
route_status_ = 0
ego_state_ = 0
object_list_ = 0
predicted_object_list_ = 0
b_observation_received = false


function callbackEgoState(msg::IkaEgoState)
    global ego_state_ = msg
end

function callbackObjectList(msg::IkaObjectList)
    global object_list_ = msg
end


function callbackRouteStatus(msg::IkaRouteStatus)
    global route_status_ = msg
end

function callbackObjectListPrediction(msg::IkaObjectListPrediction)
    global predicted_object_list_ = msg
end

function convertMeasurements2ObservationSpace()
    global b_observation_received = false
    #  object observation
    if object_list_ != 0
        obs_objs = @MMatrix zeros(size(object_list_.objects, 1), 3)
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

    # ego observation
    if route_status_ != 0 && ego_state_ != 0
        println(string("route_status_.s:", route_status_.s))
        obs_ego = EgoObserv(route_status_.s, route_status_.d, ego_state_.fVelocity)
        global b_observation_received = true
        return Observation(obs_ego, obs_objs)
    end
end

dist = initialstate_distribution(my_pomdp)
belief = initialize_belief(belief_updater, dist)
belief_old = belief


function loop(pub_action)

    loop_rate = Rate(1.0)

    while !is_shutdown()
        observation = convertMeasurements2ObservationSpace()
        # How to convert observation to belief? belief_updater?
        # How to get first belief? initialstate_distribution?
        println(string("b_observation_received? ", b_observation_received))
        if b_observation_received
            ## TODO: Belief updater
            ## TODO: update belief from Observation
            # Something like this?
            a = POMDPs.action(planner, belief)
            global belief = POMDPs.update(belief_updater, belief_old, a, observation)

            # a_sequence = show_tree_and_best_sequence(planner, belief)
            # global belief = POMDPs.update(belief_updater, belief_old, a_sequence[0], observation)

            global belief_old = belief
            println(string("action", a))

            ## TODO: get Action sequence instead of one action

            ## Dummy action sequence
            a_seq = IkaActionSequence()
            a_seq.header.stamp = RobotOS.now()
            a_seq.header.frame_id = "map"
            a_seq.actions = zeros(Int, 40)

            vel = ego_state_.fVelocity * 1.0

            for i in 1:40
                println(vel)
                if vel < 10.0
                    a_seq.actions[i] = 2
                    vel = vel + 0.25 * action_matrix[1, 2][1]
                end

                if vel > 10.0
                    a_seq.actions[i] = 8
                    vel = vel + 0.25 * action_matrix[3, 2][1]
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

    sub_ego_state =
        Subscriber{IkaEgoState}("/localization/EgoState", callbackEgoState, queue_size = 20)

    sub_object_list = Subscriber{IkaObjectList}(
        "/fusion/ikaObjectList",
        callbackObjectList,
        queue_size = 20,
    )

    sub_predicted_object_list = Subscriber{IkaObjectListPrediction}(
        "/ikaObjectListPrediction",
        callbackObjectListPrediction,
        queue_size = 20,
    )

    sub_route_status = Subscriber{IkaRouteStatus}(
        "/route_planning/routeStatus",
        callbackRouteStatus,
        queue_size = 20,
    )

    loop(pub_action)
end

if !isinteractive()
    main()
end

end
