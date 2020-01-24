#!/usr/bin/env julia
module ika_decision_making

ENV["PYTHON"] = "/usr/bin/python2.7"
using Conda
using RobotOS # ROS stuff for julia

# import ros messages
@rosimport geometry_msgs.msg:Point, Pose2D
@rosimport definitions.msg:IkaEgoState, IkaObject, IkaObjectList, IkaRouteStatus, IkaLaneletRoute, IkaActionSequence
rostypegen(@__MODULE__)
import .geometry_msgs.msg:Point, Pose2D
import .definitions.msg:IkaEgoState, IkaObject, IkaObjectList, IkaRouteStatus, IkaLaneletRoute, IkaActionSequence

using POMDPs
using POMDPModelTools
using ARDESPOT
# using POMCPOW
# using POMDPSimulators
# using POMDPPolicies
# using POMDPPolicies
# using POMDPSimulators
# using ParticleFilters

using StaticArrays
using Distributions
using Random
using LinearAlgebra
# using Parameters
# using Plot

## Lanelet2 stuff
# using Cxx
# cxx""" #include "/home/cremer/git/automated-driving-platform/ros/src/perception/processing/route_planning/lanelet2/lanelet2_core/include/lanelet2_core/primitives/Lanelet.h" """
# does not work because of include eigen, boost, etc. in header of lanelet source files

## State space
const ego_state = SVector{3,Float64} # s, d, v
const obj_states = SMatrix{(10, 4),Float64}  # s, d, v, r  | states for 10 nearest objects

struct State{ego_state,obj_states}
    ego::ego_state
    objs::obj_states
end

## Observation space
const ego_observ = SVector{3,Float64} # s, d, v
const objs_observ = SMatrix{(10, 3),Float64} # x, y, v  | observations for 10 nearest objects

struct Observation{ego_observ,objs_observ}
    ego::ego_observ
    objs::objs_observ
end

#  3x3 lat/lon acc + 1 neutral = 10 available actions
# 1 2 3
# 4 5 6
# 7 8 9
# +1.5|-0.5     1.5|0    1.5|+0.5
#    0|-0.5       0|0      0|+0.5
# -1.5|-0.5    -1.5|0   -1.5|+0.5

action_matrix = @SMatrix [  (1.5, -0.5)     (1.5, 0.)   (1.5, 0.);
                            (0., -0.5)      (0., 0.)    (0., 0.5);
                            (-1.5, -0.5)    (-1.5, 0.)  (-1.5, 0.5)]

# ? const action_sequence = SVector{40,Action} # 40 actions @ 4Hz = 10 seconds planning horizon

struct myPOMDP <: POMDP{State,Symbol,Observation} # POMDP{State, Action, Observation}
end

POMDPs.actions(pomdp::myPOMDP) = [:faster_left, :faster, :faster_right, :neutral_left, :neutral, :neutral_right, :slower_left, :slower, :slower_right]

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
end;

function getAcclerationValues(a::Symbol)
    acc_lon = 0.0
    acc_lat = 0.0
    if a == :faster_left
        acc_lon = action_matrix[1,1][1]
        acc_lat = action_matrix[1,1][2]
    elseif a == :faster
        acc_lon = action_matrix[1,2][1]
        acc_lat = action_matrix[1,2][2]
    elseif a == :faster_right
        acc_lon = action_matrix[1,3][1]
        acc_lat = action_matrix[1,3][2]
    elseif a == :neutral_left
        acc_lon = action_matrix[2,1][1]
        acc_lat = action_matrix[2,1][2]
    elseif a == :neutral
        acc_lon = action_matrix[2,2][1]
        acc_lat = action_matrix[2,2][2]
    elseif a == :neutral_right
        acc_lon = action_matrix[2,3][1]
        acc_lat = action_matrix[2,3][2]
    elseif a == :slower_left
        acc_lon = action_matrix[3,1][1]
        acc_lat = action_matrix[3,1][2]
    elseif a == :slower
        acc_lon = action_matrix[3,2][1]
        acc_lat = action_matrix[3,2][2]
    elseif a == :slower_right
        acc_lon = action_matrix[3,3][1]
        acc_lat = action_matrix[3,3][2]
    else
        error("invalid action: $a")
    end
    return (acc_lon, acc_lat)
end

function POMDPs.gen(m::myPOMDP, s::State, a::Symbol, rng)
    # suppose we plan with 0.25s steps
    acc_lon = 0.0
    acc_lat = 0.0
    acc_lon, acc_lat = getAcclerationValues(a)

    ### TRANSITION MODEL
    ## next State sp
    # ego state
    # TODO: this is not accurate because v_r can point away from route, but for ok now
   	s_r = s.ego_state[1] + 0.25 * s.ego_state[3] + 0.5 * 0.25 * 0.25 * acc_lon; # distance along route
   	d_r = s.ego_state[2] + 0.5 * 0.25 * 0.25 * acc_lat;                         # distance perpendicular to route
    v_r = s.ego_state[3] + 0.25 * sqrt(acc_lon * acc_lon + acc_lat * acc_lat);  # velocity on route
    
    state_ego = SVector(s_r, d_r, v_r) # s, d, v

    # object states
    state_objs = @MMatrix zeros(size(s.obj_states, 1), 4) # s, d, v, r  | states for 10 nearest objects
    for i = 1:size(s.obj_states, 1)
        if(s.obj_states[i].s != 0. 
            && s.obj_states[i].d != 0.
            && s.obj_states[i].v != 0. 
            && s.obj_states[i].r != 0.)

            # TODO: get conflict-zone-interaction based acceleration a_k 
            a_k = 0;
            state_objs[i].s = s.obj_states[i].s + 0.25 * s.obj_states[i].v + 0.5 * 0.25 * 0.25 * a_k
            state_objs[i].d = s.obj_states[i].d  
            state_objs[i].v = s.obj_states[i].v + 0.25 * a_k

            # TODO: Somehow calculate route probability via features (position error, v-vref error)
            state_objs[i].route_id = s.obj_states[i].route_id;
        end
    end

    sp = State(state_ego, state_objs)


    ### OBSERVATION MODEL
    ## ego observation = ego state for now
    obser_ego = SVector(state_ego[1], state_ego[2], state_ego[3]) # s, d, v # no noise yet

    # objects observation
    # TODO: Somehow generate x,y from route id and s and d 
    # -> Probably Lanelet2-functions needed, fromArccordinates(route, ...)
    obser_objs = @MMatrix zeros(size(sp.obj_states, 1), 3)
    for i = 1:size(sp.obj_states, 1) 
        # obser_objs[i, 1] = obj.fPosX 
        # obser_objs[i, 2] = obj.fPosY
        # obser_objs[i, 3] = sqrt(obj.fAbsVelX * obj.fAbsVelX + obj.fAbsVelY * obj.fAbsVelY)
    end

    o = Observation(obser_ego, obser_objs)
    
    # reward model
    r = 0.0
    # TODO: check Collision! --> negative reward if collision or very near object
    # TODO: get current v_ref of lanelet, from route_status_.v_max 
    v_ref = 50.0 / 3.6
    
    # if v is not v_ref
    r -= abs(v_ref - state_ego.v);

    # if we go in direction of destination
    r += state_ego.s - s.ego_state[1];

    # return (sp= #=new state=#, r= #=reward=#, o= #=observation=#)
    return (sp = sp, o = o, r = r)
end


# https://github.com/JuliaPOMDP/POMDPExamples.jl/blob/master/notebooks/Defining-a-POMDP-with-the-Generative-Interface.ipynb 
# Initial state distribution

# The final task for describing the POMDP is specifying the initial state distribution, which can be accomplished by implementing a method of initialstate_distribution.

# For this example, the baby always starts in the the "not hungry" state (false). This is implemented using the Deterministic distribution from POMDPModelTools. The distribution returned by initialstate_distribution can be any object that implements all or part of the POMDPs.jl distribution interface.

# Alternatively, one may implement initialstate(m::BabyPOMDP, rng::AbstractRNG), to generate initial states, but many solvers expect at least a sampleable model of the initial state distribution, so initialstate_distribution is preferred.
# In [4]:

# POMDPs.initialstate_distribution(m::BabyPOMDP) = Deterministic(false)

function POMDPs.initialstate_distribution(m::myPOMDP) 
    example_ego_state = SVector(280.34234, 1.1, 13.2)
    example_obj_states = @SMatrix [ 20. -0.5 1.5 1.;
                                15. 0.5 12. 2.;
                                11. -0.6 11. 1. ;
                                10. 0.3 10. 0.5;
                                0. 0. 0. 0.;
                                0. 0. 0. 0.;
                                0. 0. 0. 0.;
                                0. 0. 0. 0.;
                                0. 0. 0. 0.]
    example_state = State(example_ego_state, example_obj_states)
    return Deterministic(example_state)
end

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

function POMDPs.isterminal(m::myPOMDP, s::State)
    return false # TODO implement
end

POMDPs.discount(pomdp::myPOMDP) = 0.95 # necessary?
my_pomdp = myPOMDP()

## DESPOT
# construct and config the pomdp solver
# D = tree height/planning horizon
# T_max = max time for one planning step
solver = DESPOTSolver(bounds = (-20.0, 20.0), T_max = 1.0, default_action = :neutral, D = 40) # can work with generative interface

## POMCPOW
# solver = POMCPOWSolver(max_depth = 40, max_time = 1.0) # needs explicit interface

planner = solve(solver, my_pomdp) # construct the pomdp planner

belief_updater = updater(planner) 

################################################

route_status_ = IkaRouteStatus()
ego_state_ = IkaEgoState()
object_list_ = IkaObjectList()

function callbackEgoState(msg::IkaEgoState)
    global ego_state_ = msg
end

function callbackObjectList(msg::IkaObjectList)
    global object_list_ = msg
end


function callbackRouteStatus(msg::IkaRouteStatus)
    global route_status_ = msg
end

function convertMeasurements2ObservationSpace()
    #  object observation
    obs_objs = @MMatrix zeros(size(object_list_.objects, 1), 3)
    obj = get(object_list_.objects, 1, 0)
    if obj != 0
        # valid objects
        for i = 1:size(object_list_.objects, 1) 
            obj = get(object_list_.objects, i, 0)
            obs_objs[i, 1] = obj.fPosX 
            obs_objs[i, 2] = obj.fPosY
            obs_objs[i, 3] = sqrt(obj.fAbsVelX * obj.fAbsVelX + obj.fAbsVelY * obj.fAbsVelY)
        end
    end
    
    # ego observation
    if route_status_.s isa Float64 || route_status_.s isa Float32
        obs_ego = ego_observ(route_status_.s, route_status_.d, ego_state_.fVelocity)
        return Observation(obs_ego, obs_objs)
    else
        return 0
    end
end


function loop(pub_action)

    loop_rate = Rate(1.0)

    while ! is_shutdown()
        println("function loop(pub_action)")
        
        world_obs = convertMeasurements2ObservationSpace()
        # How to convert observation to belief? belief_updater?
        # How to get first belief? initialstate_distribution?
        if world_obs == 0 # no measurement yet
            # TODO: initialstate_distribution
            # TODO: initialstate
            # TODO: init Belief
            # belief = POMDPs.initialize_belief(belief_updater, )
        else
            ## TODO: Belief updater
            ## TODO: update belief from Observation

            a = POMDPs.action(planner, belief)
            belief_old = belief
            belief = POMDPs.update(belief_updater, belief_old, a, observation)
            
            ## TODO: get Action sequence instead of one action

            ## Dummy action sequence
            a_seq = IkaActionSequence()
            a_seq.header.stamp = RobotOS.now()
            a_seq.header.frame_id = "map"
            a_seq.actions = zeros(Int, 40)

            vel = ego_state_.fVelocity * 1.0

            # 1 2 3
            # 4 5 6
            # 7 8 9
            
            # +1.5|-0.5     1.5|0    1.5|+0.5
            #    0|-0.5       0|0      0|+0.5
            # -1.5|-0.5    -1.5|0   -1.5|+0.5
            for i = 1:40
                println(vel)
                if vel < 10.
                    a_seq.actions[i] = 2
                    vel = vel + 0.25 * action_matrix[1,2][1]
                end

                if vel > 10.
                    a_seq.actions[i] = 8
                    vel = vel + 0.25 * action_matrix[3,2][1]
                end
            end
            publish(pub_action, a_seq)
        end
        rossleep(loop_rate)
    end
end


function main()
    init_node("POMDP")

    pub_action = Publisher{IkaActionSequence}("/ika_decision_making/actions", queue_size = 20)

    sub_ego_state = Subscriber{IkaEgoState}("/localization/ego_state", callbackEgoState, queue_size = 20)

    sub_object_list = Subscriber{IkaObjectList}("/fusion/ikaObjectList", callbackObjectList, queue_size = 20)

    sub_route_status = Subscriber{IkaRouteStatus}("/route_planning/routeStatus", callbackRouteStatus, queue_size = 20)

    loop(pub_action)
end

if ! isinteractive()
    main()
end

end
