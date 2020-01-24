#!/usr/bin/env julia
module ika_decision_making
# https://jdlangs.github.io/RobotOS.jl/stable/

using Pkg
using Conda
ENV["PYTHON"]="/usr/bin/python2.7"
# Pkg.build("PyCall")
## https://github.com/jdlangs/RobotOS.jl/issues/54 
## https://github.com/JuliaPy/PyCall.jl/issues/569 
## https://github.com/JuliaPy/PyPlot.jl/issues/286 

using RobotOS # ROS stuff for julia
using POMDPs
using POMCPOW # POMCPOW is an online solver based on Monte Carlo tree search for POMDPs with continuous state, action, and observation spaces
using MCTS # monte carlo tree search solver for online mdp
using POMDPModels
# using POMDPSimulators
using POMDPPolicies
using Random
using POMDPModelTools # for Deterministic


# solver = POMCPOWSolver(criterion=MaxUCB(20.0))

# import ros messages
@rosimport geometry_msgs.msg:Point, Pose2D
@rosimport definitions.msg:IkaEgoMotion, IkaObject, IkaObjectList, IkaGPS, IkaActionSequence

rostypegen(@__MODULE__)

# use messages
using .geometry_msgs.msg
using .definitions.msg

println("msg import success")

#### pomdp process example
mdp = SimpleGridWorld(size=(20,50))
solver = MCTSSolver(n_iterations=50, depth=20, exploration_constant=5.0)
planner = solve(solver, mdp) # policy = solve(solver, mdp)
state = initialstate(mdp, Random.MersenneTwister(4))
a = action(planner, state)


function GPS_callback(msg::IkaGPS, pub_obj::Publisher{Point})
    pt_msg = Point(msg.fLatitude, msg.fLongitude, 0.0)
    println("msg.fLatitude: ", msg.fLatitude)
    publish(pub_obj, pt_msg)
end

function main()
    init_node("POMDP")

    #############################
    test_pub = Publisher{Point}("test_pub", queue_size = 10)
    sub = Subscriber{IkaGPS}("ikaGPS", GPS_callback, (test_pub,), queue_size = 10)

    # test_pub2 = Publisher{Point}("test_pub2", queue_size = 10)
    # sub = Subscriber{IkaEgoMotion}("ikaEgoMotion", egoMotion_callback, (test_pub,), queue_size = 10)

    loop_rate = Rate(10.0)
    while ! is_shutdown()
        spin()
        rossleep(loop_rate)
    end  
end

if ! isinteractive()
    main()
end

end