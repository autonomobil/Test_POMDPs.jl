###### TRANSITION FUNCTION
# function POMDPs.transition(pomdp::myPOMDP, s::State, a::Symbol)
#     # suppose we plan with 4Hz -> 0.25s steps
#     ego_state = s.ego_state
#     obj_states = s.obj_states

#     acc_lon = 0.0
#     acc_lat = 0.0
    
#     acc_lon, acc_lat = getAcclerationValues(a)

# 	ego_state[1] = ego_state[1] + 0.25 * ego_state.v + 0.5 * 0.25 * 0.25 * acc_lon;
# 	ego_state[2] = ego_state[2] + 0.5 * 0.25 * 0.25 * acc_lat;
# 	ego_state[3] = ego_state.v + 0.25 * sqrt(acc_lon*acc_lon + acc_lat*acc_lat);

# 	# for (uint i = 0; i < obj_states.size(); i++)
# 	# {
# 	# 	obj_states.at(i).s = obj_states.at(i).s + 0.25 * obj_states.at(i).v; // + 0.5 * 0.25 * 0.25 * acc;
# 	# 	obj_states.at(i).d = obj_states.at(i).d;
# 	# 	obj_states.at(i).v = obj_states.at(i).v; //+ 0.25 *a_k
# 	# 	obj_states.at(i).route_id = obj_states.at(i).route_id;

# 	# 	// check Collision!!!
# 	# 	// TODO: --> negative reward if collision or near collision
# 	# }

# 	# obs = 1;
# # return the transition distribution from the current State-Action pair
# end

###### OBERSERVATION FUNCTION
# function POMDPs.Observation(pomdp::myPOMDP, a::Symbol, sp::State)
#     # return a distribution
# end

###### REWARD FUNCTION
# function POMDPs.reward(pomdp::myPOMDP, s::State, a::Symbol, sp::State)
#     r = 0.0
#     return r
# end


# Generative Interface
# example:
# function POMDPs.gen(m::MyPOMDP, s, a, rng)
#   sp = s+a
#   r = s^2
#   o = s+a+randn(rng)
#   return (sp, r, o)
# end
