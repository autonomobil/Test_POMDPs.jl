using POMDPs, POMDPModels, POMDPSimulators, POMCPOW, POMDPModelTools, Random

m = TigerPOMDP()

solver = POMCPOWSolver(tree_in_info=true)
planner = solve(solver, m)

b = initialstate_distribution(m)

function show_tree_and_best_sequence(planner, b)
    a, info = action_info(planner, b)

    tree = info[:tree]
    show(stdout, MIME("text/plain"), tree)

    bindex = 1
    a_sequence = actiontype(m)[]
    while !isempty(tree.tried[bindex])
        anodes = tree.tried[bindex]
        bnode = POWTreeObsNode(tree, bindex)
        best_anode = POMCPOW.select_best(MaxQ(), bnode, MersenneTwister(1))
        push!(a_sequence, tree.a_labels[best_anode])
        children = [pair[2] for pair in tree.generated[best_anode]]
        # find most likely observation
        bindex = children[argmax([tree.total_n[c] for c in children])]
    end
    @show a_sequence
end

show_tree_and_best_sequence(planner, b)