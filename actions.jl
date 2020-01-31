##############
# Action space
#  3x3 lon/lat acc  = 9 available actions
#     1           2        3
#     4           5        6
#     7           8        9
# 
# +1.5|+0.5    1.5|0    1.5|-0.5
#    0|+0.5      0|0      0|-0.5
# -1.5|+0.5   -1.5|0   -1.5|-0.5

action_matrix = @SMatrix [
    (1.5, +0.5) (1.5, 0.0) (1.5, -0.5)
    (0.0, +0.5) (0.0, 0.0) (0.0, -0.5)
    (-1.5, +0.5) (-1.5, 0.0) (-1.5, -0.5)
]

# ? const action_sequence = SVector{40,Action} # 40 actions @ 4Hz = 10 seconds planning horizon

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
