# using LinearAlgebra
# using StaticArrays

# action_matrix = @SMatrix [ (1.5, -0.5)     (1.5, 0.)   (1.5, 0.);
#                 (0., -0.5)      (0., 0.)    (0., 0.5);
#                 (-1.5, -0.5)    (-1.5, 0.)  (-1.5, 0.5)]

# function getAcclerationValues(a::Symbol)
#     acc_lon = 0.0
#     acc_lat = 0.0
#     if a == :faster_left
#         acc_lon = action_matrix[1,1][1]
#         acc_lat = action_matrix[1,1][2]
#     elseif a == :faster
#         acc_lon = action_matrix[1,2][1]
#         acc_lat = action_matrix[1,2][2]
#     elseif a == :faster_right
#         acc_lon = action_matrix[1,3][1]
#         acc_lat = action_matrix[1,3][2]
#     elseif a == :neutral_left
#         acc_lon = action_matrix[2,1][1]
#         acc_lat = action_matrix[2,1][2]
#     elseif a == :neutral
#         acc_lon = action_matrix[2,2][1]
#         acc_lat = action_matrix[2,2][2]
#     elseif a == :neutral_right
#         acc_lon = action_matrix[2,3][1]
#         acc_lat = action_matrix[2,3][2]
#     elseif a == :slower_left
#         acc_lon = action_matrix[3,1][1]
#         acc_lat = action_matrix[3,1][2]
#     elseif a == :slower
#         acc_lon = action_matrix[3,2][1]
#         acc_lat = action_matrix[3,2][2]
#     elseif a == :slower_right
#         acc_lon = action_matrix[3,3][1]
#         acc_lat = action_matrix[3,3][2]
#     else
#         error("invalid action: $a")
#     end

#     return (acc_lon, acc_lat)
# end


# acc_lon = 0.0
# acc_lat = 0.0

# acc_lon, acc_lat = getAcclerationValues(:faster_left)

# println(acc_lon)
# println(acc_lat)



# using Cxx
# using Libdl
# using CxxWrap

# Load the module and generate the functions
module CppHello
  using CxxWrap
  @wrapmodule(joinpath("path/to/built/lib","libhello"))

  function __init__()
    @initcxx
  end
end


# Call greet and show the result
@show CppHello.greet()

# # Pkg.build(Cxx)
# cxx""" #include <iostream> """
# # cxx""" #include <fstream> """
# # cxx""" #include "/usr/include/eigen3/Eigen/Core" """
# # cxx""" #include "/usr/include/eigen3/Eigen/Geometry" """
# # cxx""" #include <lanelet2_core/primitives/Lanelet.h> """
# println(pwd())

# Libdl.dlopen("/home/cremer/git/automated-driving-platform/ros/build/lanelet2_core/CMakeFiles/lanelet2_core.dir/src" * "/Lanelet.cpp.o", Libdl.RTLD_GLOBAL)

# const path_to_lib = pwd()
# addHeaderDir(path_to_lib, kind=C_System)
# Libdl.dlopen("/home/cremer/git/automated-driving-platform/ros/build/lanelet2_core/CMakeFiles/lanelet2_core.dir/src/Lanelet.cpp", Libdl.RTLD_GLOBAL)
# cxxinclude("Lanelet.h")

# // LANELET2 Header
# include <lanelet2_core/primitives/Lanelet.h>
# include <lanelet2_core/primitives/GPSPoint.h>
# include <lanelet2_core/primitives/Point.h>
# include <lanelet2_core/LaneletMap.h>
# include <lanelet2_core/Forward.h>
# include <lanelet2_core/primitives/Area.h>
# include <lanelet2_core/geometry/Area.h>
# include <lanelet2_core/geometry/Point.h>
# include <lanelet2_core/geometry/BoundingBox.h>
# include <lanelet2_core/geometry/Lanelet.h>
# include <lanelet2_core/geometry/LaneletMap.h>
# include <lanelet2_core/primitives/LaneletOrArea.h>
# include <lanelet2_core/primitives/LaneletSequence.h>

# include <lanelet2_io/Io.h>

# include <lanelet2_projection/UTM.h>

# include <lanelet2_routing/Route.h>
# include <lanelet2_routing/RoutingCost.h>
# include <lanelet2_routing/RoutingGraph.h>
# include <lanelet2_routing/RoutingGraphContainer.h>
# // #include <lanelet2_routing/Types.h>
# include <lanelet2_routing/LaneletPath.h>
# include <lanelet2_routing/Forward.h>

# include <lanelet2_traffic_rules/TrafficRulesFactory.h>
# include <lanelet2_interface_ros/lanelet2_interface_ros.hpp>

cxx"""  
void mycppfunction() {   
            int z = 0;
            int y = 5;
            int x = 10;
            z = x*y + 2;
            std::cout << "The number is " << z << std::endl;
    }
   """

julia_function() = @cxx mycppfunction()

julia_function()