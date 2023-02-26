cmake_minimum_required(VERSION 3.5)
project(tendon_experiments LANGUAGES C CXX VERSION 1.0.0)

option(TENDON_USE_OPENMP "Use OpenMP" ON)
option(TENDON_LINK_PYTHON_SCRIPTS "Link python scripts instead of copying" ON)
option(TENDON_USE_ROS "Use ROS libraries" ON)
option(TENDON_BUILD_TESTING "Build testing" ON)

set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING
  "Build type choices: None Debug Release RelWithDebInfo MinSizeRel tsan asan lsan msan ubsan"
  FORCE
  )

# uncomment to have build output timing for each file
# - for better metrics, use clang with -ftime-trace and put the json file into
#   Google Chrome at chrome://tracing
# - another option is to use gcc with -v -ftime-report -ftime-report-details,
#   but it's not as good as with clang and Chrome.
#set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")
#set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CMAKE_COMMAND} -E time")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(octomap REQUIRED)
find_package(ompl REQUIRED)
find_package(fcl 0.6.1 EXACT REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem iostreams graph)
find_package(ITK REQUIRED)
find_package(Qt5 COMPONENTS Core SerialPort REQUIRED)

include(${ITK_USE_FILE})

find_package(PythonInterp 3)
find_package(PythonLibs 3)
find_package(pybind11)

if (TENDON_USE_OPENMP)
  find_package(OpenMP)
  if(OPENMP_FOUND)
    message(STATUS "OpenMP found")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  else()
    message(STATUS "OpenMP not found")
  endif()
endif()

if (CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)
#set(CMAKE_AUTOUIC ON)
find_package(Qt5 COMPONENTS Core Gui REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-O0")
# Note: any one of these flags cause OMPL to crash inexplicably: -mavx -mavx2 -mfma
# Note: -ffast-math turns on -fno-math-errno, -funsafe-math-optimizations,
#       -ffinite-math-only, -fno-rounding-math, -fno-signaling-nans,
#       -fcx-limited-range and -fexcess-precision=fast
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -DNDEBUG -mfpmath=sse -mtune=native")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG -mfpmath=sse -mtune=native")

option(TENDON_ENABLE_GPROF "Enable gprof" OFF)
if (TENDON_ENABLE_GPROF)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
endif()

# sanitizers integrated via the suggestion from
#   http://www.stablecoder.ca/2018/02/01/analyzer-build-types.html
# could also use this CMake module
#   https://github.com/arsenm/sanitizers-cmake

# ThreadSanitizer
set(CMAKE_C_FLAGS_TSAN
    "-fsanitize=thread -g -Ofast" #-O1"
    CACHE STRING "Flags used by the C compiler during ThreadSanitizer builds."
    FORCE)
set(CMAKE_CXX_FLAGS_TSAN
    "-fsanitize=thread -g -Ofast" #-O1"
    CACHE STRING "Flags used by the C++ compiler during ThreadSanitizer builds."
    FORCE)

# AddressSanitize
set(CMAKE_C_FLAGS_ASAN
    "-fsanitize=address -fno-optimize-sibling-calls -fsanitize-address-use-after-scope -fno-omit-frame-pointer -g -Ofast" #-O1"
    CACHE STRING "Flags used by the C compiler during AddressSanitizer builds."
    FORCE)
set(CMAKE_CXX_FLAGS_ASAN
    "-fsanitize=address -fno-optimize-sibling-calls -fsanitize-address-use-after-scope -fno-omit-frame-pointer -g -Ofast" #-O1"
    CACHE STRING "Flags used by the C++ compiler during AddressSanitizer builds."
    FORCE)

# LeakSanitizer
set(CMAKE_C_FLAGS_LSAN
    "-fsanitize=leak -fno-omit-frame-pointer -g -Ofast" #-O1"
    CACHE STRING "Flags used by the C compiler during LeakSanitizer builds."
    FORCE)
set(CMAKE_CXX_FLAGS_LSAN
    "-fsanitize=leak -fno-omit-frame-pointer -g -Ofast" #-O1"
    CACHE STRING "Flags used by the C++ compiler during LeakSanitizer builds."
    FORCE)

# MemorySanitizer
set(CMAKE_C_FLAGS_MSAN
    "-fsanitize=memory -fno-optimize-sibling-calls -fsanitize-memory-track-origins=2 -fno-omit-frame-pointer -g -Ofast" #-O2"
    CACHE STRING "Flags used by the C compiler during MemorySanitizer builds."
    FORCE)
set(CMAKE_CXX_FLAGS_MSAN
    "-fsanitize=memory -fno-optimize-sibling-calls -fsanitize-memory-track-origins=2 -fno-omit-frame-pointer -g -Ofast" #-O2"
    CACHE STRING "Flags used by the C++ compiler during MemorySanitizer builds."
    FORCE)

# UndefinedBehaviour
set(CMAKE_C_FLAGS_UBSAN
    "-fsanitize=undefined -fsanitize-minimal-runtime -fno-omit-frame-pointer -g -Ofast"
    CACHE STRING "Flags used by the C compiler during UndefinedBehaviourSanitizer builds."
    FORCE)
set(CMAKE_CXX_FLAGS_UBSAN
    "-fsanitize=undefined -fsanitize-minimal-runtime -fno-omit-frame-pointer -g -Ofast"
    CACHE STRING "Flags used by the C++ compiler during UndefinedBehaviourSanitizer builds."
    FORCE)

string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type_upper)
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS_${build_type_upper}: ${CMAKE_CXX_FLAGS_${build_type_upper}}")


set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src)
set(THIRD_PARTY_DIR ${SRC_DIR}/3rdparty)


# 3rd party
set(THIRD_PARTY_HEADERS)
set(NLOHMANN_HEADERS
  ${THIRD_PARTY_DIR}/nlohmann/json.hpp
  )
install(FILES ${NLOHMANN_HEADERS} DESTINATION include/nlohmann)
list(APPEND THIRD_PARTY_HEADERS ${NLOHMANN_HEADERS})
list(APPEND LIBTENDON_ALL_HEADERS ${THIRD_PARTY_HEADERS})

# util
set(UTIL_HEADERS
  ${SRC_DIR}/util/FunctionTimer.h
  ${SRC_DIR}/util/LifetimeTimer.h
  ${SRC_DIR}/util/SingleStopWatch.h
  ${SRC_DIR}/util/StopWatch.h
  ${SRC_DIR}/util/angles.h
  ${SRC_DIR}/util/eigen_ops.h
  ${SRC_DIR}/util/json_io.h
  ${SRC_DIR}/util/macros.h
  ${SRC_DIR}/util/ompl_logging.h
  ${SRC_DIR}/util/openfile_check.h
  ${SRC_DIR}/util/poly.h
  ${SRC_DIR}/util/string_ops.h
  ${SRC_DIR}/util/time_function_call.h
  ${SRC_DIR}/util/vector_ops.h
  )

# spline
set(SPLINE_HEADERS
  ${SRC_DIR}/spline/Cubic.h
  ${SRC_DIR}/spline/CubicSpline.h
  ${SRC_DIR}/spline/CubicSplineSequence.h
  )
install(FILES ${SPLINE_HEADERS} DESTINATION include/spline)
list(APPEND LIBTENDON_ALL_HEADERS ${SPLINE_HEADERS})

# cliparser
set(CLIPARSER_HEADERS ${SRC_DIR}/cliparser/CliParser.h)
install(FILES ${CLIPARSER_HEADERS} DESTINATION include/cliparser)
list(APPEND LIBTENDON_ALL_HEADERS ${CLIPARSER_HEADERS})

# csv
set(CSV_HEADERS ${SRC_DIR}/csv/Csv.h)
install(FILES ${CSV_HEADERS} DESTINATION include/csv)
list(APPEND LIBTENDON_ALL_HEADERS ${CSV_HEADERS})

# tip-controller
set(TIP_CONTROL_HEADERS
    ${SRC_DIR}/tip-control/tip_control.h
    ${SRC_DIR}/tip-control/Controller.h
    )
install(FILES ${TIP_CONTROL_HEADERS} DESTINATION include/tip-control)
list(APPEND LIBTENDON_ALL_HEADERS ${TIP_CONTROL_HEADERS})

# cpptoml
set(CPPTOML_HEADERS
  ${SRC_DIR}/cpptoml/cpptoml.h
  ${SRC_DIR}/cpptoml/toml_conversions.h
  )
install(FILES ${CPPTOML_HEADERS} DESTINATION include/cpptoml)
list(APPEND LIBTENDON_ALL_HEADERS ${CPPTOML_HEADERS})

# collision
set(COLLISION_HEADERS
  ${SRC_DIR}/collision/Capsule.h
  ${SRC_DIR}/collision/CapsuleSequence.h
  ${SRC_DIR}/collision/Mesh.h
  ${SRC_DIR}/collision/Point.h
  ${SRC_DIR}/collision/Sphere.h
  ${SRC_DIR}/collision/VoxelOctree.h
  ${SRC_DIR}/collision/OctomapWrap.h
  ${SRC_DIR}/collision/collision.h
  ${SRC_DIR}/collision/collision.hxx
  ${SRC_DIR}/collision/collision_primitives.h
  ${SRC_DIR}/collision/detail/TreeNode.h
  ${SRC_DIR}/collision/detail/TreeNode.hxx
  ${SRC_DIR}/collision/fcl_types.h
  ${SRC_DIR}/collision/stl_io.h
  )
install(FILES ${COLLISION_HEADERS} DESTINATION include/collision)
list(APPEND LIBTENDON_ALL_HEADERS ${COLLISION_HEADERS})

# motion planning
set(MOTION_PLANNING_HEADERS
  ${SRC_DIR}/motion-planning/AStarGoalVisitor.h
  ${SRC_DIR}/motion-planning/AbstractValidityChecker.h
  ${SRC_DIR}/motion-planning/AbstractVoxelMotionValidator.h
  ${SRC_DIR}/motion-planning/AbstractVoxelValidityChecker.h
  ${SRC_DIR}/motion-planning/Environment.h
  ${SRC_DIR}/motion-planning/LazyPRMFixed.h
  ${SRC_DIR}/motion-planning/OctomapValidityChecker.h
  ${SRC_DIR}/motion-planning/Problem.h
  ${SRC_DIR}/motion-planning/RetractionSampler.h
  ${SRC_DIR}/motion-planning/StraightLinePlanner.h
  ${SRC_DIR}/motion-planning/ValidityChecker.h
  ${SRC_DIR}/motion-planning/VoxelBackboneDiscreteMotionValidator.h
  ${SRC_DIR}/motion-planning/VoxelBackboneMotionValidator.h
  ${SRC_DIR}/motion-planning/VoxelBackboneMotionValidatorAndLogger.h
  ${SRC_DIR}/motion-planning/VoxelBackboneValidityChecker.h
  ${SRC_DIR}/motion-planning/VoxelCachedLazyPRM.h
  ${SRC_DIR}/motion-planning/VoxelEnvironment.h
  ${SRC_DIR}/motion-planning/VoxelValidityChecker.h
  ${SRC_DIR}/motion-planning/WSpaceGoal.h
  ${SRC_DIR}/motion-planning/ompl_planners.h
  ${SRC_DIR}/motion-planning/plan.h
  ${SRC_DIR}/motion-planning/io/RoadmapParser.h
  ${SRC_DIR}/motion-planning/io/RoadmapWriter.h
  )
install(FILES ${MOTION_PLANNING_HEADERS} DESTINATION include/motion-planning)
list(APPEND LIBTENDON_ALL_HEADERS ${MOTION_PLANNING_HEADERS})

# tendon
set(CONFIG_IN ${SRC_DIR}/tendon/config.h.in)
set(CONFIG_OUT ${CMAKE_CURRENT_BINARY_DIR}/include/tendon/config.h)
configure_file("${CONFIG_IN}" "${CONFIG_OUT}")
set(TENDON_HEADERS
  ${SRC_DIR}/tendon/BackboneSpecs.h
  ${SRC_DIR}/tendon/TendonResult.h
  ${SRC_DIR}/tendon/TendonRobot.h
  ${SRC_DIR}/tendon/TendonSpecs.h
  ${SRC_DIR}/tendon/get_r_info.h
  ${SRC_DIR}/tendon/solve_initial_bending.h
  ${SRC_DIR}/tendon/tendon_deriv.h
  ${CONFIG_OUT}
  )
install(FILES ${TENDON_HEADERS} DESTINATION include/tendon)
list(APPEND LIBTENDON_ALL_HEADERS ${CONFIG_IN})
list(APPEND LIBTENDON_ALL_HEADERS ${TENDON_HEADERS})


####################
#    libtendon     #
####################

set(TENDON_SOURCE
  ${SRC_DIR}/collision/Capsule.cpp
  ${SRC_DIR}/collision/CapsuleSequence.cpp
  ${SRC_DIR}/collision/Mesh.cpp
  ${SRC_DIR}/collision/Sphere.cpp
  ${SRC_DIR}/collision/VoxelOctree.cpp
  ${SRC_DIR}/collision/collision.cpp
  ${SRC_DIR}/collision/collision_primitives.cpp
  ${SRC_DIR}/cpptoml/cpptoml.cpp
  ${SRC_DIR}/cpptoml/toml_conversions.cpp
  ${SRC_DIR}/csv/Csv.cpp
  ${SRC_DIR}/motion-planning/AbstractValidityChecker.cpp
  ${SRC_DIR}/motion-planning/Environment.cpp
  ${SRC_DIR}/motion-planning/LazyPRMFixed.cpp
  ${SRC_DIR}/motion-planning/Problem.cpp
  ${SRC_DIR}/motion-planning/StraightLinePlanner.cpp
  ${SRC_DIR}/motion-planning/VoxelBackboneMotionValidator.cpp
  ${SRC_DIR}/motion-planning/VoxelBackboneDiscreteMotionValidator.cpp
  ${SRC_DIR}/motion-planning/VoxelBackboneMotionValidatorAndLogger.cpp
  ${SRC_DIR}/motion-planning/VoxelCachedLazyPRM.cpp
  ${SRC_DIR}/motion-planning/VoxelEnvironment.cpp
  ${SRC_DIR}/motion-planning/ompl_planners.cpp
  ${SRC_DIR}/motion-planning/plan.cpp
  ${SRC_DIR}/tendon/BackboneSpecs.cpp
  ${SRC_DIR}/tendon/TendonResult.cpp
  ${SRC_DIR}/tendon/TendonRobot.cpp
  ${SRC_DIR}/tendon/TendonSpecs.cpp
  ${SRC_DIR}/tendon/get_r_info.cpp
  ${SRC_DIR}/tendon/solve_initial_bending.cpp
  ${SRC_DIR}/tendon/tendon_deriv.cpp
  ${SRC_DIR}/tip-control/Controller.cpp
  ${SRC_DIR}/tip-control/tip_control.cpp
  ${SRC_DIR}/util/eigen_ops.cpp
  ${SRC_DIR}/util/json_io.cpp

  ${LIBTENDON_ALL_HEADERS}
  )
add_library(tendon SHARED ${TENDON_SOURCE})
target_include_directories(tendon PUBLIC
  ${SRC_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/include # for config.h
  ${THIRD_PARTY_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  #${FCL_INCLUDE_DIRS}
  ${ITK_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  )
set_property(TARGET tendon PROPERTY CXX_STANDARD 17)
target_include_directories(tendon PUBLIC ${SRC_DIR})
target_link_libraries(tendon
  #${FCL_LIBRARIES}
  ${ITK_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OMPL_LIBRARIES}
  fcl
  Boost::filesystem
  Boost::iostreams
  Boost::system
  Eigen3::Eigen
  levmar
  readline
  )

target_compile_definitions(tendon PUBLIC CPPTOML_USE_MAP)
install(TARGETS tendon DESTINATION lib/${PROJECT_NAME})


add_subdirectory(${THIRD_PARTY_DIR}/levmar-2.6 3rdparty/levmar-2.6)


####################
#   applications   #
####################

set(APP_SRC_DIR ${SRC_DIR}/apps)

# plan cost
add_executable(plan_cost ${APP_SRC_DIR}/plan_cost.cpp)
set_property(TARGET plan_cost PROPERTY CXX_STANDARD 17)
target_link_libraries(plan_cost tendon)
install(TARGETS plan_cost DESTINATION lib/${PROJECT_NAME})

# query planner
add_executable(query_planner ${APP_SRC_DIR}/query_planner.cpp)
set_property(TARGET query_planner PROPERTY CXX_STANDARD 17)
target_link_libraries(query_planner tendon)
install(TARGETS query_planner DESTINATION lib/${PROJECT_NAME})

# estimate tension limits
add_executable(estimate_tension_limits ${APP_SRC_DIR}/estimate_tension_limits.cpp)
set_property(TARGET estimate_tension_limits PROPERTY CXX_STANDARD 17)
target_link_libraries(estimate_tension_limits tendon)
install(TARGETS estimate_tension_limits DESTINATION lib/${PROJECT_NAME})

# estimate length limits
add_executable(estimate_length_limits ${APP_SRC_DIR}/estimate_length_limits.cpp)
set_property(TARGET estimate_length_limits PROPERTY CXX_STANDARD 17)
target_link_libraries(estimate_length_limits tendon)
install(TARGETS estimate_length_limits DESTINATION lib/${PROJECT_NAME})

# estimate length discretization
add_executable(estimate_length_discretization ${APP_SRC_DIR}/estimate_length_discretization.cpp)
set_property(TARGET estimate_length_discretization PROPERTY CXX_STANDARD 17)
target_link_libraries(estimate_length_discretization tendon)
install(TARGETS estimate_length_discretization DESTINATION lib/${PROJECT_NAME})

# sample tips
add_executable(sample_tips ${APP_SRC_DIR}/sample_tips.cpp)
set_property(TARGET sample_tips PROPERTY CXX_STANDARD 17)
target_link_libraries(sample_tips tendon)
install(TARGETS sample_tips DESTINATION lib/${PROJECT_NAME})

# nrrd 2 mesh
add_executable(nrrd2mesh ${APP_SRC_DIR}/nrrd2mesh.cpp)
set_property(TARGET nrrd2mesh PROPERTY CXX_STANDARD 17)
target_link_libraries(nrrd2mesh tendon)
install(TARGETS nrrd2mesh DESTINATION lib/${PROJECT_NAME})

# nrrd resample
add_executable(nrrdresample ${APP_SRC_DIR}/nrrdresample.cpp)
set_property(TARGET nrrdresample PROPERTY CXX_STANDARD 17)
target_include_directories(nrrdresample PUBLIC ${ITK_INCLUDE_DIRS})
target_link_libraries(nrrdresample ${ITK_LIBRARIES})
install(TARGETS nrrdresample DESTINATION lib/${PROJECT_NAME})

# prepare voxel env
add_executable(prepare_voxel_env ${APP_SRC_DIR}/prepare_voxel_env.cpp)
set_property(TARGET prepare_voxel_env PROPERTY CXX_STANDARD 17)
target_link_libraries(prepare_voxel_env tendon Boost::filesystem)
install(TARGETS prepare_voxel_env DESTINATION lib/${PROJECT_NAME})

# voxel convert
add_executable(voxel_convert ${APP_SRC_DIR}/voxel_convert.cpp)
set_property(TARGET voxel_convert PROPERTY CXX_STANDARD 17)
target_link_libraries(voxel_convert tendon)
install(TARGETS voxel_convert DESTINATION lib/${PROJECT_NAME})

# create roadmap
add_executable(create_roadmap ${APP_SRC_DIR}/create_roadmap.cpp)
set_property(TARGET create_roadmap PROPERTY CXX_STANDARD 17)
target_link_libraries(create_roadmap tendon)
install(TARGETS create_roadmap DESTINATION lib/${PROJECT_NAME})

# roadmap to samples
add_executable(roadmap2samples ${APP_SRC_DIR}/roadmap2samples.cpp)
set_property(TARGET roadmap2samples PROPERTY CXX_STANDARD 17)
target_link_libraries(roadmap2samples tendon)
install(TARGETS roadmap2samples DESTINATION lib/${PROJECT_NAME})

# profile chained plan
add_executable(profile_chained_plan ${APP_SRC_DIR}/profile_chained_plan.cpp)
set_property(TARGET profile_chained_plan PROPERTY CXX_STANDARD 17)
target_link_libraries(profile_chained_plan tendon)
install(TARGETS profile_chained_plan DESTINATION lib/${PROJECT_NAME})

# roadmap chained plan
add_executable(roadmap_chained_plan ${APP_SRC_DIR}/roadmap_chained_plan.cpp)
set_property(TARGET roadmap_chained_plan PROPERTY CXX_STANDARD 17)
target_link_libraries(roadmap_chained_plan tendon)
install(TARGETS roadmap_chained_plan DESTINATION lib/${PROJECT_NAME})

#######################
#   python bindings   #
#######################

if (${pybind11_FOUND})
  set(PYTHON_BINDINGS_SRC_DIR ${SRC_DIR}/python-bindings)
  pybind11_add_module(cpptendon
    ${PYTHON_BINDINGS_SRC_DIR}/module_cpptendon.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_collision.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_collision.h
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_controller.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_controller.h
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_motion_planning.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_motion_planning.h
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_tendon.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_tendon.h
    )
  target_link_libraries(cpptendon PUBLIC tendon)
  set_property(TARGET cpptendon PROPERTY CXX_STANDARD 17)
  install(TARGETS cpptendon DESTINATION lib/${PROJECT_NAME})
endif()



######################
#   python scripts   #
######################

set(PYTHON_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/python/src)

# creates the target for scriptfile without the .py at the end and copies it
# into the build directory.
function(add_python_script scriptfile)
  get_filename_component(filebase ${scriptfile} NAME_WE)
  set(filepath ${PYTHON_SRC_DIR}/${scriptfile})
  set(dest ${CMAKE_BINARY_DIR}/${scriptfile})
  if(TENDON_LINK_PYTHON_SCRIPTS)
    set(command ${CMAKE_COMMAND} -E create_symlink ${filepath} ${dest})
  else()
    #configure_file(${PYTHON_SRC_DIR}/${scriptfile} ${scriptfile} COPYONLY)
    set(command ${CMAKE_COMMAND} -E copy ${filepath} ${dest})
  endif(TENDON_LINK_PYTHON_SCRIPTS)
  add_custom_target(${filebase} ALL
    COMMAND ${command}
    SOURCES ${filepath}
    BYPRODUCTS ${dest}
    )
  install(PROGRAMS ${dest} DESTINATION lib/${PROJECT_NAME})
endfunction(add_python_script)

add_python_script(aggregate_roadmap_stats.py)
add_python_script(csv_combine.py)
add_python_script(plot_roadmap_chain_histogram.py)
add_python_script(roadmap_stats.py)

if (TARGET cpptendon)
  add_python_script(voxel_ops.py)
  add_dependencies(voxel_ops cpptendon)
  add_python_script(run_ik.py)
  add_dependencies(run_ik cpptendon)
  add_python_script(run_fk.py)
  add_dependencies(run_fk cpptendon)
endif()
