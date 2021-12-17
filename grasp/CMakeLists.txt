# create an executable in bin/
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
add_executable (grasp main.cpp canAPI.cpp RockScissorsPaper.cpp RedisClient.cpp predefinedGrasps.cpp)

# and link the library against the executable
target_link_libraries (grasp BHand pcanbasic pthread ${REDIS_LIBRARY})

#install (TARGETS grasp DESTINATION ${PROJECT_SOURCE_DIR}/bin)
install (TARGETS grasp DESTINATION ${PROJECT_BINARY_DIR}/bin)

# copy control launch script to runtime directory
FILE(COPY launch_grasp.sh DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
