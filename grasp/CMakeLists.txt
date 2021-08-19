add_executable (grasp main.cpp canAPI.cpp RockScissorsPaper.cpp RedisClient.cpp)
target_link_libraries (grasp BHand pcanbasic pthread ${REDIS_LIBRARY})
#install (TARGETS grasp DESTINATION ${PROJECT_SOURCE_DIR}/bin)
install (TARGETS grasp DESTINATION ${PROJECT_BINARY_DIR}/bin)
