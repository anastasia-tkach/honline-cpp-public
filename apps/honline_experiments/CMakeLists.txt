INCLUDE_DIRECTORIES(../../) #< #include "tracker/..."
file(GLOB_RECURSE SRC "*.cpp")
file(GLOB_RECURSE HDR "*.h")
add_executable(honline_experiments ${HDR} ${SRC})
target_link_libraries(honline_experiments honlinesdk)

