# Get the name of the current directory
get_filename_component(p ${CMAKE_CURRENT_SOURCE_DIR} NAME)
# Get the name of the parent directory
get_filename_component(pp ${CMAKE_CURRENT_SOURCE_DIR} PATH)
get_filename_component(pp ${pp} NAME)
# Set the name of the parent directory as the prefix
set(p ${pp}_${p}_)

set (name cliente)

set(target ${p}${name})
add_executable(${target} ${name}.c)
target_link_libraries(${target} zmq cJSON zhelpers utils)
set_target_properties(${target} PROPERTIES OUTPUT_NAME ${name})
set_target_properties(${target} PROPERTIES EXPORT_NAME ${name})

set (name broker)

set(target ${p}${name})
add_executable(${target} ${name}.c)
target_link_libraries(${target} zmq cJSON zhelpers)
set_target_properties(${target} PROPERTIES OUTPUT_NAME ${name})
set_target_properties(${target} PROPERTIES EXPORT_NAME ${name})

set (name worker)

set(target ${p}${name})
add_executable(${target} ${name}.c)
target_link_libraries(${target} zmq cJSON zhelpers)
set_target_properties(${target} PROPERTIES OUTPUT_NAME ${name})
set_target_properties(${target} PROPERTIES EXPORT_NAME ${name})
