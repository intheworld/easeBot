cmake_minimum_required(VERSION 3.16)
project(hei_hardware)
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_program(CARGO_EXECUTABLE cargo REQUIRED)
set(BACKEND_DIR "${CMAKE_CURRENT_SOURCE_DIR}/motorbridge_backend")
set(BACKEND_TARGET "${CMAKE_CURRENT_BINARY_DIR}/motorbridge-target")
set(BACKEND_LIBRARY "${BACKEND_TARGET}/release/libhei_motorbridge_backend.so")
file(GLOB_RECURSE BACKEND_SOURCES CONFIGURE_DEPENDS "${BACKEND_DIR}/src/*.rs")
add_custom_command(OUTPUT "${BACKEND_LIBRARY}"
  COMMAND "${CMAKE_COMMAND}" -E env --unset=MAKEFLAGS --unset=MFLAGS --unset=CARGO_MAKEFLAGS
    "${CARGO_EXECUTABLE}" build --offline --locked --release --jobs 2
    --manifest-path "${BACKEND_DIR}/Cargo.toml" --target-dir "${BACKEND_TARGET}"
  WORKING_DIRECTORY /tmp  # Do not inherit ROS message patches from the workspace .cargo/config.
  DEPENDS ${BACKEND_SOURCES} "${BACKEND_DIR}/Cargo.toml" "${BACKEND_DIR}/Cargo.lock"
  COMMENT "Building pinned motorbridge Rust backend (run scripts/setup.bash first)")
add_custom_target(hei_motorbridge_build DEPENDS "${BACKEND_LIBRARY}")
add_library(hei_motorbridge_backend SHARED IMPORTED)
set_target_properties(hei_motorbridge_backend PROPERTIES IMPORTED_NO_SONAME TRUE IMPORTED_LOCATION "${BACKEND_LIBRARY}")
add_dependencies(hei_motorbridge_backend hei_motorbridge_build)

add_library(${PROJECT_NAME} SHARED src/damiao_system.cpp)
# The installed plugin and Rust library are siblings; no build-tree dependency.
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")
target_link_libraries(${PROJECT_NAME} PRIVATE hei_motorbridge_backend)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface hei_hardware.xml)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES "${BACKEND_LIBRARY}" DESTINATION lib)
# Separate controller plugin, same package; it never links or opens the serial backend.
add_library(hei_hardware_gripper SHARED src/gripper_controller.cpp)
target_compile_features(hei_hardware_gripper PUBLIC cxx_std_17)
target_compile_options(hei_hardware_gripper PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(hei_hardware_gripper controller_interface pluginlib rclcpp rclcpp_action control_msgs)
pluginlib_export_plugin_description_file(controller_interface hei_controllers.xml)
install(TARGETS hei_hardware_gripper LIBRARY DESTINATION lib)
install(FILES README.md GRIPPER.md SOURCE.md MOTORBRIDGE-LICENSE DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_damiao test/test_damiao.cpp test/wire_fixture.cpp)
  target_include_directories(test_damiao PRIVATE test)
  ament_target_dependencies(test_damiao controller_interface)
  add_dependencies(test_damiao hei_hardware_gripper)
  target_link_libraries(test_damiao ${PROJECT_NAME} hei_motorbridge_backend util)
endif()
ament_package()
