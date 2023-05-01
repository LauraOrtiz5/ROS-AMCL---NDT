# CMake generated Testfile for 
# Source directory: /home/laura/ros2_ws/src/navigation2/nav2_costmap_2d/test/regression
# Build directory: /home/laura/ros2_ws/build/nav2_costmap_2d/test/regression
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(costmap_bresenham_2d "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/laura/ros2_ws/build/nav2_costmap_2d/test_results/nav2_costmap_2d/costmap_bresenham_2d.gtest.xml" "--package-name" "nav2_costmap_2d" "--output-file" "/home/laura/ros2_ws/build/nav2_costmap_2d/ament_cmake_gtest/costmap_bresenham_2d.txt" "--command" "/home/laura/ros2_ws/build/nav2_costmap_2d/test/regression/costmap_bresenham_2d" "--gtest_output=xml:/home/laura/ros2_ws/build/nav2_costmap_2d/test_results/nav2_costmap_2d/costmap_bresenham_2d.gtest.xml")
set_tests_properties(costmap_bresenham_2d PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/laura/ros2_ws/build/nav2_costmap_2d/test/regression/costmap_bresenham_2d" TIMEOUT "60" WORKING_DIRECTORY "/home/laura/ros2_ws/build/nav2_costmap_2d/test/regression" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/laura/ros2_ws/src/navigation2/nav2_costmap_2d/test/regression/CMakeLists.txt;2;ament_add_gtest;/home/laura/ros2_ws/src/navigation2/nav2_costmap_2d/test/regression/CMakeLists.txt;0;")
add_test(plugin_api_order "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/laura/ros2_ws/build/nav2_costmap_2d/test_results/nav2_costmap_2d/plugin_api_order.gtest.xml" "--package-name" "nav2_costmap_2d" "--output-file" "/home/laura/ros2_ws/build/nav2_costmap_2d/ament_cmake_gtest/plugin_api_order.txt" "--command" "/home/laura/ros2_ws/build/nav2_costmap_2d/test/regression/plugin_api_order" "--gtest_output=xml:/home/laura/ros2_ws/build/nav2_costmap_2d/test_results/nav2_costmap_2d/plugin_api_order.gtest.xml")
set_tests_properties(plugin_api_order PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/laura/ros2_ws/build/nav2_costmap_2d/test/regression/plugin_api_order" TIMEOUT "60" WORKING_DIRECTORY "/home/laura/ros2_ws/build/nav2_costmap_2d/test/regression" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/laura/ros2_ws/src/navigation2/nav2_costmap_2d/test/regression/CMakeLists.txt;21;ament_add_gtest;/home/laura/ros2_ws/src/navigation2/nav2_costmap_2d/test/regression/CMakeLists.txt;0;")
