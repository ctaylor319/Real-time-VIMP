# CMake generated Testfile for 
# Source directory: /home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf
# Build directory: /home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(cpplint "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/test_results/grid_map_sdf/cpplint.xunit.xml" "--package-name" "grid_map_sdf" "--output-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/ament_cpplint/cpplint.txt" "--command" "/opt/ros/humble/bin/ament_cpplint" "--xunit-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/test_results/grid_map_sdf/cpplint.xunit.xml" "--filters=-legal/copyright,-build/include_order")
set_tests_properties(cpplint PROPERTIES  LABELS "cpplint;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_cpplint/cmake/ament_cpplint.cmake;68;ament_add_test;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf/CMakeLists.txt;91;ament_cpplint;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf/CMakeLists.txt;0;")
add_test(grid_map_sdf-test "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/test_results/grid_map_sdf/grid_map_sdf-test.gtest.xml" "--package-name" "grid_map_sdf" "--output-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/ament_cmake_gtest/grid_map_sdf-test.txt" "--command" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/grid_map_sdf-test" "--gtest_output=xml:/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/test_results/grid_map_sdf/grid_map_sdf-test.gtest.xml")
set_tests_properties(grid_map_sdf-test PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/grid_map_sdf-test" TIMEOUT "60" WORKING_DIRECTORY "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf/CMakeLists.txt;100;ament_add_gtest;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf/CMakeLists.txt;0;")
add_test(cppcheck "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/test_results/grid_map_sdf/cppcheck.xunit.xml" "--package-name" "grid_map_sdf" "--output-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/humble/bin/ament_cppcheck" "--xunit-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/test_results/grid_map_sdf/cppcheck.xunit.xml" "--include_dirs" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf/include")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;66;ament_add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;87;ament_cppcheck;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf/CMakeLists.txt;116;ament_package;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/test_results/grid_map_sdf/lint_cmake.xunit.xml" "--package-name" "grid_map_sdf" "--output-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/humble/bin/ament_lint_cmake" "--xunit-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/test_results/grid_map_sdf/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf/CMakeLists.txt;116;ament_package;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf/CMakeLists.txt;0;")
add_test(uncrustify "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/test_results/grid_map_sdf/uncrustify.xunit.xml" "--package-name" "grid_map_sdf" "--output-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/humble/bin/ament_uncrustify" "--xunit-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/test_results/grid_map_sdf/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_uncrustify/cmake/ament_uncrustify.cmake;70;ament_add_test;/opt/ros/humble/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;43;ament_uncrustify;/opt/ros/humble/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf/CMakeLists.txt;116;ament_package;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/test_results/grid_map_sdf/xmllint.xunit.xml" "--package-name" "grid_map_sdf" "--output-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/ament_xmllint/xmllint.txt" "--command" "/opt/ros/humble/bin/ament_xmllint" "--xunit-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/grid_map_sdf/test_results/grid_map_sdf/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf/CMakeLists.txt;116;ament_package;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/grid_map/grid_map_sdf/CMakeLists.txt;0;")
subdirs("gtest")
