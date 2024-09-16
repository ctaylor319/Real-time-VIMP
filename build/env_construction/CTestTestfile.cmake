# CMake generated Testfile for 
# Source directory: /home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction
# Build directory: /home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/env_construction
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(cppcheck "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/env_construction/test_results/env_construction/cppcheck.xunit.xml" "--package-name" "env_construction" "--output-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/env_construction/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/humble/bin/ament_cppcheck" "--xunit-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/env_construction/test_results/env_construction/cppcheck.xunit.xml" "--include_dirs" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction/dependencies" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction/include/env_construction")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;66;ament_add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;87;ament_cppcheck;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction/CMakeLists.txt;61;ament_package;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/env_construction/test_results/env_construction/lint_cmake.xunit.xml" "--package-name" "env_construction" "--output-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/env_construction/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/humble/bin/ament_lint_cmake" "--xunit-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/env_construction/test_results/env_construction/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction/CMakeLists.txt;61;ament_package;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction/CMakeLists.txt;0;")
add_test(uncrustify "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/env_construction/test_results/env_construction/uncrustify.xunit.xml" "--package-name" "env_construction" "--output-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/env_construction/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/humble/bin/ament_uncrustify" "--xunit-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/env_construction/test_results/env_construction/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_uncrustify/cmake/ament_uncrustify.cmake;70;ament_add_test;/opt/ros/humble/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;43;ament_uncrustify;/opt/ros/humble/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction/CMakeLists.txt;61;ament_package;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/env_construction/test_results/env_construction/xmllint.xunit.xml" "--package-name" "env_construction" "--output-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/env_construction/ament_xmllint/xmllint.txt" "--command" "/opt/ros/humble/bin/ament_xmllint" "--xunit-file" "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/build/env_construction/test_results/env_construction/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction/CMakeLists.txt;61;ament_package;/home/ctaylor71023/ros2_ws/src/ctaylor319_capstone/env_construction/CMakeLists.txt;0;")
