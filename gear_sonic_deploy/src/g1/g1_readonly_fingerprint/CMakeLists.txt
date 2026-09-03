set(FINGERPRINT_TARGET g1_readonly_fingerprint)

add_executable(${FINGERPRINT_TARGET} src/g1_readonly_fingerprint.cpp)
target_include_directories(${FINGERPRINT_TARGET} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${FINGERPRINT_TARGET} PRIVATE g1_deploy Threads::Threads)
target_compile_options(${FINGERPRINT_TARGET} PRIVATE
  -fno-fast-math -Wall -Wextra -Wpedantic)
set_target_properties(${FINGERPRINT_TARGET} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/target/release/"
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)

if(BUILD_TESTING)
  add_executable(g1_readonly_fingerprint_tests
    tests/g1_readonly_fingerprint_tests.cpp)
  target_include_directories(g1_readonly_fingerprint_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_options(g1_readonly_fingerprint_tests PRIVATE
    -fno-fast-math -Wall -Wextra -Wpedantic)
  set_target_properties(g1_readonly_fingerprint_tests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/target/release/"
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

  add_test(NAME g1_readonly_fingerprint_tests
    COMMAND g1_readonly_fingerprint_tests)
endif()
