# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

add_executable(capture_perf captureperf.cpp)

target_compile_definitions(capture_perf PRIVATE _CRT_SECURE_NO_WARNINGS)

target_link_libraries(capture_perf PRIVATE
    azure::aziotsharedutil
    gtest::gtest
    k4a::k4a
    k4ainternal::utcommon)

k4a_add_tests(TARGET capture_perf TEST_TYPE UNIT)
