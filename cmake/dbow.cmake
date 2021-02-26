
add_subdirectory(Thirdparty/DBow3)

include_directories(
        ${PROJECT_SOURCE_DIR}/Thirdparty
)
list(APPEND ALL_TARGET_LIBRARIES
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBow3/build/src/libDBoW3.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBow3/build/src/libDBoW3.so.0.0
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBow3/build/src/libDBoW3.so.0.0.1
        )