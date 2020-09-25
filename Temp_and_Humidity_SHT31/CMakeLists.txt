#  Copyright (c) Microsoft Corporation. All rights reserved.
#  Licensed under the MIT License.

cmake_minimum_required (VERSION 3.10)
project (grove-temp-humidity-sht31 C)

add_subdirectory("../MT3620_Grove_Shield_Library" out)
# Create executable
add_executable (${PROJECT_NAME} main.c)

target_include_directories(${PROJECT_NAME} PUBLIC ${AZURE_SPHERE_API_SET_DIR}/usr/include/azureiot ${AZURE_SPHERE_API_SET_DIR}/usr/include/azure_prov_client)
target_compile_definitions(${PROJECT_NAME} PUBLIC AZURE_IOT_HUB_CONFIGURED)

target_link_libraries (${PROJECT_NAME} m azureiot applibs pthread gcc_s c MT3620_Grove_Shield_Library)

# Add MakeImage post-build command
include (${AZURE_SPHERE_MAKE_IMAGE_FILE})


