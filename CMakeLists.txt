# SPDX-License-Identifier: Apache-2.0

# Point to this project's root directory.
set(SCD4X_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "SCD4X module root directory")

# Subdirectories
add_subdirectory(drivers)

# Include headers
zephyr_include_directories(drivers)
zephyr_include_directories(include)
