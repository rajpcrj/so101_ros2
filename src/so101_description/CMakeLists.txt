cmake_minimum_required(VERSION 3.8)
project(so101_description)

find_package(ament_cmake REQUIRED)

# Copy mesh assets into the build tree first so the installed package does not
# rely on symlinks back to the source workspace.
file(GLOB SO101_ASSET_FILES
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  CONFIGURE_DEPENDS
  "assets/*.stl"
  "assets/*.part"
)

foreach(asset_file IN LISTS SO101_ASSET_FILES)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/${asset_file}"
    "${CMAKE_CURRENT_BINARY_DIR}/${asset_file}"
    COPYONLY
  )
endforeach()

install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/assets"
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()
