cmake_minimum_required(VERSION 3.8)
project(mondy_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.py"
    PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
)

ament_package()
