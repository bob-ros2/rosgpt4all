cmake_minimum_required(VERSION 3.8)
project(rosgpt4all)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install launch and config files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python libs
install(PROGRAMS
  ${PROJECT_NAME}/__init__.py
  ${PROJECT_NAME}/json_embedder.py
  ${PROJECT_NAME}/qdrant_embedder.py
  ${PROJECT_NAME}/chroma_embedder.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
ament_python_install_package(${PROJECT_NAME}
  SETUP_CFG
    ${PROJECT_NAME}/setup.cfg
  SCRIPTS_DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
