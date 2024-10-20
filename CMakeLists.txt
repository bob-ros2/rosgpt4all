cmake_minimum_required(VERSION 3.8)
project(rosgpt4all)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python executables
install(PROGRAMS
  scripts/gpt
  scripts/terminal
  scripts/json_embedder.py
  scripts/qdrant_embedder.py
  scripts/chroma_embedder.py
  scripts/embedder
  scripts/vector
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch ajnd config files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
