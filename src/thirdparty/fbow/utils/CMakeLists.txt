# Vocabulary creator step 0
add_executable(fbow_create_voc_step0
  fbow_create_voc_step0.cpp)
target_include_directories(fbow_create_voc_step0 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fbow_create_voc_step0
  fbow
  ${OpenCV_LIBS})

# Vocabulary creator step 1
add_executable(fbow_create_voc_step1
  fbow_create_voc_step1.cpp)
target_include_directories(fbow_create_voc_step1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fbow_create_voc_step1
  fbow
  ${OpenCV_LIBS})

install(
  TARGETS fbow_create_voc_step0 fbow_create_voc_step1
  DESTINATION lib/${PROJECT_NAME})
