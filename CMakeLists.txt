# Version minima requerida para generar este script
cmake_minimum_required(VERSION 3.8)

# Crea el proyecto principal llamado 'Cirobb Engine' Solucion
project(Cirobb VERSION 1.1.7) 

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)


find_package(OpenGL REQUIRED)
add_subdirectory(glad)
add_subdirectory(cirobb)
add_subdirectory(demo)
add_subdirectory(glfw)
add_subdirectory(imgui)

# Proyecto de inicio predeterminado para Visual Studio
if (MSVC)
	set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT demo)
endif()

