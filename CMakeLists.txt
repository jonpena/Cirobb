# Version minima requerida para generar este script
cmake_minimum_required(VERSION 3.8)

# Crea el proyecto principal llamado 'Cirobb Engine' Solucion
project(CirobbEngine VERSION 1.1.4) 

# Variable INCLUDE_FILES
set(INCLUDE_FILES

	include/CbMath.h
	include/Collision.h 
	include/RigidBody.h
	include/glut.h 
	include/Manifold.h
	include/Scene.h
	include/Shape.h
)

# Variable SOURCE_FILES
set(SOURCE_FILES

	src/Collision.cpp 
	src/RigidBody.cpp
	src/Manifold.cpp
	src/Scene.cpp
	src/Main.cpp
	src/Shape.cpp
)


# Esto es usado para crear carpetas de visual studio.
source_group(include FILES ${INCLUDE_FILES})
source_group(src FILES ${SOURCE_FILES})


# A veces es necesario incluir archivos de cabecera en localizaciones no estándar
# # Se añade el directorio a la ruta de búsqueda del compilador de turno
INCLUDE_DIRECTORIES("include")

# Proyecto ejecutable con el archivo fuente VAR INCLUDE_FILES Y VAR SOURCE_FILES
add_executable(CirobbEngine ${INCLUDE_FILES} ${SOURCE_FILES})
