
#add_subdirectory(libamf)

file(GLOB inc
	${CMAKE_CURRENT_SOURCE_DIR}
	LASlib/inc
	LASzip/src
    
	amf-cpp
	amf-cpp/types
  
	nanoflann
)

file(GLOB src
	LASlib/src/*.cpp
	LASzip/src/*.cpp
	
	civetweb/civetweb.c
	stb/stb_image.h
	stb/stb_image_write.h
	
	b64/cdecode.c
	b64/cencode.c
    
 	amf-cpp/*.cpp

	nanoflann/nanoflann.hpp
    
  7zip/*.c
  
	ujson/ujson.cpp
	ujson/double-conversion.cc
)

list(REMOVE_ITEM src
  ${CMAKE_CURRENT_SOURCE_DIR}/LASzip/src/laszip_dll.cpp
)

set(includes ${includes} ${inc} PARENT_SCOPE)
set(sources ${sources} ${src} PARENT_SCOPE)