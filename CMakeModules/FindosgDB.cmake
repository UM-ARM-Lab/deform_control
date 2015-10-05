# find osgDB
#
# exports:
#
#   OSGDB_FOUND
#   OSGDB_INCLUDE_DIR
#   OSGDB_LIBRARY

INCLUDE(FindPackageHandleStandardArgs)

SET(OSGDB_IncludeSearchPaths
  ${CMAKE_SOURCE_DIR}/lib/OpenSceneGraph-2.8.5/include
)

SET(OSGDB_LibrarySearchPaths
  ${CMAKE_SOURCE_DIR}/lib/OpenSceneGraph-2.8.5/lib64
)

FIND_PATH(OSGDB_INCLUDE_DIR osgDB/Version
  PATHS ${OSGDB_IncludeSearchPaths}
  NO_DEFAULT_PATH
)

FIND_LIBRARY(OSGDB_LIBRARY
  NAMES osgDB
  PATHS ${OSGDB_LibrarySearchPaths}
  NO_DEFAULT_PATH
)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
FIND_PACKAGE_HANDLE_STANDARD_ARGS(osgDB "Could NOT find osgDB SDK"
  OSGDB_LIBRARY
  OSGDB_INCLUDE_DIR
)

IF(OSGDB_FOUND)
  FIND_PACKAGE_MESSAGE(osgDB_FOUND "Found osgDB: ${OSGDB_INCLUDE_DIR}" "[${OSGDB_LIBRARY}][${OSGDB_INCLUDE_DIR}]")
ELSE(OSGDB_FOUND)
  SET(OSGDB_INCLUDE_DIR "")
  SET(OSGDB_LIBRARY "")
ENDIF(OSGDB_FOUND)

MARK_AS_ADVANCED(
  OSGDB_INCLUDE_DIR
  OSGDB_LIBRARY
)

