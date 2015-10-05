# find osg
#
# exports:
#
#   OSG_FOUND
#   OSG_INCLUDE_DIR
#   OSG_LIBRARY

INCLUDE(FindPackageHandleStandardArgs)

SET(OSG_IncludeSearchPaths
  ${CMAKE_SOURCE_DIR}/lib/OpenSceneGraph-2.8.5/include
)

SET(OSG_LibrarySearchPaths
  ${CMAKE_SOURCE_DIR}/lib/OpenSceneGraph-2.8.5/lib64
)

FIND_PATH(OSG_INCLUDE_DIR osg/Version
  PATHS ${OSG_IncludeSearchPaths}
  NO_DEFAULT_PATH
)

FIND_LIBRARY(OSG_LIBRARY
  NAMES osg
  PATHS ${OSG_LibrarySearchPaths}
  NO_DEFAULT_PATH
)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
FIND_PACKAGE_HANDLE_STANDARD_ARGS(osg "Could NOT find osg SDK"
  OSG_LIBRARY
  OSG_INCLUDE_DIR
)

IF(OSG_FOUND)
  FIND_PACKAGE_MESSAGE(osg_FOUND "Found osg: ${OSG_INCLUDE_DIR}" "[${OSG_LIBRARY}][${OSG_INCLUDE_DIR}]")
ELSE(OSG_FOUND)
  SET(OSG_INCLUDE_DIR "")
  SET(OSG_LIBRARY "")
ENDIF(OSG_FOUND)

MARK_AS_ADVANCED(
  OSG_INCLUDE_DIR
  OSG_LIBRARY
)

