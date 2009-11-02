
# Find the location of Webots
IF ("x$ENV{WEBOTS_HOME}x" STREQUAL "xx")
    IF (${CMAKE_SYSTEM_NAME} STREQUAL Windows)
        # I am going to assume that Webots is in the default location
        IF (EXISTS "/Program Files/Webots")
            SET (ENV{WEBOTS_HOME} "/Program Files/Webots")
        ENDIF (EXISTS "/Program Files/Webots")
        
    ELSE (${CMAKE_SYSTEM_NAME} STREQUAL Windows)
        IF (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
            MESSAGE(STATUS "Looking for Webots on Darwin")

        ELSE (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
            IF (${CMAKE_SYSTEM_NAME} STREQUAL Linux)
                # There are only two logical places to put Webots on Linux 
                # /usr/local/webots or ~/webots
                
                IF (EXISTS /usr/local/webots)
                    SET (ENV{WEBOTS_HOME} /usr/local/webots)
                ENDIF (EXISTS /usr/local/webots)

                IF (EXISTS $ENV{HOME}/webots)
                    SET (ENV{WEBOTS_HOME} $ENV{HOME}/webots)
                ENDIF (EXISTS $ENV{HOME}/webots)

            ELSE (${CMAKE_SYSTEM_NAME} STREQUAL Linux)
                MESSAGE(ERROR "I didn't know Webots could run on ${CMAKE_SYSTEM_NAME}")
            ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL Linux)
        ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
    ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL Windows)
ENDIF ("x$ENV{WEBOTS_HOME}x" STREQUAL "xx")

## See if we were sucessful in finding webots
IF ("x$ENV{WEBOTS_HOME}x" STREQUAL "xx")
    MESSAGE(ERROR "Unable to find Webots. You will need to set ENV{WEBOTS_HOME}")
ELSE ("x$ENV{WEBOTS_HOME}x" STREQUAL "xx")
	SET(WEBOTS_FOUND TRUE)
ENDIF ("x$ENV{WEBOTS_HOME}x" STREQUAL "xx")

# set the directory and then includes
SET(WEBOTS_DIR $ENV{WEBOTS_HOME})
SET(WEBOTS_INCLUDE_DIR  "${WEBOTS_DIR}/include/controller/cpp"
                        "${WEBOTS_DIR}/include"
                        "${WEBOTS_DIR}/projects/contests/nao_robocup/controllers"
                        CACHE FILEPATH "Cleared." FORCE)

# now set the platform dependant library
IF (${CMAKE_SYSTEM_NAME} STREQUAL Windows)
    SET(WEBOTS_LIBRARIES "${WEBOTS_DIR}/lib/libCppController.dll" CACHE FILEPATH "Cleared." FORCE)

ELSE (${CMAKE_SYSTEM_NAME} STREQUAL Windows)
    IF (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
        SET(WEBOTS_LIBRARIES "${WEBOTS_DIR}/lib/libCppController.dylib" CACHE FILEPATH "Cleared." FORCE)

    ELSE (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
        SET(WEBOTS_LIBRARIES "${WEBOTS_DIR}/lib/libCppController.so" CACHE FILEPATH "Cleared." FORCE)

    ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL Windows)

# finally set the webots definition
SET(WEBOTS_DEFINITIONS -DTARGET_IS_WEBOTS)

MARK_AS_ADVANCED(
    WEBOTS_INCLUDE_DIR
    WEBOTS_LIBRARIES
)


