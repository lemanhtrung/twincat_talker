cmake_minimum_required(VERSION 2.8.4)
project(adscxx CXX)

set(CMAKE_SUPPRESS_REGENERATION TRUE)

set(SOURCE
        AdsLib/AdsDef.cpp
        AdsLib/AdsLib.cpp
        AdsLib/AmsConnection.cpp
        AdsLib/AmsPort.cpp
        AdsLib/AmsRouter.cpp
        AdsLib/Frame.cpp
        AdsLib/Log.cpp
        AdsLib/NotificationDispatcher.cpp
        AdsLib/Sockets.cpp
        AdsLibOOI/AdsDevice.cpp
        AdsLibOOI/AdsNotification.cpp
        AdsLib/AdsDef.h
        AdsLib/AdsLib.h
        AdsLib/AdsNotification.h
        AdsLib/AmsConnection.h
        AdsLib/AmsHeader.h
        AdsLib/AmsPort.h
        AdsLib/AmsRouter.h
        AdsLib/Frame.h
        AdsLib/Log.h
        AdsLib/NotificationDispatcher.h
        AdsLib/RingBuffer.h
        AdsLib/Router.h
        AdsLib/Sockets.h
        AdsLib/wrap_endian.h
        AdsLib/wrap_socket.h
        AdsLibOOI/AdsDevice.h
        AdsLibOOI/AdsException.h
        AdsLibOOI/AdsNotificationOOI.h
        AdsLibOOI/AdsVariable.h)

add_library(adscxx ${SOURCE})

include_directories(
    AdsLib
    AdsLibOOI
)

install(TARGETS adscxx
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)