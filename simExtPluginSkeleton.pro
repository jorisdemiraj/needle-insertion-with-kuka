QT -= core
QT -= gui

TARGET = simExtPluginSkeleton
TEMPLATE = lib

DEFINES -= UNICODE
DEFINES += QT_COMPIL
CONFIG += shared
INCLUDEPATH += "../include"
INCLUDEPATH += "../include/stack"

*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}
*-g++* {
    QMAKE_CXXFLAGS += -O3
    QMAKE_CXXFLAGS += -Wall
    QMAKE_CXXFLAGS += -Wno-unused-parameter
    QMAKE_CXXFLAGS += -Wno-strict-aliasing
    QMAKE_CXXFLAGS += -Wno-empty-body
    QMAKE_CXXFLAGS += -Wno-write-strings

    QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
    QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
    QMAKE_CXXFLAGS += -Wno-narrowing

    QMAKE_CXXFLAGS += -lboost_math_c99
    QMAKE_CXXFLAGS += -lboost_thread
    QMAKE_CXXFLAGS += -lm
    QMAKE_CXXFLAGS += -lCGAL
    QMAKE_CXXFLAGS += -lmpfr
    QMAKE_CXXFLAGS += -lgmp

    QMAKE_CFLAGS += -O3
    QMAKE_CFLAGS += -Wall
    QMAKE_CFLAGS += -Wno-strict-aliasing
    QMAKE_CFLAGS += -Wno-unused-parameter
    QMAKE_CFLAGS += -Wno-unused-but-set-variable
    QMAKE_CFLAGS += -Wno-unused-local-typedefs

    QMAKE_CFLAGS += -lboost_math_c99
    QMAKE_CFLAGS += -lboost_thread
    QMAKE_CFLAGS += -lm
    QMAKE_CFLAGS += -lCGAL
    QMAKE_CFLAGS += -lmpfr
    QMAKE_CFLAGS += -lgmp



}

win32 {
    DEFINES += WIN_SIM
}

macx {
    DEFINES += MAC_SIM
}

unix:!macx {
    DEFINES += LIN_SIM
}

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}

HEADERS += \
    simExtPluginSkeleton.h \
    ../include/stack/stackBool.h \
    ../include/stack/stackNull.h \
    ../include/stack/stackNumber.h \
    ../include/stack/stackString.h \
    ../include/stack/stackArray.h \
    ../include/stack/stackMap.h \
    ../include/stack/stackObject.h \
    ../include/simLib.h

SOURCES += \
    simExtPluginSkeleton.cpp \
    ../common/stack/stackBool.cpp \
    ../common/stack/stackNull.cpp \
    ../common/stack/stackNumber.cpp \
    ../common/stack/stackString.cpp \
    ../common/stack/stackArray.cpp \
    ../common/stack/stackMap.cpp \
    ../common/stack/stackObject.cpp \
    ../common/simLib.cpp



