TEMPLATE = app

CONFIG += console c++17

CONFIG -= app_bundle

CONFIG += thread

CONFIG -= qt

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv \
                /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_*

SOURCES += \
    main.cpp

HEADERS += \
    serialib.hpp \
    authlib.hpp
