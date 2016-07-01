QT += widgets

CONFIG += c++11

TARGET = awts

TEMPLATE = app

SOURCES += main.cpp \
    MainWindow.cpp \
    RoadGenerator.cpp \
    SettingsManager.cpp \
    RoadSegment.cpp \
    RandGen.cpp \
    Simulator.cpp \
    SimulatorView.cpp

HEADERS += \
    RoadGenerator.h \
    MainWindow.h \
    SettingsManager.h \
    RoadSegment.h \
    RandGen.h \
    Simulator.h \
    SimulatorView.h

DISTFILES += \
    images/car.png \
    README