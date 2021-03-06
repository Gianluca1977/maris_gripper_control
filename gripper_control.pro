TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/tcp_interface.cpp \
    src/supervisor.cpp \
    src/state_machine.cpp \
    src/rt_thread.cpp \
    src/ros_interface.cpp \
    src/motor.cpp \
    src/main.cpp \
    src/gripper.cpp \
    src/event_data.cpp \
    src/controllerdata.cpp \
    src/controller.cpp \
    src/can_driver.cpp \
    src/motor_simulator.cpp \
    src/timer.cpp \
    src/motor_configurator.cpp \
    src/motorguard.cpp \
    src/shape_actionclient.cpp \
    src/simulator.cpp \
    src/motordriver.cpp \
    src/can_fpga.cpp

HEADERS += \
    src/can_define.h \
    src/tcp_interface.h \
    src/supervisor.h \
    src/state_machine.h \
    src/rt_thread.h \
    src/ros_interface.h \
    src/motor.h \
    src/main.h \
    src/interface_data.h \
    src/gripper.h \
    src/event_data.h \
    src/controllerdata.h \
    src/controller.h \
    src/can_driver.h \
    src/motor_simulator.h \
    src/timer.h \
    src/motor_configurator.h \
    src/motorguard.h \
    src/simulator.h \
    src/can_define.h \
    src/motordriver.h \
    src/can_fpga.h

OTHER_FILES += \
    TODO.txt

