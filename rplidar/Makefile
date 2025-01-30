HOME_TREE := rplidar_sdk
MODULE_NAME := rp_lidar
include $(HOME_TREE)/mak_def.inc
CXXSRC += main.cpp
C_INCLUDES += -I$(HOME_TREE)/sdk/include -I$(HOME_TREE)/sdk/src
EXTRA_OBJ :=
LD_LIBS += -lstdc++ -lpthread

all: build_app

include $(HOME_TREE)/mak_common.inc

clean: clean_app
