# Standalone sim tools. They need only libzmq + OpenCV (+ taskflow headers for
# fsm_drive), so they build without the project's nix dependency tree.
#
# taskflow is header-only. Inside `nix develop` it is already on the include
# path; outside, point at a checkout:
#   make TASKFLOW_INC=/path/to/taskflow
TASKFLOW_INC ?=

CXXFLAGS := -O2 -std=c++20 -Wall $(shell pkg-config --cflags libzmq opencv4)
LDLIBS   := $(shell pkg-config --libs libzmq opencv4)

FSM_CXXFLAGS := $(CXXFLAGS) -I../../include $(if $(TASKFLOW_INC),-I$(TASKFLOW_INC),)

all: bridge_check fsm_drive

bridge_check: bridge_check.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

fsm_drive: fsm_drive.cpp ../../include/fsm.hpp
	$(CXX) $(FSM_CXXFLAGS) -o $@ $< $(LDLIBS) -lpthread

clean:
	rm -f bridge_check fsm_drive

.PHONY: all clean
