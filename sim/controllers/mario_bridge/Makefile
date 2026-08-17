CXX_SOURCES = mario_bridge.cpp

# libzmq's C API only -- cppzmq is not needed here, which keeps this controller
# buildable without the project's nix dependency tree.
INCLUDE   = $(shell pkg-config --cflags libzmq)
LIBRARIES = $(shell pkg-config --libs libzmq) -lutil

CFLAGS  = -O2
CXXFLAGS = -O2 -std=c++17

WEBOTS_HOME ?= /usr/local/webots
include $(WEBOTS_HOME)/resources/Makefile.include
