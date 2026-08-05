# Standalone bridge validator. Needs only libzmq + OpenCV, so it builds
# without the project's nix dependency tree.
CXXFLAGS := -O2 -std=c++17 -Wall $(shell pkg-config --cflags libzmq opencv4)
LDLIBS   := $(shell pkg-config --libs libzmq opencv4)

bridge_check: bridge_check.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f bridge_check

.PHONY: clean
