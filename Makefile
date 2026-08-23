CXX := c++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -g -Iinclude

SRC := src/graph.cpp src/dijkstra.cpp
BUILD := build

.PHONY: all test clean

all: test

test: $(BUILD)/atlas_test
	$(BUILD)/atlas_test

$(BUILD)/atlas_test: $(SRC) tests/test_atlas.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(SRC) tests/test_atlas.cpp -o $@

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
