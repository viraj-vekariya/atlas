CXX := c++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -g -Iinclude

SRC := src/graph.cpp src/dijkstra.cpp src/astar.cpp
BUILD := build

.PHONY: all cli test clean

all: cli test

cli: $(BUILD)/atlas_cli

test: $(BUILD)/atlas_test
	$(BUILD)/atlas_test

$(BUILD)/atlas_cli: $(SRC) src/main.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(SRC) src/main.cpp -o $@

$(BUILD)/atlas_test: $(SRC) tests/test_atlas.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(SRC) tests/test_atlas.cpp -o $@

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
