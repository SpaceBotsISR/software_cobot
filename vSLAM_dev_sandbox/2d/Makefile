# Compiler settings
CXX = clang++
CXXFLAGS = -std=c++17 -Wall -Iinclude -I/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3 -I/opt/homebrew/Cellar/tbb/2021.11.0/include -g
LDFLAGS = -L/usr/local/lib -lgtsam -L/opt/homebrew/Cellar/tbb/2021.11.0/lib -ltbb

# Define the source and object files
SRC_DIR = src
BUILD_DIR = build
SRC_FILES = $(wildcard $(SRC_DIR)/*.cpp)
OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRC_FILES))

# Executable name
EXEC = vslam

# Default target
all: $(EXEC)

$(EXEC): $(OBJ_FILES)
	$(CXX) $^ $(LDFLAGS) -o $@

# Rule for building object files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Create object file directory if it doesn't exist
$(OBJ_FILES): | $(BUILD_DIR)

$(BUILD_DIR):
	mkdir $(BUILD_DIR)

# Clean up
clean:
	rm -rf $(BUILD_DIR) $(EXEC)

# Phony targets
.PHONY: all clean
