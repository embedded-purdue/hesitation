# ==============================================================================
# Makefile - Hesitation Software Unified Build Driver
#
# Provides convenient CLI shortcuts for CMake-based configuration, compilation,
# testing, and maintenance across all software submodules:
#   - star_tracking: Autonomous lost-in-space & AIM star tracker system
#   - td3_attitude:  Twin Delayed DDPG spacecraft attitude control (LibTorch)
#   - thrust_vectoring: Thruster geometry and torque allocation engine
# ==============================================================================

SHELL        := /bin/bash
BUILD_DIR    ?= build
BUILD_TYPE   ?= Release
JOBS         ?= $(shell nproc 2>/dev/null || echo 4)
CMAKE        := cmake
CTEST        := ctest

# Optional extra flags passed to CMake configure (e.g. EXTRA_FLAGS="-DCMAKE_CXX_COMPILER=clang++")
EXTRA_FLAGS  ?=

.DEFAULT_GOAL := help

##@ Build Commands

.PHONY: all build debug release

all: configure ## Build all software modules (default Release mode)
	@echo "==> Building all targets in $(BUILD_DIR) (jobs: $(JOBS))..."
	@$(CMAKE) --build $(BUILD_DIR) --parallel $(JOBS)
	@$(MAKE) --no-print-directory update-compile-commands

build: all ## Alias for 'all'

debug: ## Configure and build all modules with Debug symbols (-g, -O0)
	@$(MAKE) all BUILD_TYPE=Debug

release: ## Configure and build all modules in Release mode (-O3)
	@$(MAKE) all BUILD_TYPE=Release

##@ Configuration & Tooling

.PHONY: configure update-compile-commands

configure: $(BUILD_DIR)/CMakeCache.txt ## Run CMake configuration step

$(BUILD_DIR)/CMakeCache.txt: CMakeLists.txt
	@echo "==> Configuring build in $(BUILD_DIR) [$(BUILD_TYPE)]..."
	@mkdir -p $(BUILD_DIR)
	@$(CMAKE) -B $(BUILD_DIR) -S . \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
		$(EXTRA_FLAGS)
	@$(MAKE) --no-print-directory update-compile-commands

update-compile-commands: ## Symlink compile_commands.json to root for clangd/LSP
	@if [ -f "$(BUILD_DIR)/compile_commands.json" ]; then \
		ln -sf $(BUILD_DIR)/compile_commands.json compile_commands.json; \
	fi

##@ Subsystem Builds

.PHONY: star_tracking td3_attitude thrust_vectoring

star_tracking: configure ## Build only Star Tracking targets (lib, main, tools)
	@echo "==> Building Star Tracking targets..."
	@$(CMAKE) --build $(BUILD_DIR) --target star_tracker_lib star_tracker_main generate_database monte_carlo_validation --parallel $(JOBS)

td3_attitude: configure ## Build only TD3 Attitude Control targets (train, eval)
	@echo "==> Building TD3 Attitude targets..."
	@$(CMAKE) --build $(BUILD_DIR) --target train eval --parallel $(JOBS)

thrust_vectoring: configure ## Build only Thrust Vectoring targets
	@echo "==> Building Thrust Vectoring targets..."
	@$(CMAKE) --build $(BUILD_DIR) --target thrust_vectoring_main --parallel $(JOBS)

##@ Testing & Verification

.PHONY: test check test-star-tracking

test: configure ## Build all test targets and run CTest test suite
	@echo "==> Building test executables..."
	@$(CMAKE) --build $(BUILD_DIR) --parallel $(JOBS)
	@echo "==> Running full test suite..."
	@$(CTEST) --test-dir $(BUILD_DIR) --output-on-failure --parallel $(JOBS)

check: test ## Alias for 'test'

test-star-tracking: configure ## Run star tracking GoogleTest suite specifically
	@echo "==> Running Star Tracking unit tests..."
	@$(CTEST) --test-dir $(BUILD_DIR) -R "^test_" --output-on-failure --parallel $(JOBS)

##@ Execution Helpers

.PHONY: run-star-tracker run-thrust-vectoring run-eval

run-star-tracker: star_tracking ## Run star tracker simulation application
	@echo "==> Running star_tracker_main..."
	@./$(BUILD_DIR)/star_tracking/star_tracker_main

run-thrust-vectoring: thrust_vectoring ## Run thrust vectoring allocation demo
	@echo "==> Running thrust_vectoring_main..."
	@./$(BUILD_DIR)/thrust_vectoring/thrust_vectoring_main

run-eval: td3_attitude ## Run TD3 attitude policy greedy evaluation
	@echo "==> Running TD3 eval..."
	@./td3_attitude/bin/eval

##@ Clean & Maintenance

.PHONY: clean distclean

clean: ## Remove build objects while keeping CMake configuration
	@if [ -d "$(BUILD_DIR)" ]; then \
		echo "==> Cleaning build artifacts..."; \
		$(CMAKE) --build $(BUILD_DIR) --target clean; \
	fi

distclean: ## Purge build directory, compile_commands symlink, and submodule builds
	@echo "==> Purging build directory and generated files..."
	@rm -rf $(BUILD_DIR)
	@rm -f compile_commands.json
	@rm -rf star_tracking/build td3_attitude/build thrust_vectoring/build

##@ Help

.PHONY: help

help: ## Show this help menu with all targets and options
	@echo ""
	@echo "Hesitation Flight Software Build System"
	@echo "======================================="
	@echo "Usage: make [TARGET] [VARIABLE=VALUE ...]"
	@awk 'BEGIN {FS = ":.*##"; printf ""} \
		/^[a-zA-Z0-9_-]+:.*?##/ { printf "  \033[36m%-24s\033[0m %s\n", $$1, $$2 } \
		/^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) } ' $(MAKEFILE_LIST)
	@echo ""
	@echo -e "\033[1mConfigurable Variables:\033[0m"
	@echo "  BUILD_TYPE    Build configuration (Release, Debug). [Default: $(BUILD_TYPE)]"
	@echo "  BUILD_DIR     Output directory for CMake build tree. [Default: $(BUILD_DIR)]"
	@echo "  JOBS          Parallel compile job count. [Default: $(JOBS)]"
	@echo "  EXTRA_FLAGS   Additional CMake configuration flags."
	@echo ""
	@echo -e "\033[1mQuick Examples:\033[0m"
	@echo "  make                     # Show this help menu"
	@echo "  make all                 # Build all modules in Release mode"
	@echo "  make test                # Build and run all unit tests"
	@echo "  make star_tracking       # Build only star tracking"
	@echo "  make td3_attitude        # Build only TD3 attitude control"
	@echo "  make thrust_vectoring    # Build only thrust vectoring"
	@echo "  make run-thrust-vectoring# Build and execute thrust vectoring demo"
	@echo "  make clean               # Clean object files"
	@echo "  make distclean           # Completely remove build artifacts"
	@echo ""
