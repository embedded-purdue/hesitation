#pragma once
#include <string>
#include "nep_type.h"

// This file loads JSON config at startup. If missing file or out of bounds, it throws std::runtime error.

struct LoadedNEPSetup {
    WorkingFluid fluid;
    NEPSystemConfig config;
};

LoadedNEPSetup loadNEPConfig(const std::string& filepath);
