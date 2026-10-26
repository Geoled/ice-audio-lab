// core/config.h — JSON -> EngineConfig
#pragma once
#include "engine.h"
#include <string>

namespace ice {

// Loads an engine JSON (v1 schema subset used by baseline_i4.json).
// Returns empty optional on failure (err filled).
bool loadEngineJson(const std::string &path, EngineConfig &out, std::string &err);

// k_carb / k_28inH2O style calibration helpers (flow coefficients)
double kCarb(double scfm);
double k28inH2O(double scfm);

} // namespace ice
