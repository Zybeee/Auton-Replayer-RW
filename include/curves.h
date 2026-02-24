#ifndef CURVES_H
#define CURVES_H

#include <string>

// ============================================================================
// DRIVE CURVES MODULE
// Provides multiple joystick response curves for driver control.
// Change ACTIVE_CURVE below to switch between curve types for testing.
// Press DOWN arrow during driving to cycle curves at runtime.
// ============================================================================

enum CurveType {
  CURVE_LINEAR,       // Direct 1:1 — no curve applied to this curve
  CURVE_EXPONENTIAL,  // Gentle low end, snappy high end (LemLib-style)
  CURVE_CUBIC,        // Popular in VEX — smooth and predictable
  CURVE_QUADRATIC,    // Softer than linear, less aggressive than cubic
  CURVE_SCURVE,       // Sigmoid — smooth S-shape response
  CURVE_SQUARED,      // x² — smooth low-end, popular default
  CURVE_PIECEWISE,    // Two-zone linear — simple and predictable
  CURVE_PLATEAU       // Wide mid-high plateau — easy to drive straight
};

// Total number of curve types (for cycling)
constexpr int CURVE_TYPE_COUNT = 8;

// ========================================================
// CHANGE THIS TO SELECT YOUR CURVE TYPE
// ========================================================
extern CurveType ACTIVE_CURVE;

// Shared parameters (apply to all curves)
extern double CURVE_DEADBAND;    // Joystick deadband (default: 3) - TEST THIS
extern double CURVE_MIN_OUTPUT;  // Minimum motor output past deadband (default: 10)
extern double CURVE_STRENGTH;    // Curve intensity — meaning varies per curve type:
                                 //   EXPONENTIAL: exponent steepness (default: 1.05)
                                 //   CUBIC:       blend factor 0.0=linear, 1.0=full cubic (default: 0.8)
                                 //   QUADRATIC:   unused (pure quadratic)
                                 //   SCURVE:      sigmoid steepness (default: 0.05)
                                 //   LINEAR:      unused
                                 //   SQUARED:     unused (pure squared)
extern double CURVE_PARAM;       // Extra tuning parameter:
                                 //   EXPONENTIAL: aggressiveness (default 1.5, range ~0.5-3.0)
                                 //   PIECEWISE:   breakpoint as fraction (default 0.3, range 0.1-0.5)

// Main entry point — applies the active curve to a joystick input [-127, 127]
// Returns curved output in [-127, 127] range
double applyCurve(double input);

// Get a human-readable name for the curve type (for brain/controller display)
std::string getCurveName(CurveType curve);

// Cycle to the next curve type (wraps around)
CurveType nextCurve(CurveType current);

#endif // CURVES_H
