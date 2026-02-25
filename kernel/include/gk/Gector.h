#pragma once
// GectorKernel — top-level convenience header.

// ── Iteration 1.2: Core math types ───────────────────────────────────────────
#include "gk/math/Vec2.h"
#include "gk/math/Vec3.h"
#include "gk/math/Vec4.h"
#include "gk/math/Mat2x2.h"
#include "gk/math/Mat3x3.h"
#include "gk/math/Mat4x4.h"
#include "gk/math/Quaternion.h"
#include "gk/math/Interval.h"

// ── Iteration 1.3: Tolerance & Precision Model ───────────────────────────────
#include "gk/Tolerances.h"

// ── Iteration 1.4: Memory & Handle Model ─────────────────────────────────────
#include "gk/Handle.h"
#include "gk/Arena.h"
#include "gk/EntityId.h"

// ── Iteration 1.5: Error Handling & Diagnostics ──────────────────────────────
#include "gk/Error.h"
#include "gk/Result.h"
#include "gk/Log.h"

// ── Chapter 3: Parametric Surfaces ───────────────────────────────────────────
#include "gk/surface/Surfaces.h"

// ── Chapter 2: Parametric Curves ─────────────────────────────────────────────
#include "gk/curve/Curves.h"

// ── Chapter 4: Topological Data Model ────────────────────────────────────────
#include "gk/brep/BRep.h"

// ── Chapter 5: Primitive Solid Builders + Sketch ─────────────────────────────
#include "gk/builders/Builders.h"
