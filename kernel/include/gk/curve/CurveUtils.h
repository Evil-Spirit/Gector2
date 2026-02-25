#pragma once
#include "gk/curve/ICurve.h"
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace gk {
namespace CurveUtils {

inline double closestPoint(const ICurve3& curve, const Vec3& pt, int nInit = 32)
{
    Interval dom = curve.domain();
    double bestT = dom.lo;
    double bestD = (curve.evaluate(dom.lo).p - pt).squaredNorm();

    for (int i = 0; i <= nInit; ++i) {
        double t = dom.lo + dom.width() * (double(i) / double(nInit));
        double d = (curve.evaluate(t).p - pt).squaredNorm();
        if (d < bestD) { bestD = d; bestT = t; }
    }

    double t = bestT;
    for (int iter = 0; iter < 32; ++iter) {
        auto cp = curve.evaluate(t);
        Vec3 diff = cp.p - pt;
        double f  = diff.dot(cp.d1);
        double df = cp.d1.dot(cp.d1) + diff.dot(cp.d2);
        if (df == 0.0) break;
        double step = f / df;
        t -= step;
        t = std::max(dom.lo, std::min(dom.hi, t));
        if (std::abs(step) < 1e-12) break;
    }
    return t;
}

inline double closestPoint(const ICurve2& curve, const Vec2& pt, int nInit = 32)
{
    Interval dom = curve.domain();
    double bestT = dom.lo;
    double bestD = (curve.evaluate(dom.lo).p - pt).squaredNorm();

    for (int i = 0; i <= nInit; ++i) {
        double t = dom.lo + dom.width() * (double(i) / double(nInit));
        double d = (curve.evaluate(t).p - pt).squaredNorm();
        if (d < bestD) { bestD = d; bestT = t; }
    }

    double t = bestT;
    for (int iter = 0; iter < 32; ++iter) {
        auto cp = curve.evaluate(t);
        Vec2 diff = cp.p - pt;
        double f  = diff.dot(cp.d1);
        double df = cp.d1.dot(cp.d1) + diff.dot(cp.d2);
        if (df == 0.0) break;
        double step = f / df;
        t -= step;
        t = std::max(dom.lo, std::min(dom.hi, t));
        if (std::abs(step) < 1e-12) break;
    }
    return t;
}

inline std::vector<std::pair<double,double>>
arcLengthTable(const ICurve3& curve, int n = 128)
{
    Interval dom = curve.domain();
    std::vector<std::pair<double,double>> table;
    table.reserve(n + 1);
    double arcLen = 0.0;
    Vec3 prev = curve.evaluate(dom.lo).p;
    table.push_back({0.0, dom.lo});
    for (int i = 1; i <= n; ++i) {
        double t = dom.lo + dom.width() * (double(i) / double(n));
        Vec3 cur = curve.evaluate(t).p;
        arcLen += (cur - prev).norm();
        prev = cur;
        table.push_back({arcLen, t});
    }
    return table;
}

namespace detail_ci {

inline void newtonRefineIntersect2D(
    const ICurve2& c1, double& t1,
    const ICurve2& c2, double& t2,
    double tol, const Interval& dom1, const Interval& dom2)
{
    for (int iter = 0; iter < 32; ++iter) {
        auto cp1 = c1.evaluate(t1);
        auto cp2 = c2.evaluate(t2);
        Vec2 diff = cp1.p - cp2.p;
        if (diff.squaredNorm() < tol * tol) break;

        double a11 = cp1.d1.x, a12 = -cp2.d1.x;
        double a21 = cp1.d1.y, a22 = -cp2.d1.y;
        double det = a11*a22 - a12*a21;
        if (det == 0.0) break;
        double dt1 = (-diff.x * a22 + diff.y * a12) / det;
        double dt2 = (-diff.y * a11 + diff.x * a21) / det;
        t1 += dt1;
        t2 += dt2;
        t1 = std::max(dom1.lo, std::min(dom1.hi, t1));
        t2 = std::max(dom2.lo, std::min(dom2.hi, t2));
        if (std::abs(dt1) < 1e-13 && std::abs(dt2) < 1e-13) break;
    }
}

inline void subdivideIntersect2D(
    const ICurve2& c1, double t1lo, double t1hi,
    const ICurve2& c2, double t2lo, double t2hi,
    double tol, int maxDepth, int depth,
    std::vector<std::pair<double,double>>& results)
{
    Vec2 p1a = c1.evaluate(t1lo).p;
    Vec2 p1b = c1.evaluate(t1hi).p;
    Vec2 p2a = c2.evaluate(t2lo).p;
    Vec2 p2b = c2.evaluate(t2hi).p;

    double len1 = (p1b - p1a).norm();
    double len2 = (p2b - p2a).norm();

    double minX1 = std::min(p1a.x, p1b.x), maxX1 = std::max(p1a.x, p1b.x);
    double minY1 = std::min(p1a.y, p1b.y), maxY1 = std::max(p1a.y, p1b.y);
    double minX2 = std::min(p2a.x, p2b.x), maxX2 = std::max(p2a.x, p2b.x);
    double minY2 = std::min(p2a.y, p2b.y), maxY2 = std::max(p2a.y, p2b.y);

    double margin = tol * 10.0 + (len1 + len2) * 0.5;
    if (maxX1 + margin < minX2 || maxX2 + margin < minX1) return;
    if (maxY1 + margin < minY2 || maxY2 + margin < minY1) return;

    if (depth >= maxDepth || (len1 < tol && len2 < tol)) {
        double tt1 = (t1lo + t1hi) * 0.5;
        double tt2 = (t2lo + t2hi) * 0.5;
        Interval d1{t1lo, t1hi}, d2{t2lo, t2hi};
        newtonRefineIntersect2D(c1, tt1, c2, tt2, tol, d1, d2);
        auto cp1 = c1.evaluate(tt1);
        auto cp2 = c2.evaluate(tt2);
        if ((cp1.p - cp2.p).squaredNorm() < tol * tol * 100)
            results.push_back({tt1, tt2});
        return;
    }

    double t1mid = (t1lo + t1hi) * 0.5;
    double t2mid = (t2lo + t2hi) * 0.5;
    subdivideIntersect2D(c1, t1lo, t1mid, c2, t2lo, t2mid, tol, maxDepth, depth+1, results);
    subdivideIntersect2D(c1, t1lo, t1mid, c2, t2mid, t2hi, tol, maxDepth, depth+1, results);
    subdivideIntersect2D(c1, t1mid, t1hi, c2, t2lo, t2mid, tol, maxDepth, depth+1, results);
    subdivideIntersect2D(c1, t1mid, t1hi, c2, t2mid, t2hi, tol, maxDepth, depth+1, results);
}

} // namespace detail_ci

inline std::vector<std::pair<double,double>>
intersect2D(const ICurve2& c1, const ICurve2& c2,
            double tol = 1e-8, int maxDepth = 16)
{
    std::vector<std::pair<double,double>> results;
    auto dom1 = c1.domain();
    auto dom2 = c2.domain();
    detail_ci::subdivideIntersect2D(c1, dom1.lo, dom1.hi,
                                     c2, dom2.lo, dom2.hi,
                                     tol, maxDepth, 0, results);

    std::sort(results.begin(), results.end());
    auto last = std::unique(results.begin(), results.end(),
        [&](const std::pair<double,double>& a, const std::pair<double,double>& b) {
            return std::abs(a.first - b.first) < tol * 100 &&
                   std::abs(a.second - b.second) < tol * 100;
        });
    results.erase(last, results.end());
    return results;
}

} // namespace CurveUtils
} // namespace gk
