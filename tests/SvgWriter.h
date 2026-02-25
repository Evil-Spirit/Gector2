#pragma once
// Simple SVG writer used only by tests to produce visual debug output.

#include "gk/math/Vec2.h"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// Returns "svg_test_output/<filename>" and ensures the directory exists.
inline std::string svgOutputPath(const std::string& filename) {
    std::filesystem::create_directories("svg_test_output");
    return "svg_test_output/" + filename;
}

class SvgWriter {
public:
    SvgWriter(int width, int height)
        : width_(width), height_(height)
        , viewMinX_(0), viewMinY_(0)
        , viewMaxX_(double(width)), viewMaxY_(double(height))
    {}

    void setView(double minX, double minY, double maxX, double maxY) {
        viewMinX_ = minX; viewMinY_ = minY;
        viewMaxX_ = maxX; viewMaxY_ = maxY;
    }

    void addPolyline(const std::vector<gk::Vec2>& pts,
                     const std::string& stroke = "#000",
                     double strokeWidth = 1.5)
    {
        if (pts.size() < 2) return;
        std::ostringstream oss;
        oss << "<polyline points=\"";
        for (auto& p : pts) {
            auto sp = toSvg(p);
            oss << sp.first << "," << sp.second << " ";
        }
        oss << "\" fill=\"none\" stroke=\"" << stroke
            << "\" stroke-width=\"" << strokeWidth << "\"/>\n";
        elements_ += oss.str();
    }

    void addPoint(gk::Vec2 pt, const std::string& fill = "#f00", double r = 4.0) {
        auto sp = toSvg(pt);
        std::ostringstream oss;
        oss << "<circle cx=\"" << sp.first << "\" cy=\"" << sp.second
            << "\" r=\"" << r << "\" fill=\"" << fill << "\"/>\n";
        elements_ += oss.str();
    }

    void addTangent(gk::Vec2 pt, gk::Vec2 dir, double len,
                    const std::string& stroke = "#0a0") {
        if (dir.squaredNorm() < 1e-14) return;
        gk::Vec2 nd = dir.normalized();
        gk::Vec2 end = pt + nd * len;
        auto sp1 = toSvg(pt), sp2 = toSvg(end);
        std::ostringstream oss;
        oss << "<line x1=\"" << sp1.first << "\" y1=\"" << sp1.second
            << "\" x2=\"" << sp2.first << "\" y2=\"" << sp2.second
            << "\" stroke=\"" << stroke << "\" stroke-width=\"1.5\"/>\n";
        elements_ += oss.str();
    }

    void addLabel(gk::Vec2 pt, const std::string& text) {
        auto sp = toSvg(pt);
        std::ostringstream oss;
        oss << "<text x=\"" << sp.first << "\" y=\"" << sp.second
            << "\" font-size=\"12\">" << text << "</text>\n";
        elements_ += oss.str();
    }

    bool write(const std::string& path) {
        std::ofstream f(path);
        if (!f.is_open()) return false;
        f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        f << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width_
          << "\" height=\"" << height_ << "\">\n";
        f << "<rect width=\"" << width_ << "\" height=\"" << height_
          << "\" fill=\"white\"/>\n";
        f << elements_;
        f << "</svg>\n";
        return true;
    }

private:
    int width_, height_;
    double viewMinX_, viewMinY_, viewMaxX_, viewMaxY_;
    std::string elements_;

    std::pair<double,double> toSvg(gk::Vec2 p) const {
        double sx = (viewMaxX_ > viewMinX_)
            ? (p.x - viewMinX_) / (viewMaxX_ - viewMinX_) * double(width_) : 0.0;
        double sy = (viewMaxY_ > viewMinY_)
            ? (1.0 - (p.y - viewMinY_) / (viewMaxY_ - viewMinY_)) * double(height_) : 0.0;
        return {sx, sy};
    }
};
