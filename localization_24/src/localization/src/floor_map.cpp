#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <stdexcept>

// Point class for geometric calculations
class Point {
public:
    double x;
    double y;

    Point(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}
    
    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y);
    }
    
    Point operator-(const Point& other) const {
        return Point(x - other.x, y - other.y);
    }
    
    Point operator*(double f) const {
        return Point(x * f, y * f);
    }
};

// Shape classes for different geometric elements
struct Shape {
    double lineWidth = 0.0;
    
    virtual ~Shape() = default;
};

struct Line : public Shape {
    Point from;
    Point to;
};

struct Circle : public Shape {
    Point center;
    double radius;
};

struct Rectangle : public Shape {
    Point center;
    Point size;
};

struct Arc : public Shape {
    Point center;
    Point size;
    double angle;
    double startAngle;
    double endAngle;
};

// Standard field model parameters
struct FieldModel {
    double a = 12.0;  // Field length
    double b = 8.0;   // Field width
    double c = 3.0;   // Penalty area length
    double d = 2.0;   // Goal area length
    double e = 1.0;   // Penalty area width
    double f = 0.5;   // Goal area width
    double g = 0.75;  // Corner arc radius
    double h = 1.5;   // Center circle diameter
    double i = 2.0;   // Penalty mark distance
    double j = 0.1;   // Center spot radius
    double k = 0.1;   // Line width
};

class FloorMap {
public:
    FloorMap() = default;
    
    void configure(double pixelsPerMeter, double borderSize) {
        _ppm = pixelsPerMeter;
        _borderSize = borderSize;
        
        // Calculate floor dimensions
        _sizeX = _model.b + 2.0 * _borderSize;
        _sizeY = _model.a + 2.0 * _borderSize;
        
        // Set origin (center of the field)
        _originX = -0.5 * _sizeX;
        _originY = -0.5 * _sizeY;
        
        // Calculate image dimensions
        _numPixelsY = int(_sizeY * _ppm);
        _numPixelsX = int(_sizeX * _ppm);
    }
    
    void setModel(const FieldModel& model) {
        _model = model;
    }
    
    cv::Mat createFloorMap() {
        if (_numPixelsX * _numPixelsY == 0) {
            throw std::runtime_error("Invalid floor map dimensions");
        }
        
        cv::Mat mat = cv::Mat::zeros(_numPixelsX, _numPixelsY, CV_8UC1);
        std::vector<Shape*> shapes;
        createShapes(shapes);
        drawShapes(shapes, mat);
        
        // Cleanup
        for (auto shape : shapes) {
            delete shape;
        }
        
        return mat;
    }
    
    cv::Mat applyBlur(const cv::Mat& image, float blurFactor = 0.7, int blurMaxDepth = 10, uchar blurMinValue = 30) const {
        int nx = image.cols;
        int ny = image.rows;
        
        // Create list of white pixels
        std::vector<cv::Point> whitePixels;
        for (int x = 1; x < nx-1; x++) {
            for (int y = 1; y < ny-1; y++) {
                if (image.at<uchar>(y, x) > 0) {
                    whitePixels.push_back(cv::Point(x, y));
                }
            }
        }
        
        cv::Mat imageOut = image.clone();
        bool done = false;
        int iteration = 0;
        
        while (!done && iteration < blurMaxDepth) {
            done = (0 == blurSinglePass(imageOut, whitePixels, blurFactor, blurMinValue));
            iteration++;
        }
        
        return imageOut;
    }

private:
    FieldModel _model;
    double _ppm = 100;  // pixels per meter
    double _borderSize = 1.0;
    double _sizeX = 0.0;
    double _sizeY = 0.0;
    double _originX = 0.0;
    double _originY = 0.0;
    int _numPixelsX = 0;
    int _numPixelsY = 0;
    
    cv::Point pointToPixel(const Point& p) const {
        return cv::Point((p.y - _originY) * _ppm, (p.x - _originX) * _ppm);
    }
    
    void createShapes(std::vector<Shape*>& shapes) {
        // Field boundary
        auto* rect = new Rectangle();
        rect->lineWidth = _model.k;
        rect->size = Point(_model.b - _model.k, _model.a - _model.k);
        rect->center = Point(0, 0);
        shapes.push_back(rect);
        
        // Middle line
        auto* line = new Line();
        line->lineWidth = _model.k;
        line->from = Point(-0.5 * (_model.b - _model.k), 0);
        line->to = Point(0.5 * (_model.b - _model.k), 0);
        shapes.push_back(line);
        
        // Center circle
        if (_model.h > 0.1) {
            auto* circle = new Circle();
            circle->lineWidth = _model.k;
            circle->center = Point(0, 0);
            circle->radius = 0.5 * (_model.h - _model.k);
            shapes.push_back(circle);
        }
        
        // Penalty areas
        for (int sign = -1; sign <= 1; sign += 2) {
            if (_model.c > 0.0 && _model.e > 0.0) {
                auto* penaltyArea = new Rectangle();
                penaltyArea->lineWidth = _model.k;
                penaltyArea->center = Point(0, sign * 0.5 * (_model.a - _model.e));
                penaltyArea->size = Point(_model.c - _model.k, _model.e - _model.k);
                shapes.push_back(penaltyArea);
            }
            
            // Goal areas
            if (_model.d > 0.0 && _model.f > 0.0) {
                auto* goalArea = new Rectangle();
                goalArea->lineWidth = _model.k;
                goalArea->center = Point(0, sign * 0.5 * (_model.a - _model.f));
                goalArea->size = Point(_model.d - _model.k, _model.f - _model.k);
                shapes.push_back(goalArea);
            }
        }
        
        // Corner arcs
        if (_model.g > 0.0) {
            float deg2rad = M_PI / 180.0;
            for (int signX = -1; signX <= 1; signX += 2) {
                for (int signY = -1; signY <= 1; signY += 2) {
                    auto* arc = new Arc();
                    arc->lineWidth = _model.k;
                    arc->center = Point(signX * 0.5 * _model.b, signY * 0.5 * _model.a);
                    arc->size = Point(_model.g - 0.5 * _model.k, _model.g - 0.5 * _model.k);
                    float a_quadrant = deg2rad * (signX>0 ? (signY>0 ? 180 : 270) : (signY>0 ? 90 : 0));
                    arc->angle = a_quadrant;
                    arc->startAngle = 5 * deg2rad;
                    arc->endAngle = 85 * deg2rad;
                    shapes.push_back(arc);
                }
            }
        }
        
        // Center and penalty spots
        if (_model.j > 0.0) {
            auto* centerSpot = new Circle();
            centerSpot->lineWidth = 2.0 * _model.j;
            centerSpot->center = Point(0, 0);
            centerSpot->radius = 0;
            shapes.push_back(centerSpot);
        }
        
        if (_model.i > 0.0) {
            for (int signY = -1; signY <= 1; signY += 2) {
                auto* penaltySpot = new Circle();
                penaltySpot->lineWidth = 2.0 * _model.j;
                penaltySpot->center = Point(0, signY * (0.5 * _model.a - _model.i));
                penaltySpot->radius = 0;
                shapes.push_back(penaltySpot);
            }
        }
    }
    
    void drawShapes(const std::vector<Shape*>& shapes, cv::Mat& mat) const {
        cv::Scalar color(255, 255, 255);  // white
        
        for (const auto* shape : shapes) {
            if (auto* line = dynamic_cast<const Line*>(shape)) {
                int lw = line->lineWidth * _ppm;
                auto pfrom = pointToPixel(line->from);
                auto pto = pointToPixel(line->to);
                cv::line(mat, pfrom, pto, color, lw);
            }
            else if (auto* circle = dynamic_cast<const Circle*>(shape)) {
                int lw = circle->lineWidth * _ppm;
                int radius = circle->radius * _ppm;
                auto p = pointToPixel(circle->center);
                cv::circle(mat, p, radius, color, lw);
            }
            else if (auto* arc = dynamic_cast<const Arc*>(shape)) {
                int lw = arc->lineWidth * _ppm;
                float rad2deg = 180.0 / M_PI;
                float a = arc->angle * rad2deg;
                float as = arc->startAngle * rad2deg;
                float ae = arc->endAngle * rad2deg;
                auto p = pointToPixel(arc->center);
                cv::Size2f sz(arc->size.x * _ppm, arc->size.y * _ppm);
                cv::ellipse(mat, p, sz, a, as, ae, color, lw);
            }
            else if (auto* rect = dynamic_cast<const Rectangle*>(shape)) {
                int lw = rect->lineWidth * _ppm;
                auto p1 = pointToPixel(rect->center - rect->size * 0.5);
                auto p2 = pointToPixel(rect->center + rect->size * 0.5);
                cv::rectangle(mat, p1, p2, color, lw);
            }
        }
    }
    
    int blurSinglePass(cv::Mat& image, std::vector<cv::Point>& whitePixels, float blurFactor, uchar blurMinValue) const {
        int nx = image.cols;
        int ny = image.rows;
        int numCurrent = whitePixels.size();
        int numAdded = 0;
        
        std::vector<cv::Point> whitePixelsCurrent = whitePixels;
        for (const cv::Point& whitePixel : whitePixelsCurrent) {
            int x = whitePixel.x;
            int y = whitePixel.y;
            
            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    int neighborX = x + i;
                    int neighborY = y + j;
                    
                    if (image.at<uchar>(neighborY, neighborX) == 0) {
                        uchar newValue = static_cast<uchar>(blurFactor * static_cast<float>(image.at<uchar>(y, x)));
                        if (newValue > blurMinValue) {
                            if (neighborX > 0 && neighborX < nx-1 && neighborY > 0 && neighborY < ny-1) {
                                image.at<uchar>(neighborY, neighborX) = newValue;
                                whitePixels.push_back(cv::Point(neighborX, neighborY));
                                ++numAdded;
                            }
                        }
                    }
                }
            }
        }
        
        whitePixels.erase(whitePixels.begin(), whitePixels.begin() + numCurrent);
        return numAdded;
    }
};

// Main function showing usage
int main() {
    try {
        FloorMap floorMap;
        
        // Configure the floor map
        floorMap.configure(100.0, 1.0);  // 100 pixels per meter, 1m border
        
        // Create and customize field model if needed
        FieldModel model;
        // model.a = 12.0;  // Uncomment to modify field parameters
        // model.b = 8.0;
        floorMap.setModel(model);
        
        // Generate the floor map
        cv::Mat map = floorMap.createFloorMap();
        
        // Apply blur effect
        cv::Mat blurredMap = floorMap.applyBlur(map);
        
        // Display the maps
        cv::imshow("Original Floor Map", map);
        cv::imshow("Blurred Floor Map", blurredMap);
        cv::waitKey(0);
        
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}