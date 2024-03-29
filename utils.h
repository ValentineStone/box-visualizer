#ifndef UTILS_H
#define UTILS_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <mutex>
#include <limits>
#include <QString>
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <QDebug>

#ifndef PI
#define PI  3.14159265358979323846
#define PI_FL  3.141592f
#endif

template <typename Point>
Point rotate(const Point& point, float a, float b, float c) {
    Point rotated;
    float x = point.x;
    float y = point.y;
    float z = point.z;

    float sinA = sin(a);
    float cosA = cos(a);
    float sinB = sin(b);
    float cosB = cos(b);
    float sinC = sin(c);
    float cosC = cos(c);

    rotated.x = x*cosB*cosC - y*sinC*cosB + z*sinB;
    rotated.y = x*(sinA*sinB*cosC+sinC*cosA) + y*(-sinA*sinB*sinC+cosA*cosC) - z*sinA*cosB;
    rotated.z = x*(sinA*sinC-sinB*cosA*cosC) + y*(sinA*cosC+sinB*sinC*cosA) + z*cosA*cosB;

    return rotated;
}

template <typename Point>
Point rotate(const Point& point, float a, float b, float c, const Point& around) {
    Point rotated = point;
    rotated.x -= around.x;
    rotated.y -= around.y;
    rotated.z -= around.z;
    rotated = rotate(rotated, a, b, c);
    rotated.x += around.x;
    rotated.y += around.y;
    rotated.z += around.z;
    return rotated;
}


template<typename Point>
QString point_toString(Point p) {
    return QString::number(p.x,'f',4) + ", " + QString::number(p.y,'f',4) + ", " + QString::number(p.z,'f',4);
}


template <typename Uint>
Uint to_grayscale(float val, float min, float max) {
    if (val == 0) return 0;
    static const Uint max_color = std::numeric_limits<Uint>::max();
    float dv = max - min;
    if (val < min) return max_color;
    if (val > max) return 0;
    return max_color - (val - min) / dv * max_color;
}

struct __attribute__ ((packed)) rgb_t {
    uint8_t r = 0, b = 0, g = 0;

    rgb_t& operator=(uint8_t num) {
        r = g = b = num;
        return *this;
    }
    template<typename UNumber>
    rgb_t& operator=(UNumber num) {
        r = g = b = num / (float)std::numeric_limits<UNumber>::max() * 255;
        return *this;
    }
    operator const uint8_t() const {
        return (r + g + b) / 3;
    }
    operator const QString() const {
        return QString::number(r) + ',' + QString::number(g) + ',' + QString::number(b);
    }
};

template <typename Pixel, typename Color>
void draw_line(Pixel* canvas, size_t width, size_t height, Color color, int x1, int y1, int x2, int y2) {
    int dx = x1 < x2 ? 1 : -1;
    int nx = std::abs(x1 - x2);
    int dy = y1 < y2 ? 1 : -1;
    int ny = std::abs(y1 - y2);

    if (nx > ny) {
        float fdy = dy * ny / (float)nx;
        for (int ix = 0; ix < nx; ix++) {
            int x = x1 + dx * ix;
            int y = std::round(y1 + fdy * ix);
            if (x < 0 || x >= width) continue;
            if (y < 0 || y >= height) continue;
            auto i = y * width + x;
            canvas[i] = color;
        }
    } else {
        float fdx = dx * nx / (float)ny;
        for (int iy = 0; iy < ny; iy++) {
            int y = y1 + dy * iy;
            int x = std::round(x1 + fdx * iy);
            if (x < 0 || x >= width) continue;
            if (y < 0 || y >= height) continue;
            auto i = y * width + x;
            canvas[i] = color;
        }
    }
}

template <typename Float>
float angle_between(Float ax, Float ay, Float bx, Float by) {
    float deltaY = abs(by - ay);
    float deltaX = abs(bx - ax);
    return atan2(ay-by, ax-bx);
}

template <typename Point>
Point angle(const Point& from, const Point& to) {
    Point vector;
    vector.x = to.x - from.x;
    vector.y = to.y - from.y;
    vector.z = to.z - from.z;

    float angle_to_x = std::atan2(vector.y,vector.x);
    float angle_to_z = std::atan2(std::sqrt(vector.x * vector.x + vector.y * vector.y), vector.z);

    Point rotation;
    rotation.x = angle_to_x;
    rotation.y = 0;
    rotation.z = angle_to_z;

    qDebug() << point_toString(vector) << point_toString(rotation);

    return rotation;
}

template<typename Point = rs2::vertex>
void pixel_to_point(Point& point, const rs2_intrinsics* intrin, size_t pixel_x, size_t pixel_y, float depth) {
    assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
    assert(intrin->model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image
    //assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model
    float x = (pixel_x - intrin->ppx) / intrin->fx;
    float y = (pixel_y - intrin->ppy) / intrin->fy;
    if(intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY) {
        float r2  = x*x + y*y;
        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        float ux = x*f + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        float uy = y*f + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = ux;
        y = uy;
    }
    point.x = depth * x;
    point.y = depth * y;
    point.z = depth;
}

template<typename Point = rs2::vertex, typename Color = rgb_t>
struct shape_t {
    Point* points;
    Color* colors;
    Color line_color{std::numeric_limits<uint8_t>::max()};
    std::vector<Point> lines;
    size_t width;
    size_t height;
    size_t size;

    shape_t(size_t _width, size_t _height, Point* _points = nullptr):
        width(_width), height(_height)
    {
        size = width * height;
        points = new Point[size];
        colors = new Color[size];
        memset(points, 0, size * sizeof(Point));
        memset(colors, 0, size * sizeof(Color));
        if (_points)
            memcpy(points, _points, size * sizeof(Point));
    }

    shape_t(size_t _width, size_t _height, uint16_t* depths, const rs2_intrinsics& intr, float depth_units):
        shape_t(_width, _height, nullptr)
    {
        for (size_t y = 0; y < height; y++) {
            for (size_t x = 0; x < width; x++) {
                auto i = y * width + x;
                pixel_to_point(points[i], &intr, x, y, depths[i] * depth_units);
            }
        }
    }

    shape_t(shape_t& other): shape_t(other.width, other.height, other.points) {}

    ~shape_t() {
        delete[] points;
        delete[] colors;
    }

    shape_t& rotate(float a, float b, float c) {
        float sinA = sin(a);
        float cosA = cos(a);
        float sinB = sin(b);
        float cosB = cos(b);
        float sinC = sin(c);
        float cosC = cos(c);
        for (size_t i = 0; i < size; i++) {
            float x = points[i].x;
            float y = points[i].y;
            float z = points[i].z;
            points[i].x = x*cosB*cosC - y*sinC*cosB + z*sinB;
            points[i].y = x*(sinA*sinB*cosC+sinC*cosA) + y*(-sinA*sinB*sinC+cosA*cosC) - z*sinA*cosB;
            points[i].z = x*(sinA*sinC-sinB*cosA*cosC) + y*(sinA*cosC+sinB*sinC*cosA) + z*cosA*cosB;
        }
        return *this;
    }

    shape_t& rotate(float a, float b, float c, const Point& around) {
        float sinA = sin(a);
        float cosA = cos(a);
        float sinB = sin(b);
        float cosB = cos(b);
        float sinC = sin(c);
        float cosC = cos(c);
        for (size_t i = 0; i < size; i++) {
            float x = points[i].x -= around.x;
            float y = points[i].y -= around.y;
            float z = points[i].z -= around.z;
            points[i].x = x*cosB*cosC - y*sinC*cosB + z*sinB + around.x;
            points[i].y = x*(sinA*sinB*cosC+sinC*cosA) + y*(-sinA*sinB*sinC+cosA*cosC) - z*sinA*cosB + around.y;
            points[i].z = x*(sinA*sinC-sinB*cosA*cosC) + y*(sinA*cosC+sinB*sinC*cosA) + z*cosA*cosB + around.z;
        }
        return *this;
    }

    shape_t&  colorize(float z_min, float z_max) {
        for (size_t i = 0; i < size; i++)
            colors[i] = to_grayscale<uint8_t>(points[i].z, z_min, z_max);
        return *this;
    }

    template<typename Pixel = rgb_t>
    uint8_t* render(int scale = 1) const {
        Pixel* pixels = (Pixel*)new uint8_t[size * sizeof(Pixel)];
        memset(pixels, 0, size * sizeof(Pixel));
        for (size_t i = 0; i < size; i++) {
            auto& point = points[i];
            size_t x = point.x * scale * width/2 + width/2;
            if (x < 0 || x >= width) continue;
            size_t y = point.y * scale * width/2 + height/2;
            if (y < 0 || y >= height) continue;
            Pixel& pixel = pixels[y * width + x];
            if (colors[i]) pixel = colors[i];
        }
        for (auto& vertex1 : lines) {
            for (auto& vertex2 : lines) {
                if (&vertex1 == &vertex2) continue;
                size_t x1 = vertex1.x * scale * width/2 + width/2;
                size_t y1 = vertex1.y * scale * width/2 + height/2;
                size_t x2 = vertex2.x * scale * width/2 + width/2;
                size_t y2 = vertex2.y * scale * width/2 + height/2;
                draw_line(pixels, width, height, line_color, x1, y1, x2, y2);
            }
        }
        return (uint8_t*)pixels;
    }

    Point center_mass(float z_min = std::numeric_limits<float>::min(), float z_max = std::numeric_limits<float>::max()) {
        Point center_mass{0,0,0};
        size_t center_mass_cnt = 0;
        for (size_t fy = 0; fy < height; fy++) {
            for (size_t fx = 0; fx < width; fx++) {
                auto i = fy * width + fx;
                if (points[i].z == 0 || points[i].z < z_min || points[i].z > z_max) continue;
                center_mass.z += points[i].z;
                center_mass_cnt++;
            }
        }
        center_mass.z /= center_mass_cnt;
        return center_mass;
    }

    Point center_mass(size_t x_min, size_t x_max, size_t y_min, size_t y_max, float z_min = std::numeric_limits<float>::min(), float z_max = std::numeric_limits<float>::max()) {
        Point center_mass{0,0,0};
        size_t center_mass_cnt = 0;
        for (size_t fy = y_min; fy < y_max && fy < height; fy++) {
            for (size_t fx = x_min; fx < x_max && fx < width; fx++) {
                auto i = fy * width + fx;
                if (points[i].z == 0 || points[i].z < z_min || points[i].z > z_max) continue;
                center_mass.x += points[i].x;
                center_mass.y += points[i].y;
                center_mass.z += points[i].z;
                center_mass_cnt++;
            }
        }
        center_mass.x /= center_mass_cnt;
        center_mass.y /= center_mass_cnt;
        center_mass.z /= center_mass_cnt;
        return center_mass;
    }
};


class WorkerThread : public QThread
{
    Q_OBJECT


public:
    WorkerThread(QObject* parent = nullptr):  QThread(parent) {}
    bool paused = false;
    float rotateA = 0;
    float rotateB = 0;
    float rotateC = 0;
    float zoomScale = 300;
    float sliceLevel = 0;
    float boxHeight = 0.4;

    float sliceEpsilon = 0.1;
    float guaranteed_floor = 0.9;
signals:
    void drawDepth(uint16_t* data, size_t width, size_t height, int rotate = 0);
    void drawColor(uint8_t* data, size_t width, size_t height, int rotate = 0);
    void drawRgb(QString, uint8_t* data, size_t width, size_t height, int rotate = 0);
    void statusMessage(QString);
    void debugText(QString);

private:
    std::mutex wait_for_frames_mutex;

    void renderRgb(QString name, const shape_t<>& shape, int rotate = 0) {
        emit drawRgb(name, shape.render(), shape.width, shape.height, rotate);
    }

    void run() override {
        rs2::config rs_config;

        if (true) {
            //rs_config.enable_stream(RS2_STREAM_COLOR, 320, 240, RS2_FORMAT_RGB8, 30); // 1920, 1080
            rs_config.enable_stream(RS2_STREAM_DEPTH, 320, 240, RS2_FORMAT_Z16, 30);   // 1024, 768
            //rs_config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
            //rs_config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        } else {
            //rs_config.enable_device_from_file("/home/v/flyover.bag");
            //rs_config.enable_device_from_file("/home/v/boxes.bag");
            //rs_config.enable_device_from_file("/home/v/Documents/20240229_082253.bag");
            //rs_config.enable_device_from_file("/home/v/Downloads/test4.bag");
        }

        rs2::pipeline rs_pipeline;
        rs2::pipeline_profile rs_profile = rs_pipeline.start(rs_config);

        qDebug() << "started!";

        while (true) {
            if (paused) continue;

            std::unique_lock<std::mutex> lock(wait_for_frames_mutex);
            rs2::frameset frameset = rs_pipeline.wait_for_frames();

            for (const rs2::frame& frame: frameset) {
                auto stream_type = frame.get_profile().stream_type();
                auto format = frame.get_profile().format();

                if (stream_type == RS2_STREAM_DEPTH) {
                    auto depth = frame.as<rs2::depth_frame>();
                    auto intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
                    auto sensor = rs2::sensor_from_frame(depth);
                    auto depth_units = sensor->get_option(RS2_OPTION_DEPTH_UNITS);
                    auto frame_width = depth.get_width();
                    auto frame_height = depth.get_height();
                    auto frame_size = frame_width * frame_height;
                    auto depth_data = (uint16_t*)depth.get_data();
                    float depth_min = 0.5;
                    float depth_max = 5;



                    float rotation1 = 0;
                    float rotation2 = 0;
                    float rot1 = 0;
                    float rot2 = 0;

                    auto depth_copy = new uint16_t[frame_size];
                    memcpy(depth_copy, depth_data, frame_size * sizeof(uint16_t));
                    for (size_t i = 0; i < frame_size; i++)
                        depth_copy[i] = to_grayscale<uint16_t>(depth_data[i] * depth_units, depth_min, depth_max);
                    emit drawDepth(depth_copy, frame_width, frame_height, 90);

                    shape_t shape_original(frame_width, frame_height, depth_data, intr, depth_units);
                    shape_original.colorize(depth_min, depth_max);

                    auto center_mass = shape_original.center_mass(depth_min, depth_max);
                    {
                    auto avg_bot_right = shape_original.center_mass(
                        frame_width*(guaranteed_floor + (1-guaranteed_floor)/2),
                        frame_width,
                        0,
                        frame_height / 2,
                        depth_min,
                        depth_max
                    );

                    auto avg_bot_left = shape_original.center_mass(
                        frame_width*(guaranteed_floor + (1-guaranteed_floor)/2),
                        frame_width,
                        frame_height / 2,
                        frame_height,
                        depth_min,
                        depth_max
                        );

                    auto avg_top_center = shape_original.center_mass(
                        frame_width * guaranteed_floor,
                        frame_width * (guaranteed_floor + (1 - guaranteed_floor) / 2),
                        0,
                        frame_height,
                        depth_min,
                        depth_max
                    );


                    rs2::vertex avg_bot_center;
                    avg_bot_center.x = (avg_bot_left.x + avg_bot_right.x) / 2;
                    avg_bot_center.y = (avg_bot_left.y + avg_bot_right.y) / 2;
                    avg_bot_center.z = (avg_bot_left.z + avg_bot_right.z) / 2;

                    shape_original.lines.push_back(avg_bot_right);
                    shape_original.lines.push_back(avg_bot_left);
                    shape_original.lines.push_back(avg_top_center);
                    shape_original.lines.push_back(avg_bot_center);
                    shape_original.line_color = rgb_t{255,0,0};

                    renderRgb("shape_original", shape_original);




                    rot1 = angle_between(avg_top_center.z, avg_top_center.x, avg_bot_center.z, avg_bot_center.x);
                    auto avg_bot_right_2 = rotate(avg_bot_right, 0, -rot1, 0,center_mass);
                    auto avg_bot_left_2 = rotate(avg_bot_left, 0, -rot1, 0,center_mass);
                    rot2 = PI - angle_between(avg_bot_right_2.y, avg_bot_right_2.x, avg_bot_left_2.y, avg_bot_left_2.x);



                    shape_t shape_floored2(shape_original);
                    shape_floored2.rotate(0,-rot1,-rot2,center_mass).rotate(0,0,0,center_mass).rotate(rotateA, rotateB, rotateC, center_mass).colorize(depth_min, depth_max);
                    renderRgb("shape_floored22", shape_floored2);

                    }

                    shape_t shape_rotated(shape_original);
                    shape_rotated.rotate(0, 0, PI/2, center_mass).colorize(depth_min, depth_max);

                    {
                        auto avg_bot_right = shape_rotated.center_mass(
                            frame_width*(guaranteed_floor + (1-guaranteed_floor)/2),
                            frame_width,
                            0,
                            frame_height / 2,
                            depth_min,
                            depth_max
                            );

                        auto avg_bot_left = shape_rotated.center_mass(
                            frame_width*(guaranteed_floor + (1-guaranteed_floor)/2),
                            frame_width,
                            frame_height / 2,
                            frame_height,
                            depth_min,
                            depth_max
                            );

                        auto avg_top_center = shape_rotated.center_mass(
                            frame_width * guaranteed_floor,
                            frame_width * (guaranteed_floor + (1 - guaranteed_floor) / 2),
                            0,
                            frame_height,
                            depth_min,
                            depth_max
                            );


                        rs2::vertex avg_bot_center;
                        avg_bot_center.x = (avg_bot_left.x + avg_bot_right.x) / 2;
                        avg_bot_center.y = (avg_bot_left.y + avg_bot_right.y) / 2;
                        avg_bot_center.z = (avg_bot_left.z + avg_bot_right.z) / 2;

                        shape_rotated.lines.push_back(avg_bot_right);
                        shape_rotated.lines.push_back(avg_bot_left);
                        shape_rotated.lines.push_back(avg_top_center);
                        shape_rotated.lines.push_back(avg_bot_center);
                        shape_rotated.line_color = rgb_t{255,0,0};

                        renderRgb("shape_rotated", shape_rotated);

                        rotation1 = angle_between(avg_top_center.z, avg_top_center.y, avg_bot_center.z, avg_bot_center.y);
                        auto avg_bot_right_2 = rotate(avg_bot_right, rotation1, 0,0,center_mass);
                        auto avg_bot_left_2 = rotate(avg_bot_left, rotation1, 0,0,center_mass);
                        rotation2 = angle_between(avg_bot_right_2.x, avg_bot_right_2.y, avg_bot_left_2.x, avg_bot_left_2.y);


                        shape_t shape_rotated2(shape_original);
                        shape_rotated2.rotate(rotation1, 0, 90*PI/180-rotation2, center_mass).colorize(depth_min, depth_max);
                        renderRgb("shape_rotated2", shape_rotated2);

                        shape_t shape_floored2(shape_original);
                        shape_floored2.rotate(0,-rotation1,-rotation2,center_mass).rotate(0,0,0,center_mass).rotate(rotateA, rotateB, rotateC, center_mass).colorize(depth_min, depth_max);
                        renderRgb("shape_floored2", shape_floored2);
                    }



                    shape_t shape_floored(shape_original);
                    shape_floored.rotate(rotateA, rotateB, rotateC, center_mass).colorize(depth_min, depth_max);
                    renderRgb("shape_floored", shape_floored);

                    rs2::vertex an{rotateA,rotateB,rotateC};
                    rs2::vertex an2{rotation1,rotation2,0};
                    rs2::vertex an3{rot1,rot2,0};

                    statusMessage(point_toString(an) + " : " + point_toString(an2) + " : " + point_toString(an3));


                } else if (stream_type == RS2_STREAM_COLOR) {
                    auto color = frame.as<rs2::video_frame>();
                    auto frame_width = color.get_width();
                    auto frame_height = color.get_height();
                    auto frame_size = frame_width * frame_height;
                    auto color_data = (uint8_t*)color.get_data();

                    auto color_copy = new uint8_t[frame_size * 3];
                    memcpy(color_copy, color_data, frame_size * 3);
                    emit drawColor(color_copy, frame_width, frame_height);
                }
            }

        }

    }
};

#endif // UTILS_H
