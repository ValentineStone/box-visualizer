#ifndef UTILS_H
#define UTILS_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <QMutex>
#include <limits>
#include <QString>
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <filesystem>
#include <fstream>

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
QString pointToString(Point p) {
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

    shape_t(const shape_t& other): shape_t(other.width, other.height, other.points) {}

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
    uint8_t* render(float scale = 1, size_t* render_map = nullptr) const {
        Pixel* pixels = (Pixel*)new uint8_t[size * sizeof(Pixel)];
        memset(pixels, 0, size * sizeof(Pixel));
        for (size_t i = 0; i < size; i++) {
            auto& point = points[i];
            size_t x = point.x * scale * width/2 + width/2;
            if (x < 0 || x >= width) continue;
            size_t y = point.y * scale * width/2 + height/2;
            if (y < 0 || y >= height) continue;
            Pixel& pixel = pixels[y * width + x];
            if (render_map) render_map[y * width + x] = i;
            if (colors[i]) pixel = colors[i];
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

    shape_t& slice_x(float min, float max) {
        for (size_t i = 0; i < size; i++)
            if (points[i].x < min || points[i].x > max)
                colors[i] = 0;
        return *this;
    }

    shape_t& slice_y(float min, float max) {
        for (size_t i = 0; i < size; i++)
            if (points[i].y < min || points[i].y > max)
                colors[i] = 0;
        return *this;
    }

    shape_t& slice_z(float min, float max) {
        for (size_t i = 0; i < size; i++)
            if (points[i].z < min || points[i].z > max)
                colors[i] = 0;
        return *this;
    }
};

struct distances_t {
    uint16_t frame_width = 0;
    uint16_t frame_height = 0;
    float depth_units = 0;
    uint16_t* data = nullptr;
    bool owns_data = false;

    distances_t() = default;
    distances_t(const rs2::depth_frame& frame) {
        frame_width = frame.get_width();
        frame_height = frame.get_height();
        if (auto sensor = rs2::sensor_from_frame(frame)) {
            depth_units = sensor->get_option(RS2_OPTION_DEPTH_UNITS);
            data = (uint16_t*)frame.get_data();
        }
        else {
            data = nullptr;
        }
    }
    float get_distance(uint16_t x, uint16_t y) {
        return data[frame_width * y + x] * depth_units;
    }
    int serialize_size() {
        if (!data) return -1;
        return 2 + 2 + 4 + (2 * frame_width * frame_height);
    }
    bool serialize(uint8_t* buff) {
        if (!data) return false;
        *(uint16_t*)(buff + 0) = frame_width;
        *(uint16_t*)(buff + 2) = frame_height;
        *(float*)(buff + 4) = depth_units;
        auto cpy_n = frame_width * frame_height * 2;
        memcpy(buff + 8, data, cpy_n);
        return true;
    }
    bool serialize_to_file(const std::filesystem::path& path) {
        auto distances_bin_buff_len = serialize_size();
        if (distances_bin_buff_len < 0)
            return false;
        uint8_t* distances_bin_buff = new uint8_t[distances_bin_buff_len];
        if (serialize(distances_bin_buff)) {
            std::ofstream distances_file;
            distances_file.open(path);
            distances_file.write((const char*)distances_bin_buff, distances_bin_buff_len);
            distances_file.close();
            delete[] distances_bin_buff;
            return true;
        }
        else {
            delete[] distances_bin_buff;
            return false;
        }
    }
    distances_t(const uint8_t* buff, size_t len) {
        if (len < 8) {
            data  = nullptr;
            return;
        }
        frame_width = *(uint16_t*)(buff + 0);
        frame_height = *(uint16_t*)(buff + 2);
        depth_units = *(float*)(buff + 4);
        if (len < serialize_size()) {
            data  = nullptr;
            return;
        }
        data = new uint16_t[frame_width * frame_height];
        owns_data = true;
        auto cpy_n = frame_width * frame_height * 2;
        memcpy(data, buff + 8, cpy_n);
    }
    distances_t(std::filesystem::path path) {
        std::ifstream distances_file;
        distances_file.open(path);
        distances_file.read((char*)&frame_width, 2);
        distances_file.read((char*)&frame_height, 2);
        distances_file.read((char*)&depth_units, 4);
        data = new uint16_t[frame_width * frame_height];
        owns_data = true;
        auto cpy_n = frame_width * frame_height * 2;
        distances_file.read((char*)data, cpy_n);
        distances_file.close();
    }
    ~distances_t() {
        if (owns_data)
            delete[] data;
        data = nullptr;
    }
    std::string to_string() {
        if (!data) return "";
        std::stringstream ss;
        ss << frame_width << ',' << frame_height << ',' << depth_units << "\r\n\r\n";
        for (uint16_t y = 0; y < frame_height; y++) {
            for (uint16_t x = 0; x < frame_width; x++) {
                ss << get_distance(x, y) << ',';
            }
            ss << "\r\n";
        }
        return ss.str();
    }
    operator std::string() {
        return to_string();
    }
};

template <typename T = std::nullptr_t>
void save_to_file(const std::filesystem::path& path, const void* buff, size_t size) {
    std::ofstream file;
    file.open(path);
    file.write((const char*)buff, size);
    file.close();
}

template <typename T = std::nullptr_t>
void load_from_file(const std::filesystem::path& path, void* buff, size_t size) {
    std::ifstream file;
    file.open(path);
    file.read((char*)buff, size);
    file.close();
}


class WorkerThread : public QThread
{
    Q_OBJECT


public:
    WorkerThread(QObject* parent = nullptr):  QThread(parent) {}
    bool paused = false;
    float rotateA = 0;
    float rotateB = 0;
    float rotateC = 0;
    float zoomScale = 1;
    float sliceLevel = 0;
    float boxHeight = 0.4;

    float lookat_x = 0;
    float lookat_y = 0;

    float sliceEpsilon = 0.1;
    float guaranteed_floor = 0.9;

    float depth_min = 0.5;
    float depth_max = 5;

    distances_t* loaded_distances = nullptr;
    rs2_intrinsics loaded_intr;

    void setDataSource(QString dataSource) {
        loop_mutex.lock();
        if (pipelineStarted) {
            rs_pipeline.stop();
            pipelineStarted = false;
        }
        if (loaded_distances)
            delete loaded_distances;
        loaded_distances = nullptr;

        if (dataSource == "live") {
            rs2::config rs_config;
            //rs_config.enable_stream(RS2_STREAM_COLOR, 320, 240, RS2_FORMAT_RGB8, 30); // 1920, 1080
            rs_config.enable_stream(RS2_STREAM_DEPTH, 320, 240, RS2_FORMAT_Z16, 30);   // 1024, 768
            //rs_config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
            //rs_config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);

            rs_profile = rs_pipeline.start(rs_config);
            pipelineStarted = true;
        } else if (dataSource.endsWith(".bag")) {
            rs2::config rs_config;
            rs_config.enable_device_from_file(dataSource.toStdString());
            rs_profile = rs_pipeline.start(rs_config);
            pipelineStarted = true;
        } else if (dataSource.endsWith("distances.bin")) {
            presentFromFiles(dataSource);
        } else {
            // do nothing
        }
        loop_mutex.unlock();
    }

signals:
    void drawDepth(uint16_t* data, size_t width, size_t height, int rotate = 0);
    void drawColor(uint8_t* data, size_t width, size_t height, int rotate = 0);
    void drawRgb(QString, uint8_t* data, size_t width, size_t height, int rotate = 0);
    void statusMessage(QString);
    void debugText(QString);

private:
    rs2::pipeline rs_pipeline;
    rs2::pipeline_profile rs_profile;
    QMutex loop_mutex;
    bool pipelineStarted = false;

    void run() override {

        qDebug() << "started!";

        while (true) {
            loop_mutex.lock();
            if (!paused) {
                if (pipelineStarted) {
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

                            auto depth_copy = new uint16_t[frame_size];
                            memcpy(depth_copy, depth_data, frame_size * sizeof(uint16_t));
                            for (size_t i = 0; i < frame_size; i++)
                                depth_copy[i] = to_grayscale<uint16_t>(depth_data[i] * depth_units, depth_min, depth_max);
                            emit drawDepth(depth_copy, frame_width, frame_height, 90);

                            processDepth(depth_data, frame_width, frame_height, intr, depth_units);


                        } else if (stream_type == RS2_STREAM_COLOR) {
                            auto color = frame.as<rs2::video_frame>();
                            auto frame_width = color.get_width();
                            auto frame_height = color.get_height();
                            auto frame_size = frame_width * frame_height;
                            auto color_data = (uint8_t*)color.get_data();

                            auto color_copy = new uint8_t[frame_size * 3];
                            memcpy(color_copy, color_data, frame_size * 3);
                            emit drawColor(color_copy, frame_width, frame_height, 90);
                        }
                    }
                } else if (loaded_distances) {
                    processDepth(
                        loaded_distances->data,
                        loaded_distances->frame_width,
                        loaded_distances->frame_height,
                        loaded_intr,
                        loaded_distances->depth_units
                    );
                }
            }

            loop_mutex.unlock();

            this->msleep(10);
        }

    }

    void presentFromFiles(QString filename) {
        std::filesystem::path distances_path = filename.toStdString();
        std::filesystem::path intr_path = filename.replace("distances.bin", "intrinsics.bin").toStdString();
        if (loaded_distances)
            delete loaded_distances;
        loaded_distances = new distances_t(distances_path);
        load_from_file(intr_path, &loaded_intr, sizeof(rs2_intrinsics));
    }

    void processDepth(uint16_t *depth_data, size_t frame_width, size_t frame_height, rs2_intrinsics &intr, float depth_units) {
        auto frame_size = frame_width * frame_height;

        shape_t shape_original(frame_width, frame_height, depth_data, intr, depth_units);

        auto center_mass = shape_original.center_mass(depth_min, depth_max);

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

        size_t* shape_original_render_map = new size_t[frame_size];
        std::fill_n(shape_original_render_map, frame_size, -1);
        emit drawRgb(
            "shape_original",
            shape_t(shape_original)
                .rotate(rotateA,rotateB,rotateC,center_mass)
                .colorize(depth_min, depth_max)
                .render(zoomScale, shape_original_render_map),
            frame_width,
            frame_height,
            90
            );

        // account for 90 degree rotation
        size_t lookat_y = (frame_height-1) - this->lookat_x * frame_height;
        size_t lookat_x = this->lookat_y * frame_width;
        size_t lookat_i = lookat_y * frame_width + lookat_x;
        float lookat_depth = std::numeric_limits<float>::quiet_NaN();
        if (shape_original_render_map[lookat_i] != -1)
            lookat_depth = shape_original.points[shape_original_render_map[lookat_i]].z;
        delete[] shape_original_render_map;

        emit statusMessage(QString("Lookat (%1,%2) depth = %3").arg(lookat_x).arg(lookat_y).arg(lookat_depth));

        float rot1 = angle_between(avg_top_center.z, avg_top_center.x, avg_bot_center.z, avg_bot_center.x);
        auto avg_bot_right_2 = rotate(avg_bot_right, 0, -rot1, 0,center_mass);
        auto avg_bot_left_2 = rotate(avg_bot_left, 0, -rot1, 0,center_mass);
        float rot2 = PI - angle_between(avg_bot_right_2.y, avg_bot_right_2.x, avg_bot_left_2.y, avg_bot_left_2.x);

        shape_t shape_floored(shape_original);
        shape_floored.rotate(0,-rot1,-rot2,center_mass);

        emit drawRgb(
            "shape_floored",
            shape_t(shape_floored)
                .rotate(rotateA,rotateB,rotateC,center_mass)
                .colorize(depth_min, depth_max)
                .render(zoomScale),
            frame_width,
            frame_height,
            90
            );

        shape_t shape_topview(shape_floored);
        shape_topview.rotate(0,-PI/2,0,center_mass);

        emit drawRgb(
            "shape_topview",
            shape_t(shape_topview)
                .rotate(rotateA,rotateB,rotateC,center_mass)
                .colorize(depth_min, depth_max)
                .render(zoomScale),
            frame_width,
            frame_height,
            90
            );

        auto floor_level = rotate(avg_top_center,0,-PI/2-rot1,-rot2,center_mass);

        emit drawRgb(
            "shape_topview_floor",
            shape_t(shape_topview)
                .rotate(rotateA,rotateB,rotateC,center_mass)
                .colorize(depth_min, depth_max)
                .slice_z(floor_level.z - sliceEpsilon, floor_level.z + sliceEpsilon)
                .render(zoomScale),
            frame_width,
            frame_height,
            90
            );

        emit drawRgb(
            "shape_topview_box",
            shape_t(shape_topview)
                .rotate(rotateA,rotateB,rotateC,center_mass)
                .colorize(depth_min, depth_max)
                .slice_z(floor_level.z - boxHeight + sliceLevel - sliceEpsilon, floor_level.z - boxHeight + sliceLevel + sliceEpsilon)
                .render(zoomScale),
            frame_width,
            frame_height,
            90
            );
    }
};

#endif // UTILS_H
