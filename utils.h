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

struct float3 {
    float x = 0, y = 0, z = 0;
    float3 operator*(float t) {
        return { x * t, y * t, z * t };
    }
    float3 operator-(float t) {
        return { x - t, y - t, z - t };
    }
    void operator*=(float t) {
        x = x * t;
        y = y * t;
        z = z * t;
    }
    void operator=(float3 other) {
        x = other.x;
        y = other.y;
        z = other.z;
    }
    void add(float t1, float t2, float t3) {
        x += t1;
        y += t2;
        z += t3;
    }
    float3 to_deg() {
        float3 deg;
        deg.x = x * 180 / PI;
        deg.y = y * 180 / PI;
        deg.z = z * 180 / PI;
        return deg;
    }
    float3 to_rad() {
        float3 deg;
        deg.x = x * PI / 180;
        deg.y = y * PI / 180;
        deg.z = z * PI / 180;
        return deg;
    }

    QString toString() {
        return "(x=" + QString::number(x) + ",y=" + QString::number(y) + ",z=" + QString::number(z) + ")";
    }
};

struct rotation_estimator_t
{
    // theta is the angle of camera rotation in x, y and z components
    float3 theta;
    std::mutex theta_mtx;
    /* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
    values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
    float alpha = 0.98f;
    bool firstGyro = true;
    bool firstAccel = true;
    // Keeps the arrival time of previous gyro frame
    double last_ts_gyro = 0;

    // Function to calculate the change in angle of motion based on data from gyro
    void process_gyro(rs2_vector gyro_data, double ts)
    {
        if (firstGyro) // On the first iteration, use only data from accelerometer to set the camera's initial position
        {
            firstGyro = false;
            last_ts_gyro = ts;
            return;
        }
        // Holds the change in angle, as calculated from gyro
        float3 gyro_angle;

        // Initialize gyro_angle with data from gyro
        gyro_angle.x = gyro_data.x; // Pitch
        gyro_angle.y = gyro_data.y; // Yaw
        gyro_angle.z = gyro_data.z; // Roll

        // Compute the difference between arrival times of previous and current gyro frames
        double dt_gyro = (ts - last_ts_gyro) / 1000.0;
        last_ts_gyro = ts;

        // Change in angle equals gyro measures * time passed since last measurement
        gyro_angle = gyro_angle * static_cast<float>(dt_gyro);

        // Apply the calculated change of angle to the current angle (theta)
        std::lock_guard<std::mutex> lock(theta_mtx);
        theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
    }

    void process_accel(rs2_vector accel_data)
    {
        // Holds the angle as calculated from accelerometer data
        float3 accel_angle;

        // Calculate rotation angle from accelerometer data
        accel_angle.z = atan2(accel_data.y, accel_data.z);
        accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

        // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
        std::lock_guard<std::mutex> lock(theta_mtx);
        if (firstAccel)
        {
            firstAccel = false;
            theta = accel_angle;
            // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
            theta.y = PI_FL;
        }
        else
        {
            /*
            Apply Complementary Filter:
                - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                  that are steady over time, is used to cancel out drift.
                - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations
            */
            theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
            theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
        }
    }

    // Returns the current rotation angle
    float3 get_theta()
    {
        std::lock_guard<std::mutex> lock(theta_mtx);
        return theta;
    }
};

struct point_t {
    float x;
    float y;
    float z;
    QString toString() {
        return QString::number(x) + ',' + QString::number(y) + ',' + QString::number(z);
    }
    point_t& rotate(float a, float b, float c) {
        float x = this->x;
        float y = this->y;
        float z = this->z;

        float sinA = sin(a);
        float cosA = cos(a);
        float sinB = sin(b);
        float cosB = cos(b);
        float sinC = sin(c);
        float cosC = cos(c);

        /*
        this->x = x*cosB*cosC - y*sinC*cosB + z*sinB;
        this->y = x*(sinA*sinB*cosC+sinC*cosA) - y*(sinA*sinB*sinC+cosA*cosC) - z*sinA*cosB;
        this->z = x*(sinA*sinC-sinB*cosA*cosC) + y*(sinA*cosC+sinB*sinC*cosA) + z*cosA*cosB;
        */


        this->x = x*cosB*cosC - y*sinC*cosB + z*sinB;
        this->y = x*(sinA*sinB*cosC+sinC*cosA) + y*(-sinA*sinB*sinC+cosA*cosC) - z*sinA*cosB;
        this->z = x*(sinA*sinC-sinB*cosA*cosC) + y*(sinA*cosC+sinB*sinC*cosA) + z*cosA*cosB;

        return *this;
    }
};

template <typename T>
T rotate(const T& point, float a, float b, float c) {
    T rotated;
    float x = point.x;
    float y = point.y;
    float z = point.z;

    float sinA = sin(a);
    float cosA = cos(a);
    float sinB = sin(b);
    float cosB = cos(b);
    float sinC = sin(c);
    float cosC = cos(c);

    /*
        this->x = x*cosB*cosC - y*sinC*cosB + z*sinB;
        this->y = x*(sinA*sinB*cosC+sinC*cosA) - y*(sinA*sinB*sinC+cosA*cosC) - z*sinA*cosB;
        this->z = x*(sinA*sinC-sinB*cosA*cosC) + y*(sinA*cosC+sinB*sinC*cosA) + z*cosA*cosB;
        */


    rotated.x = x*cosB*cosC - y*sinC*cosB + z*sinB;
    rotated.y = x*(sinA*sinB*cosC+sinC*cosA) + y*(-sinA*sinB*sinC+cosA*cosC) - z*sinA*cosB;
    rotated.z = x*(sinA*sinC-sinB*cosA*cosC) + y*(sinA*cosC+sinB*sinC*cosA) + z*cosA*cosB;

    return rotated;
}


struct rpoint_t {
    float a_horizontal;
    float a_vertical;
    float depth;
    point_t to_3d() {
        point_t point;
        point.x = depth * std::sin(a_horizontal);
        point.y = depth * std::cos(a_horizontal) * std::sin(a_vertical);
        point.z = depth;// * std::cos(a_horizontal) * std::cos(a_vertical);
        return point;
    }
    QString toString() {
        return QString::number(a_horizontal) + ',' + QString::number(a_vertical) + ',' + QString::number(depth);
    }
};


template<typename Point>
QString point_toString(Point p) {
    return QString::number(p.x) + ',' + QString::number(p.y) + ',' + QString::number(p.z);
}


struct scaline_normalizer {
    static bool detect(float* points, size_t width, size_t height, size_t cutoff, size_t& x1, size_t& y1, size_t& x2, size_t& y2, size_t& x3, size_t& y3) {

        float maxR = 0;//std::numeric_limits<float>::infinity();
        float maxL = 0;//std::numeric_limits<float>::infinity();
        float minR = std::numeric_limits<float>::infinity();
        float minL = std::numeric_limits<float>::infinity();
        size_t maxRX = width - 1;
        size_t maxLX = width - 1;
        size_t yR = 0;
        size_t yL = height - 1;
        for (size_t x = cutoff; x < width; x++) {
            size_t iR = x + yR * width;
            size_t iL = x + yL * width;
            if (points[iR] > maxR) {
                maxR = points[iR];
            }
            if (points[iL] > maxL) {
                maxL = points[iL];
            }
            if (points[iR] != 0 && points[iR] > maxR) {
                maxR = points[iR];
            }
            if (points[iL] > maxL) {
                maxL = points[iL];
            }
        }

        if (maxL > maxR) {

        }


        return true;
    }
};


template <typename Uint>
Uint to_color(float val, float min, float max) {
    static const Uint max_color = std::numeric_limits<Uint>::max();
    float dv = max - min;
    if (val < min) return max_color;
    if (val > max) return 0;
    return max_color - (val - min) / dv * max_color;
}

struct rgb_pixel_t {
    uint8_t r = 0, g = 0, b = 0;

    template<typename Number>
    rgb_pixel_t& operator=(Number num) {
        r = g = b = num / (float)std::numeric_limits<Number>::max() * 255;
        return *this;
    }
    operator const uint8_t() const { return r; }
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
    return atan2(deltaY, deltaX);
}

/*
struct rgb_points_t {
    std::vector<rs2::vertex> vertexes;
    std::vector<rgb_points_t> colors;
    size_t width;
    size_t height;
    rgb_points_t(size_t width, size_t height) {
        this->width = width;
        this->height = height;
        vertexes.resize(width * height);
        colors.resize(width * height);
    }

    void push(rs2::vertex vertex, rgb_points_t color) {
        vertexes.push_back(vertex);
        colors.push_back(color);
    }

    void render(rgb_points_t* canvas, size_t c_width, size_t c_height) {

    }
};
*/


class WorkerThread : public QThread
{
    Q_OBJECT

    std::mutex wait_for_frames_mutex;

    void run() override {
        rs2::config rs_config;



        if (false) {
            rs_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30); // 1920, 1080
            rs_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);   // 1024, 768
            //rs_config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
            //rs_config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        } else {
            //rs_config.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);
            rs_config.enable_stream(RS2_STREAM_DEPTH, 1024, 768, RS2_FORMAT_Z16, 30);
            //rs_config.enable_device_from_file("/home/v/flyover.bag");
            //rs_config.enable_device_from_file("/home/v/Documents/20240229_082253.bag");
            rs_config.enable_device_from_file("/home/v/Downloads/test4.bag");
        }

        rs2::pipeline rs_pipeline;
        rotation_estimator_t rotation_estimator;
        rs2::pipeline_profile rs_profile;

        rs_profile = rs_pipeline.start(rs_config);
        qDebug() << "started!";


        constexpr size_t dimensionX = 500;
        constexpr size_t dimensionY = 500;
        uint16_t* pixels = new uint16_t[dimensionX * dimensionY];

        uint64_t last_fno = 0;

        float sliceEpsilon = 0.1;

        float guaranteed_floor = 0.9;

        while (true) {
            if (paused) continue;

            std::unique_lock<std::mutex> lock(wait_for_frames_mutex);
            rs2::frameset frameset = rs_pipeline.wait_for_frames();

            auto fno = frameset.get_frame_number();
            if (fno < last_fno) {
                // reset;
                rotation_estimator.firstAccel = true;
                rotation_estimator.firstGyro = true;
            }
            last_fno = fno;

            for (const rs2::frame& frame: frameset) {
                auto stream_type = frame.get_profile().stream_type();
                auto format = frame.get_profile().format();

                if (stream_type == RS2_STREAM_DEPTH) {
                    auto depth = frame.as<rs2::depth_frame>();
                    auto sensor = rs2::sensor_from_frame(depth);
                    auto depth_units = sensor->get_option(RS2_OPTION_DEPTH_UNITS);
                    auto frame_width = depth.get_width();
                    auto frame_height = depth.get_height();
                    auto frame_points_count = frame_width * frame_height;
                    auto depth_data = (uint16_t*)depth.get_data();

                    auto depth_data2 = new uint16_t[frame_points_count];

                    uint16_t maxDepth = 0;
                    for (size_t i = 0; i < frame_points_count; i++) {
                        if (depth_data[i] > maxDepth)
                            maxDepth = depth_data[i];
                    }
                    float scale_up = 2;//maxDepth * depth_units;

                    for (size_t y = 0; y < frame_height; y++) {
                        for (size_t x = 0; x < frame_width; x++) {
                            auto i = y * frame_width + x;
                            if (depth_data[i])
                                depth_data2[i] = to_color<uint16_t>(depth_data[i] * depth_units, 0.5, 9);//65535 - depth_data[i] * depth_units / scale_up * 65535;
                            else
                                depth_data2[i] = 0;

                            if (y % 50 == 0)
                                depth_data2[i] = 0;
                            if (x % 100 == 0)
                                depth_data2[i] = 0;
                        }
                    }

                    emit resultReady("label1", depth_data2, frame_width, frame_height, true);



                    // ==================================== color ====================

                    rgb_pixel_t* pixels = new rgb_pixel_t[dimensionX * dimensionY];
                    memset(pixels, 0, dimensionX * dimensionY * sizeof(rgb_pixel_t));

                    std::vector<rs2::vertex> extra_points;
                    rs2::pointcloud pointcloud;
                    rs2::points rspoints = pointcloud.calculate(depth);
                    auto vertices = rspoints.get_vertices();
                    for (size_t i = 0; i < rspoints.size(); i++) {
                        if (vertices[i].z == 0) continue;
                        auto vertex = rotate(vertices[i],rotateA,rotateB,rotateC+90*PI/180);
                        size_t x = vertex.x * zoomScale * dimensionX/2 + dimensionX/2;
                        size_t y = vertex.y * zoomScale * dimensionX/2 + dimensionY/2;
                        uint8_t color = to_color<uint8_t>(vertices[i].z, 0.5, 9);
                        if (y < 0 || y >= dimensionY) continue;
                        if (x < 0 || x >= dimensionX) continue;
                        pixels[y * dimensionX + x] = pixels[y * dimensionX + x] < color ? color : pixels[y * dimensionX + x];
                    }

                    /*
                    for (size_t fy = 0; fy < frame_height; fy++) {
                        for (size_t fx = frame_width * 0.75; fx < frame_width; fx++) {
                            auto i = fy * frame_width + fx;
                            if (vertices[i].z == 0) continue;
                            auto vertex = rotate(vertices[i],rotateA,rotateB,rotateC+90*PI/180);

                            size_t x = vertex.x * zoomScale * dimensionX/2 + dimensionX/2;
                            size_t y = vertex.y * zoomScale * dimensionX/2 + dimensionY/2;
                            uint8_t color = to_color<uint8_t>(vertices[i].z, 0.5, 2);
                            if (y < 0 || y >= dimensionY) continue;
                            if (x < 0 || x >= dimensionX) continue;
                            pixels[y * dimensionX + x] = (uint8_t)(pixels[y * dimensionX + x] * 0.5);
                        }
                    }
                    */



                    rs2::vertex avg_bot_right{0,0,0};
                    size_t avg_bot_right_cnt = 0;
                    for (size_t fy = 0; fy < frame_height / 2; fy++) {
                        for (size_t fx = frame_width*(guaranteed_floor + (1-guaranteed_floor)/2); fx < frame_width; fx++) {
                            auto i = fy * frame_width + fx;
                            if (vertices[i].z == 0) continue;
                            auto vertex = rotate(vertices[i],rotateA,rotateB,rotateC+90*PI/180);

                            size_t x = vertex.x * zoomScale * dimensionX/2 + dimensionX/2;
                            size_t y = vertex.y * zoomScale * dimensionX/2 + dimensionY/2;
                            uint8_t color = to_color<uint8_t>(vertices[i].z, 0.5, 2);

                            avg_bot_right.x += vertex.x;
                            avg_bot_right.y += vertex.y;
                            avg_bot_right.z += vertex.z;
                            avg_bot_right_cnt++;

                            if (y < 0 || y >= dimensionY) continue;
                            if (x < 0 || x >= dimensionX) continue;
                            pixels[y * dimensionX + x] = 0;
                            pixels[y * dimensionX + x].r = 128;
                        }
                    }
                    avg_bot_right.x /= avg_bot_right_cnt;
                    avg_bot_right.y /= avg_bot_right_cnt;
                    avg_bot_right.z /= avg_bot_right_cnt;



                    rs2::vertex avg_bot_left{0,0,0};
                    size_t avg_bot_left_cnt = 0;
                    for (size_t fy = frame_height - 1; fy > frame_height / 2; fy--) {
                        for (size_t fx = frame_width*(guaranteed_floor + (1-guaranteed_floor)/2); fx < frame_width; fx++) {
                            auto i = fy * frame_width + fx;
                            if (vertices[i].z == 0) continue;
                            auto vertex = rotate(vertices[i],rotateA,rotateB,rotateC+90*PI/180);

                            size_t x = vertex.x * zoomScale * dimensionX/2 + dimensionX/2;
                            size_t y = vertex.y * zoomScale * dimensionX/2 + dimensionY/2;
                            uint8_t color = to_color<uint8_t>(vertices[i].z, 0.5, 2);

                            avg_bot_left.x += vertex.x;
                            avg_bot_left.y += vertex.y;
                            avg_bot_left.z += vertex.z;
                            avg_bot_left_cnt++;

                            if (y < 0 || y >= dimensionY) continue;
                            if (x < 0 || x >= dimensionX) continue;
                            pixels[y * dimensionX + x] = 0;
                            pixels[y * dimensionX + x].g = 128;
                        }
                    }
                    avg_bot_left.x /= avg_bot_left_cnt;
                    avg_bot_left.y /= avg_bot_left_cnt;
                    avg_bot_left.z /= avg_bot_left_cnt;



                    rs2::vertex avg_top_center{0,0,0};
                    size_t avg_top_center_cnt = 0;
                    for (size_t fy = 0; fy < frame_height; fy++) {
                        for (size_t fx = frame_width * guaranteed_floor; fx < frame_width * (guaranteed_floor + (1-guaranteed_floor)/2); fx++) {
                            auto i = fy * frame_width + fx;
                            if (vertices[i].z == 0) continue;
                            auto vertex = rotate(vertices[i],rotateA,rotateB,rotateC+90*PI/180);

                            size_t x = vertex.x * zoomScale * dimensionX/2 + dimensionX/2;
                            size_t y = vertex.y * zoomScale * dimensionX/2 + dimensionY/2;
                            uint8_t color = to_color<uint8_t>(vertices[i].z, 0.5, 2);

                            avg_top_center.x += vertex.x;
                            avg_top_center.y += vertex.y;
                            avg_top_center.z += vertex.z;
                            avg_top_center_cnt++;

                            if (y < 0 || y >= dimensionY) continue;
                            if (x < 0 || x >= dimensionX) continue;
                            pixels[y * dimensionX + x] = 0;
                            pixels[y * dimensionX + x].b = 128;
                        }
                    }
                    avg_top_center.x /= avg_top_center_cnt;
                    avg_top_center.y /= avg_top_center_cnt;
                    avg_top_center.z /= avg_top_center_cnt;

                    rs2::vertex avg_bot_center;
                    avg_bot_center.x = (avg_bot_left.x + avg_bot_right.x) / 2;
                    avg_bot_center.y = (avg_bot_left.y + avg_bot_right.y) / 2;
                    avg_bot_center.z = (avg_bot_left.z + avg_bot_right.z) / 2;

                    extra_points.push_back(avg_top_center);
                    extra_points.push_back(avg_bot_left);
                    extra_points.push_back(avg_bot_right);
                    extra_points.push_back(avg_bot_center);
                    for (auto& vertex : extra_points) {
                        size_t x = vertex.x * zoomScale * dimensionX/2 + dimensionX/2;
                        size_t y = vertex.y * zoomScale * dimensionY/2 + dimensionY/2;
                        if (y < 0 || y >= dimensionY) continue;
                        if (x < 0 || x >= dimensionX) continue;
                        pixels[y * dimensionX + x] = std::numeric_limits<uint8_t>::max();
                    }

                    for (auto& vertex1 : extra_points) {
                        for (auto& vertex2 : extra_points) {
                            if (&vertex1 == &vertex2) continue;
                            size_t x1 = vertex1.x * zoomScale * dimensionX/2 + dimensionX/2;
                            size_t y1 = vertex1.y * zoomScale * dimensionY/2 + dimensionY/2;
                            size_t x2 = vertex2.x * zoomScale * dimensionX/2 + dimensionX/2;
                            size_t y2 = vertex2.y * zoomScale * dimensionY/2 + dimensionY/2;
                            draw_line(pixels, dimensionX, dimensionY, std::numeric_limits<uint8_t>::max(), x1, y1, x2, y2);
                        }
                    }

                    emit resultReady("rgb", pixels, dimensionX, dimensionY, true);

                    float rotation2 = angle_between(avg_top_center.z, avg_top_center.y, avg_bot_center.z, avg_bot_center.y);
                    auto avg_bot_right_2 = rotate(avg_bot_right, -rotation2, 0,0);
                    auto avg_bot_left_2 = rotate(avg_bot_left, -rotation2, 0,0);
                    float rotation1 = angle_between(avg_bot_right_2.x, avg_bot_right_2.y, avg_bot_left_2.x, avg_bot_left_2.y);

                    auto floor_level = rotate(avg_bot_center,rotateA - rotation2-90*PI/180,rotateB,rotateC - rotation1);

                    if (true) {
                        rgb_pixel_t* pixels = new rgb_pixel_t[dimensionX * dimensionY];
                        memset(pixels, 0, dimensionX * dimensionY * sizeof(rgb_pixel_t));
                        std::vector<rs2::vertex> extra_points;
                        rs2::pointcloud pointcloud;
                        rs2::points rspoints = pointcloud.calculate(depth);
                        auto vertices = rspoints.get_vertices();
                        for (size_t i = 0; i < rspoints.size(); i++) {
                            if (vertices[i].z == 0) continue;
                            auto vertex = rotate(vertices[i],rotateA - rotation2,rotateB,rotateC+90*PI/180 - rotation1);
                            size_t x = vertex.x * zoomScale * dimensionX/2 + dimensionX/2;
                            size_t y = vertex.y * zoomScale * dimensionX/2 + dimensionY/2;
                            uint8_t color = to_color<uint8_t>(vertices[i].z, 0.5, 9);
                            if (y < 0 || y >= dimensionY) continue;
                            if (x < 0 || x >= dimensionX) continue;
                            pixels[y * dimensionX + x] = pixels[y * dimensionX + x] < color ? color : pixels[y * dimensionX + x];
                        }
                        emit resultReady("rgb2", pixels, dimensionX, dimensionY, true);
                    }

                    float avgCenterX1 = 0, avgCenterY1 = 0, countCenter1 = 0;
                    float avgCenterX2 = 0, avgCenterY2 = 0, countCenter2 = 0;

                    if (true) {
                        rgb_pixel_t* pixels = new rgb_pixel_t[dimensionX * dimensionY];
                        memset(pixels, 0, dimensionX * dimensionY * sizeof(rgb_pixel_t));
                        std::vector<rs2::vertex> extra_points;
                        rs2::pointcloud pointcloud;
                        rs2::points rspoints = pointcloud.calculate(depth);
                        auto vertices = rspoints.get_vertices();
                        for (size_t i = 0; i < rspoints.size(); i++) {
                            if (vertices[i].z == 0) continue;
                            auto vertex = rotate(vertices[i],rotateA - rotation2-90*PI/180,rotateB,rotateC+90*PI/180 - rotation1);
                            //if (i % 100 == 0) qDebug() << point_toString(vertex);
                            if (vertex.z < floor_level.z + sliceLevel + boxHeight - sliceEpsilon || vertex.z > floor_level.z + sliceLevel + boxHeight + sliceEpsilon) continue;
                            size_t x = vertex.x * zoomScale * dimensionX/2 + dimensionX/2;
                            size_t y = vertex.y * zoomScale * dimensionX/2 + dimensionY/2*0;
                            uint8_t color = to_color<uint8_t>(vertices[i].z, 0.5, 9);
                            if (y < 0 || y >= dimensionY) continue;
                            if (x < 0 || x >= dimensionX) continue;
                            pixels[y * dimensionX + x] = pixels[y * dimensionX + x] < color ? color : pixels[y * dimensionX + x];
                        }



                        for (size_t x = 0; x < dimensionX; x++) {
                            for (size_t y = 0; y < dimensionY; y++) {

                            }
                        }

                        /*
                        rgb_pixel_t green{0,255,0};
                        float prevCenterX = -1;
                        float prevCenterX_Y = -1;
                        size_t minCenterX = 0, minCenterY = 0;
                        size_t maxCenterX = 0, maxCenterY = 0;
                        for (size_t y = 0; y < dimensionY; y++) {
                            float centerX = 0;
                            size_t countX = 0;
                            for (size_t x = 0; x < dimensionX; x++) {
                                size_t i = y * dimensionX + x;
                                if (pixels[i]) {
                                    centerX += x;
                                    countX++;
                                }
                            }
                            if (countX > 10) {
                                centerX /= countX;
                                if (centerX < 0 || centerX > dimensionX) continue;
                                pixels[y * dimensionX + (size_t)centerX].g = 255;
                                if (prevCenterX != -1) {
                                    draw_line(pixels,dimensionX, dimensionY,green,prevCenterX, prevCenterX_Y,centerX,y);
                                }

                                if (minCenterY == 0) {
                                    minCenterY = y;
                                    minCenterX = centerX;
                                }
                                if (y > maxCenterY) {
                                    maxCenterY = y;
                                    maxCenterX = centerX;
                                }
                                prevCenterX = centerX;
                                prevCenterX_Y = y;
                            }
                        }
                        size_t dCenterY = maxCenterY - minCenterY;
                        for (size_t y = minCenterY; y < minCenterY + dCenterY / 2; y++) {
                            for (size_t x = 0; x < dimensionX; x++) {
                                size_t i = y * dimensionX + x;
                                if (pixels[i]) {
                                    pixels[i] = 0;
                                    pixels[i].r = 255;

                                    avgCenterX1 += x;
                                    avgCenterY1 += y;
                                    countCenter1++;
                                }
                            }
                        }
                        avgCenterX1 /= countCenter1;
                        avgCenterY1 /= countCenter1;
                        for (size_t y = minCenterY + dCenterY / 2; y < maxCenterY; y++) {
                            for (size_t x = 0; x < dimensionX; x++) {
                                size_t i = y * dimensionX + x;
                                if (pixels[i]) {
                                    pixels[i] = 0;
                                    pixels[i].b = 255;

                                    avgCenterX2 += x;
                                    avgCenterY2 += y;
                                    countCenter2++;
                                }
                            }
                        }
                        avgCenterX2 /= countCenter2;
                        avgCenterY2 /= countCenter2;


                        draw_line(pixels,dimensionX, dimensionY,(uint8_t)255,avgCenterX1, avgCenterY1,avgCenterX2,avgCenterY2);
                        */

                        emit resultReady("rgb3", pixels, dimensionX, dimensionY, true);
                    }

                    float rotation3 = angle_between(avgCenterY1, avgCenterX1, avgCenterY2, avgCenterX2);
                    //emit statusBarResult(QString::number(rotation3));

                    if (true) {
                        rgb_pixel_t* pixels = new rgb_pixel_t[dimensionX * dimensionY];
                        memset(pixels, 0, dimensionX * dimensionY * sizeof(rgb_pixel_t));
                        std::vector<rs2::vertex> extra_points;
                        rs2::pointcloud pointcloud;
                        rs2::points rspoints = pointcloud.calculate(depth);
                        auto vertices = rspoints.get_vertices();
                        //qDebug() << point_toString(avg_bot_center) << "   |   " << point_toString(floor_level);
                        for (size_t i = 0; i < rspoints.size(); i++) {
                            if (vertices[i].z == 0) continue;
                            auto vertex = rotate(vertices[i],rotateA - rotation2-90*PI/180,rotateB,rotateC+90*PI/180 - rotation1);
                            //if (i % 100 == 0) qDebug() << point_toString(vertex);
                            //if (vertex.z < sliceLevel - 0.1 || vertex.z > sliceLevel + 0.1) continue;
                            if (vertex.z < floor_level.z + sliceLevel - sliceEpsilon || vertex.z > floor_level.z + sliceLevel + sliceEpsilon) continue;
                            size_t x = vertex.x * zoomScale * dimensionX/2 + dimensionX/2;
                            size_t y = vertex.y * zoomScale * dimensionX/2 + dimensionY/2*0;
                            uint8_t color = to_color<uint8_t>(vertices[i].z, 0.5, 9);
                            if (y < 0 || y >= dimensionY) continue;
                            if (x < 0 || x >= dimensionX) continue;
                            pixels[y * dimensionX + x] = pixels[y * dimensionX + x] < color ? color : pixels[y * dimensionX + x];
                        }

                        extra_points.push_back(rotate(avg_top_center,rotateA - rotation2-90*PI/180,rotateB,rotateC- rotation1));
                        extra_points.push_back(rotate(avg_bot_left,rotateA - rotation2-90*PI/180,rotateB,rotateC - rotation1));
                        extra_points.push_back(rotate(avg_bot_right,rotateA - rotation2-90*PI/180,rotateB,rotateC - rotation1));
                        extra_points.push_back(rotate(avg_bot_center,rotateA - rotation2-90*PI/180,rotateB,rotateC - rotation1));
                        for (auto& vertex : extra_points) {
                            size_t x = vertex.x * zoomScale * dimensionX/2 + dimensionX/2;
                            size_t y = vertex.y * zoomScale * dimensionY/2 + dimensionY/2*0;
                            if (y < 0 || y >= dimensionY) continue;
                            if (x < 0 || x >= dimensionX) continue;
                            pixels[y * dimensionX + x] = std::numeric_limits<uint8_t>::max();
                        }

                        rgb_pixel_t blue{0,0,255};

                        for (auto& vertex1 : extra_points) {
                            for (auto& vertex2 : extra_points) {
                                if (&vertex1 == &vertex2) continue;
                                size_t x1 = vertex1.x * zoomScale * dimensionX/2 + dimensionX/2;
                                size_t y1 = vertex1.y * zoomScale * dimensionY/2 + dimensionY/2*0;
                                size_t x2 = vertex2.x * zoomScale * dimensionX/2 + dimensionX/2;
                                size_t y2 = vertex2.y * zoomScale * dimensionY/2 + dimensionY/2*0;
                                draw_line(pixels, dimensionX, dimensionY, blue, x1, y1, x2, y2);
                            }
                        }


                        emit resultReady("rgb4", pixels, dimensionX, dimensionY, true);
                    }

                }
                else if (stream_type == RS2_STREAM_GYRO && format == RS2_FORMAT_MOTION_XYZ32F) {
                    auto motion = frame.as<rs2::motion_frame>();
                    double ts = motion.get_timestamp();
                    rs2_vector gyro_data = motion.get_motion_data();
                    rotation_estimator.process_gyro(gyro_data, ts);
                }
                else if (stream_type == RS2_STREAM_ACCEL && format == RS2_FORMAT_MOTION_XYZ32F) {
                    auto motion = frame.as<rs2::motion_frame>();
                    rs2_vector accel_data = motion.get_motion_data();
                    rotation_estimator.process_accel(accel_data);
                } else if (stream_type == RS2_STREAM_COLOR) {
                    auto color = frame.as<rs2::video_frame>();
                    auto frame_width = color.get_width();
                    auto frame_height = color.get_height();
                    auto frame_points_count = frame_width * frame_height;
                    auto color_data = (uint8_t*)color.get_data();
                    auto color_copy = new uint8_t[frame_points_count * 3];
                    memcpy(color_copy, color_data, frame_points_count * 3);
                    emit resultReady("label3", color_copy, frame_width, frame_height, true);
                }
            }

            auto theta = rotation_estimator.get_theta();
            float xrot_rad = theta.y - 180*PI/180;
            float xrot_deg = xrot_rad * 180/PI;
            emit statusBarResult(theta.toString() + " xrot = " + QString::number(xrot_deg));

        }

        delete[] pixels;

    }
public:
    WorkerThread(QObject* parent = nullptr):  QThread(parent) {}
    bool paused = false;
    float rotateA = 0;
    float rotateB = 0;
    float rotateC = 0;
    float zoomScale = 300;
    float sliceLevel = 0;
    float boxHeight = 0.4;
signals:
    void resultReady1(uint16_t* data, size_t width, size_t height, bool cleanup = false);
    void resultReady2(uint16_t* data, size_t width, size_t height, bool cleanup = false);
    void resultReady3(uint8_t* data, size_t width, size_t height, bool cleanup = false);
    void resultReady(QString, void* data, size_t width, size_t height, bool cleanup = false);
    void statusBarResult(QString);
    void debugText(QString);
};

#endif // UTILS_H
