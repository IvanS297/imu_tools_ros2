#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include "imu_tools/serialib.h"
#include <unistd.h>
#include <string>
#include <stdexcept>
#include <sstream>
#include <vector>
#include <chrono>
#include <memory>
#include <iostream>
#include <numeric>
#include <limits>
#include <thread>
#include <stop_token>
#include <fstream>

class imu_calibrator : public rclcpp::Node
{
public:
    imu_calibrator() : Node("imu_calibrator") {
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("timer_interval_ms", 100);
        this->declare_parameter("buffer_size", 255);
        this->declare_parameter("gyro_samples", 1000);

        serial_port_      = this->get_parameter("serial_port").as_string();
        baud_rate_        = this->get_parameter("baud_rate").as_int();
        timer_interval_   = this->get_parameter("timer_interval_ms").as_int();
        buffer_size_      = this->get_parameter("buffer_size").as_int();
        gyro_samples_     = this->get_parameter("gyro_samples").as_int();

        RCLCPP_INFO(this->get_logger(), "Node has been started.");

        serial_init();
        serial.flushReceiver();
        menu_thread_ = std::thread(&imu_calibrator::menu_loop, this);
    }

    ~imu_calibrator() {
        if (menu_thread_.joinable()) {
            menu_thread_.join();
            close_serial();
        }
    }

private:
    int serial_init();
    void close_serial();
    void menu_loop();
    std::vector<double> read_data();
    std::vector<double> calibrate_gyro(const std::vector<double>& gyro_data);
    std::vector<double> calibrate_accel(const std::vector<double>& accel_data);
    std::vector<double> calibrate_mag(const std::vector<double>& mag_data);
    void save_results(
        const std::vector<double>& g_cal,
        const std::vector<double>& a_cal,
        const std::vector<double>& m_cal
    );

    std::thread menu_thread_;

    serialib serial;

    std::string serial_port_;
    int baud_rate_;
    int timer_interval_;
    int buffer_size_;
    int gyro_samples_;
    std::vector<double> gyro_offsets_;   // 3 значения: x y z
    std::vector<double> accel_calib_;    // 6 значений: off_x y z, sc_x y z
    std::vector<double> mag_calib_;      // 6 значений: off_x y z, sc_x y z
};

int imu_calibrator::serial_init() {
    char error_opening = serial.openDevice(serial_port_.c_str(), baud_rate_);
    if (error_opening != 1) {
        RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %d", error_opening);
        return error_opening;
    }
    RCLCPP_INFO(this->get_logger(), "Serial port opened successfully."); 
    return 0;
}

void imu_calibrator::close_serial() {
    serial.closeDevice();
    RCLCPP_INFO(this->get_logger(), "Serial port closed.");
}

std::vector<double> imu_calibrator::read_data() {
    try {
        char raw[256] = {0};
        int result = serial.readString(raw, '\n', buffer_size_, 10); // буфер buffer_size_, таймаут 1000мс
        if (result <= 0) {
            //RCLCPP_WARN(this->get_logger(), "readString returned %d", result);
            return {};
        }

        std::string data(raw);
        // убираем \r\n и пробелы по краям
        while (!data.empty() && (data.back() == '\n' || data.back() == '\r' || data.back() == ' '))
            data.pop_back();

        //RCLCPP_INFO(this->get_logger(), "Raw data read: %s", data.c_str());

        std::vector<double> values;
        std::stringstream ss(data);
        std::string token;

        while (std::getline(ss, token, ',')) {
            if (!token.empty())
                values.push_back(std::stod(token));
        }

        if (values.size() != 9) {
            //RCLCPP_WARN(this->get_logger(), "Expected 9 values, got %zu", values.size());
            return {};
        }

        return values; // ax,ay,az,gx,gy,gz,mx,my,mz
    }
    catch (const std::exception& e) {
        //RCLCPP_ERROR(this->get_logger(), "Error reading data: %s", e.what());
        return {};
    }
}

std::vector<double> imu_calibrator::calibrate_gyro(const std::vector<double>& gyro_data) {
    RCLCPP_INFO(this->get_logger(), std::string(60, '=').c_str());
    RCLCPP_INFO(this->get_logger(), "Calibration gyro");
    RCLCPP_INFO(this->get_logger(), "Put down the IMU and do not move it until the end of calibration");
    RCLCPP_INFO(this->get_logger(), "Calibration will take %d values", gyro_samples_);

    // сначала выводим сообщение, потом читаем
    std::cout << "Press Enter to start, or type anything + Enter to cancel: " << std::flush;
    std::string input;
    if (!std::getline(std::cin, input)) {
        return {};
    }

    if (!input.empty()) {
        RCLCPP_INFO(this->get_logger(), "Calibration cancelled");
        return {};
    }

    RCLCPP_INFO(this->get_logger(), "Calibration started");
    serial.flushReceiver();

    uint64_t count = 0;
    std::vector<std::vector<double>> values;
    while (count < (uint64_t)gyro_samples_) {
        std::vector<double> data = read_data();
        if (!data.empty()) {
            values.push_back({data[3], data[4], data[5]});
            count++;
            if (count % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), "Progress: %lu/%d", count, gyro_samples_);
            }
        }
    }

    double offset_x = std::accumulate(values.begin(), values.end(), 0.0,
        [](double sum, const std::vector<double>& g) { return sum + g[0]; }) / values.size();
    double offset_y = std::accumulate(values.begin(), values.end(), 0.0,
        [](double sum, const std::vector<double>& g) { return sum + g[1]; }) / values.size();
    double offset_z = std::accumulate(values.begin(), values.end(), 0.0,
        [](double sum, const std::vector<double>& g) { return sum + g[2]; }) / values.size();

    RCLCPP_INFO(this->get_logger(), "Calibration results, gyro offsets (rad/s):");
    RCLCPP_INFO(this->get_logger(), "Offset X: %f", offset_x);
    RCLCPP_INFO(this->get_logger(), "Offset Y: %f", offset_y);
    RCLCPP_INFO(this->get_logger(), "Offset Z: %f", offset_z);

    return {offset_x, offset_y, offset_z};
}

std::vector<double> imu_calibrator::calibrate_accel(const std::vector<double>& accel_data) {
    RCLCPP_INFO(this->get_logger(), std::string(60, '=').c_str());
    RCLCPP_INFO(this->get_logger(), "Calibrating accelerometer");
    RCLCPP_INFO(this->get_logger(), "Rotate the IMU slowly in all axes (full 3D rotation).");
    RCLCPP_INFO(this->get_logger(), "Each axis must be directed up and down.");
    RCLCPP_INFO(this->get_logger(), "Press Enter again when you are finished rotating.");

    // x y z
    std::vector<double> mn = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    // x y z
    std::vector<double> mx = {-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
    uint64_t count = 0;
    serial.flushReceiver();
    RCLCPP_INFO(this->get_logger(), "Collecting data...");

    // ждем подтверждения
    std::stop_source stop_flag;
    auto wait_enter = [&stop_flag]() {      
        std::cout << "Press Enter to stop collecting..." << std::endl;
        std::string input;
        std::getline(std::cin, input);
        stop_flag.request_stop();
    };
    std::jthread t(wait_enter);

    // собираем, пока не нажмут enter
    while (!stop_flag.get_token().stop_requested()) {
        std::vector<double> data = read_data();
        if (data.empty()) continue;
        
        double ax = data[0];
        double ay = data[1];
        double az = data[2];
        mn.at(0) = std::min(mn.at(0), ax);
        mx.at(0) = std::max(mx.at(0), ax);
        mn.at(1) = std::min(mn.at(1), ay);
        mx.at(1) = std::max(mx.at(1), ay);
        mn.at(2) = std::min(mn.at(2), az);
        mx.at(2) = std::max(mx.at(2), az);
        count++;
        if (count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Progress: %lu samples | X: [%3f, %3f] | Y: [%3f, %3f] | Z: [%3f, %3f]", count, mn.at(0), mx.at(0), mn.at(1), mx.at(1), mn.at(2), mx.at(2));
        }
    }
    t.join();

    double off_x = (mn.at(0) + mx.at(0)) / 2.0;
    double off_y = (mn.at(1) + mx.at(1)) / 2.0;
    double off_z = (mn.at(2) + mx.at(2)) / 2.0;

    double sc_x = (mx.at(0) - mn.at(0)) / 2.0;
    double sc_y = (mx.at(1) - mn.at(1)) / 2.0;
    double sc_z = (mx.at(2) - mn.at(2)) / 2.0;

    RCLCPP_INFO(this->get_logger(), "\n[RESULTS] Acc offsets (m/s^2):");
    RCLCPP_INFO(this->get_logger(), "Offset X: %f | Scale X: %f", off_x, sc_x);
    RCLCPP_INFO(this->get_logger(), "Offset Y: %f | Scale Y: %f", off_y, sc_y);
    RCLCPP_INFO(this->get_logger(), "Offset Z: %f | Scale Z: %f", off_z, sc_z);

    double avg_range = (sc_x + sc_y + sc_z) / 3.0 * 2.0;
    if (avg_range < 5.0 || avg_range > 25.0) {
        RCLCPP_WARN(this->get_logger(), "Average range is %f, which looks strange.", avg_range);
        RCLCPP_WARN(this->get_logger(), "Please make sure you rotated the IMU correctly and try again.");
    }
    return {off_x, off_y, off_z, sc_x, sc_y, sc_z};
}

std::vector<double> imu_calibrator::calibrate_mag(const std::vector<double>& mag_data) {
    RCLCPP_INFO(this->get_logger(), std::string(60, '=').c_str());
    RCLCPP_INFO(this->get_logger(), "Calibrating magnetometer");
    RCLCPP_INFO(this->get_logger(), "Rotate the IMU in a figure-eight pattern, directing each axis");
    RCLCPP_INFO(this->get_logger(), "alternately to the magnetic north, making full 360° turns.");
    RCLCPP_INFO(this->get_logger(), "Press Enter again when you are finished.");
    RCLCPP_INFO(this->get_logger(), "Press Enter to start collecting data...");

    // x y z
    std::vector<double> mn = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    // x y z
    std::vector<double> mx = {-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
    uint64_t count = 0;
    serial.flushReceiver();
    RCLCPP_INFO(this->get_logger(), "Collecting data... (Press Enter to stop)");

    // ждем подтверждения
    std::stop_source stop_flag;
    auto wait_enter = [&stop_flag]() {      
        std::cout << "Press Enter to stop collecting..." << std::endl;
        std::string input;
        std::getline(std::cin, input);
        stop_flag.request_stop();
    };
    std::jthread t(wait_enter);
    while (!stop_flag.get_token().stop_requested()) {
        std::vector<double> data = read_data();
        if (data.empty()) continue;
        double mx_v = data[6];
        double my_v = data[7];
        double mz_v = data[8];
        mn.at(0) = std::min(mn.at(0), mx_v);
        mx.at(0) = std::max(mx.at(0), mx_v);
        mn.at(1) = std::min(mn.at(1), my_v);
        mx.at(1) = std::max(mx.at(1), my_v);
        mn.at(2) = std::min(mn.at(2), mz_v);
        mx.at(2) = std::max(mx.at(2), mz_v);
        count++;
        if (count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Progress: %lu samples | X: [%3f, %3f] | Y: [%3f, %3f] | Z: [%3f, %3f]", count, mn.at(0), mx.at(0), mn.at(1), mx.at(1), mn.at(2), mx.at(2));
        }
    }
    t.join();
    double off_x = (mn.at(0) + mx.at(0)) / 2.0;
    double off_y = (mn.at(1) + mx.at(1)) / 2.0;
    double off_z = (mn.at(2) + mx.at(2)) / 2.0;

    double range_x = (mn.at(0) - mx.at(0));
    double range_y = (mn.at(1) - mx.at(1));
    double range_z = (mn.at(2) - mx.at(2));

    double sc_x, sc_y, sc_z;

    if (range_x > 1e-12) sc_x = 2.0 / range_x; else sc_x = 1.0;
    if (range_y > 1e-12) sc_y = 2.0 / range_y; else sc_y = 1.0;
    if (range_z > 1e-12) sc_z = 2.0 / range_z; else sc_z = 1.0;

    RCLCPP_INFO(this->get_logger(), "\n[RESULTS] Mag offsets and scales:");
    RCLCPP_INFO(this->get_logger(), "Offset X: %f | Scale X: %f", off_x, sc_x);
    RCLCPP_INFO(this->get_logger(), "Offset Y: %f | Scale Y: %f", off_y, sc_y);
    RCLCPP_INFO(this->get_logger(), "Offset Z: %f | Scale Z: %f", off_z, sc_z);

    return {off_x, off_y, off_z, sc_x, sc_y, sc_z};
}

void imu_calibrator::save_results(
    const std::vector<double>& g_cal,
    const std::vector<double>& a_cal,
    const std::vector<double>& m_cal
) {
    // Проверка корректности размеров векторов перед обработкой
    if (g_cal.size() < 3 || a_cal.size() < 6 || m_cal.size() < 6) {
        RCLCPP_ERROR(this->get_logger(), "Wrong size of calibration vectors!");
        return;
    }

    // Извлекаем значения для гироскопа (размер 3)
    double gx_off = g_cal[0], gy_off = g_cal[1], gz_off = g_cal[2];

    // Извлекаем значения для акселерометра (размер 6)
    double ax_off = a_cal[0], ax_sc = a_cal[1], ay_off = a_cal[2];
    double ay_sc  = a_cal[3], az_off = a_cal[4], az_sc  = a_cal[5];

    // Извлекаем значения для магнетометра (размер 6)
    double mx_off = m_cal[0], mx_sc = m_cal[1], my_off = m_cal[2];
    double my_sc  = m_cal[3], mz_off = m_cal[4], mz_sc  = m_cal[5];

    // --- gCal.txt ---
    if (std::ofstream f("gCal.txt"); f.is_open()) {
        f << std::format("{},{},{}\n", gx_off, gy_off, gz_off);
        RCLCPP_INFO(this->get_logger(), "gCal.txt saved");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Can't open gCal.txt for writing!");
    }

    // --- accCal.txt ---
    if (std::ofstream f("accCal.txt"); f.is_open()) {
        f << std::format("{},{},{},{},{},{}\n", ax_off, ax_sc, ay_off, ay_sc, az_off, az_sc);
        RCLCPP_INFO(this->get_logger(), "accCal.txt saved");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Can't open accCal.txt for writing!");
    }

    // --- magCal.txt ---
    if (std::ofstream f("magCal.txt"); f.is_open()) {
        f << std::format("{},{},{},{},{},{}\n", mx_off, mx_sc, my_off, my_sc, mz_off, mz_sc);
        RCLCPP_INFO(this->get_logger(), "magCal.txt saved");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Can't open magCal.txt for writing!");
    }

    // --- calData.txt (Arduino функция) ---
    if (std::ofstream f("calData.txt"); f.is_open()) {
        std::string arduino_code = std::format(
R"(// Автоматически сгенерировано IMU Calibrator
// Формулы: https://github.com

void applyCal() {{
    // --- Акселерометр (результат в g) ---
    const float ax_off = {:.8f};
    const float ay_off = {:.8f};
    const float az_off = {:.8f};
    const float ax_sc  = {:.8f};
    const float ay_sc  = {:.8f};
    const float az_sc  = {:.8f};

    // --- Гироскоп (рад/с) ---
    const float gx_off = {:.8f};
    const float gy_off = {:.8f};
    const float gz_off = {:.8f};

    // --- Магнетометр (нормализован в [-1,+1]) ---
    const float mx_off = {:.8e};
    const float my_off = {:.8e};
    const float mz_off = {:.8e};
    const float mx_sc  = {:.8e};
    const float my_sc  = {:.8e};
    const float mz_sc  = {:.8e};

    AxCal = (AxRaw - ax_off) / ax_sc;
    AyCal = (AyRaw - ay_off) / ay_sc;
    AzCal = (AzRaw - az_off) / az_sc;

    GxCal = GxRaw - gx_off;
    GyCal = GyRaw - gy_off;
    GzCal = GzRaw - gz_off;

    MxCal = (MxRaw - mx_off) * mx_sc;
    MyCal = (MyRaw - my_off) * my_sc;
    MzCal = (MzRaw - mz_off) * mz_sc;
}}
)", 
        ax_off, ay_off, az_off, ax_sc, ay_sc, az_sc,
        gx_off, gy_off, gz_off,
        mx_off, my_off, mz_off, mx_sc, my_sc, mz_sc
        );

        f << arduino_code;
        RCLCPP_INFO(this->get_logger(), "calData.txt saved with Arduino code");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Can't open calData.txt for writing!");
    }

    // --- Итоговый вывод ---
    // Собираем весь большой текстовый отчет в одну строку с помощью std::format
    std::string report = std::format(
        "\n{}\n"
        "FINAL RESULTS\n"
        "{}\n"
        "\nGyroscope (bias, rad/s):\n"
        "  gx_offset = {:.8f}\n"
        "  gy_offset = {:.8f}\n"
        "  gz_offset = {:.8f}\n"
        "\nAccelerometer (offset m/s², scale m/s²):\n"
        "  ax: offset={:.8f}, scale={:.8f}\n"
        "  ay: offset={:.8f}, scale={:.8f}\n"
        "  az: offset={:.8f}, scale={:.8f}\n"
        "\nMagnetometer (offset T, scale 1/T):\n"
        "  mx: offset={:.4e}, scale={:.4e}\n"
        "  my: offset={:.4e}, scale={:.4e}\n"
        "  mz: offset={:.4e}, scale={:.4e}\n"
        "\nFiles: gCal.txt, accCal.txt, magCal.txt, calData.txt",
        std::string(60, '='),
        std::string(60, '='),
        gx_off, gy_off, gz_off,
        ax_off, ax_sc, ay_off, ay_sc, az_off, az_sc,
        mx_off, mx_sc, my_off, my_sc, mz_off, mz_sc
    );

    // Выводим сгенерированный отчет одной командой
    RCLCPP_INFO(this->get_logger(), "%s", report.c_str());
}


void imu_calibrator::menu_loop() {
    std::string choice;
    std::vector<double> g_cal, a_cal, m_cal;

    while (rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Type '1' to start gyro calibration");
        RCLCPP_INFO(this->get_logger(), "Type '2' to start accel calibration");
        RCLCPP_INFO(this->get_logger(), "Type '3' to start mag calibration");
        RCLCPP_INFO(this->get_logger(), "Type 'save' to save results to files");
        RCLCPP_INFO(this->get_logger(), "Type 'exit' to quit");

        if (!std::getline(std::cin, choice)) break;

        if (choice == "1") {
            RCLCPP_INFO(this->get_logger(), "Starting gyro calibration...");
            gyro_offsets_ = calibrate_gyro(read_data());
            RCLCPP_INFO(this->get_logger(), "Calibration done!");
        } else if (choice == "2") {
            RCLCPP_INFO(this->get_logger(), "Starting accel calibration...");
            accel_calib_ = calibrate_accel(read_data());
            RCLCPP_INFO(this->get_logger(), "Calibration done!");
        } else if (choice == "3") {
            RCLCPP_INFO(this->get_logger(), "Starting mag calibration...");
            mag_calib_ = calibrate_mag(read_data());
            RCLCPP_INFO(this->get_logger(), "Calibration done!");
        } else if (choice == "save") {
            RCLCPP_INFO(this->get_logger(), "Saving results...");
            save_results(gyro_offsets_, accel_calib_, mag_calib_);
            RCLCPP_INFO(this->get_logger(), "Results saved!");
        } else if (choice == "exit") {
            RCLCPP_INFO(this->get_logger(), "Exiting...");
            rclcpp::shutdown();
            break;
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<imu_calibrator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}