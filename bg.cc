#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <numeric>
#include <limits>

struct SensorRecord
{
    double time;
    double seconds_elapsed;
    double accelerometer_z, accelerometer_y, accelerometer_x;
    double gyroscope_z, gyroscope_y, gyroscope_x;
    double total_acceleration_magnitude;
    double gyroscope_vector_magnitude;
};

struct Window
{
    std::vector<SensorRecord> records;
    int window_id;
    double total_acc_mag;
    double gyro_vect_mag;
    double total_acc_mag_min;
    double mean_gyro_energy;
    double mean_acc_energy;
    double angle_gx_gy;
    double gyro_z_std_dev;
};

struct WindowSummary
{

    double total_acc_mag;
    double gyro_vect_mag;
    double total_acc_mag_min;
    double mean_gyro_energy;
    double mean_acc_energy;
    double angle_gx_gy;
    double gyro_z_std_dev;
};

double calculateMagnitude(double x, double y, double z)
{
    return std::sqrt(x * x + y * y + z * z);
}

void addMagnitudes(std::vector<SensorRecord> &records)
{
    for (auto &record : records)
    {
        record.total_acceleration_magnitude = calculateMagnitude(record.accelerometer_x, record.accelerometer_y, record.accelerometer_z);
        record.gyroscope_vector_magnitude = calculateMagnitude(record.gyroscope_x, record.gyroscope_y, record.gyroscope_z);
    }
}

void generateRandomSensorData(std::vector<SensorRecord> &records, int numberOfRecords)
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    for (int i = 0; i < numberOfRecords; ++i)
    {
        SensorRecord record;
        record.time = static_cast<double>(i);
        record.seconds_elapsed = static_cast<double>(i) / 98.0;
        record.accelerometer_x = static_cast<double>(rand() % 100) / 10.0 - 5.0;
        record.accelerometer_y = static_cast<double>(rand() % 100) / 10.0 - 5.0;
        record.accelerometer_z = static_cast<double>(rand() % 100) / 10.0 - 5.0;
        record.gyroscope_x = static_cast<double>(rand() % 100) / 10.0 - 5.0;
        record.gyroscope_y = static_cast<double>(rand() % 100) / 10.0 - 5.0;
        record.gyroscope_z = static_cast<double>(rand() % 100) / 10.0 - 5.0;
        records.push_back(record);
    }
}
void addEnergyToWindows(std::vector<Window> &windows)
{
    for (auto &window : windows)
    {
        double acc_energy_sum = 0, gyro_energy_sum = 0;

        for (const auto &record : window.records)
        {
            acc_energy_sum += std::pow(record.accelerometer_x, 2) + std::pow(record.accelerometer_y, 2) + std::pow(record.accelerometer_z, 2);
            gyro_energy_sum += std::pow(record.gyroscope_x, 2) + std::pow(record.gyroscope_y, 2) + std::pow(record.gyroscope_z, 2);
        }

        double mean_acc_energy = acc_energy_sum / window.records.size();
        double mean_gyro_energy = gyro_energy_sum / window.records.size();

        window.mean_acc_energy = mean_acc_energy;
        window.mean_gyro_energy = mean_gyro_energy;

        // std::cout << "Window ID: " << window.window_id
        //           << " Mean Acc Energy: " << mean_acc_energy
        //           << " Mean Gyro Energy: " << mean_gyro_energy << std::endl;
    }
}
std::vector<Window> applySlidingWindowWithTiming(const std::vector<SensorRecord> &data, int sampling_rate = 98, int window_duration = 20, double overlap_fraction = 0.5, int start_window_id = 0)
{
    int window_size = sampling_rate * window_duration;
    int overlap = static_cast<int>(window_size * overlap_fraction);
    std::vector<Window> windows;

    int window_id = start_window_id;
    size_t start_index = 0;
    double end_time = data.back().seconds_elapsed;

    while (start_index < data.size())
    {
        size_t end_index = start_index;
        double window_end_time = data[start_index].seconds_elapsed + window_duration;
        while (end_index < data.size() && data[end_index].seconds_elapsed < window_end_time)
        {
            ++end_index;
        }

        if (end_index > start_index)
        {
            std::vector<SensorRecord> windowRecords(data.begin() + start_index, data.begin() + end_index);
            windows.push_back({windowRecords, window_id++});
        }

        start_index += overlap;
        if (start_index >= data.size() || data[start_index].seconds_elapsed + window_duration > end_time)
        {
            break;
        }
    }

    return windows;
}

void addStatisticsToWindows(std::vector<Window> &windows)
{
    for (auto &window : windows)
    {
        double gyro_z_sum = 0, total_acc_mag_sum = 0, total_gyro_vect_mag_sum = 0;
        double gyro_z_min = std::numeric_limits<double>::max(), total_acc_mag_min = std::numeric_limits<double>::max();
        int count = window.records.size();

        for (const auto &record : window.records)
        {
            gyro_z_sum += record.gyroscope_z;
            total_acc_mag_sum += record.total_acceleration_magnitude;
            total_gyro_vect_mag_sum += record.gyroscope_vector_magnitude;

            if (record.gyroscope_z < gyro_z_min)
                gyro_z_min = record.gyroscope_z;
            if (record.total_acceleration_magnitude < total_acc_mag_min)
                total_acc_mag_min = record.total_acceleration_magnitude;
        }

        double gyro_z_mean = gyro_z_sum / count;
        double total_acc_mag_mean = total_acc_mag_sum / count;
        double gyro_z_variance = 0, total_acc_mag_variance = 0;

        for (const auto &record : window.records)
        {
            gyro_z_variance += std::pow(record.gyroscope_z - gyro_z_mean, 2);
            total_acc_mag_variance += std::pow(record.total_acceleration_magnitude - total_acc_mag_mean, 2);
        }

        double gyro_z_std_dev = std::sqrt(gyro_z_variance / count);
        double total_acc_mag_std_dev = std::sqrt(total_acc_mag_variance / count);

        window.gyro_z_std_dev = gyro_z_std_dev;
        window.total_acc_mag = total_acc_mag_mean;
        window.gyro_vect_mag = total_gyro_vect_mag_sum / count;
        window.total_acc_mag_min = total_acc_mag_min;
    }
}

double dotProduct(const std::vector<double> &v1, const std::vector<double> &v2)
{
    double sum = 0.0;
    for (size_t i = 0; i < v1.size() && i < v2.size(); ++i)
    {
        sum += v1[i] * v2[i];
    }
    return sum;
}

double magnitude(const std::vector<double> &v)
{
    return std::sqrt(dotProduct(v, v));
}

double angleBetweenVectors(const std::vector<double> &v1, const std::vector<double> &v2)
{
    double dot = dotProduct(v1, v2);
    double magV1 = magnitude(v1);
    double magV2 = magnitude(v2);
    double cosTheta = dot / (magV1 * magV2);
    return cosTheta;
}
void addGyroscopeAngleFeature(std::vector<Window> &windows)
{
    for (auto &window : windows)
    {
        std::vector<double> gyroX, gyroY;

        for (const auto &record : window.records)
        {
            gyroX.push_back(record.gyroscope_x);
            gyroY.push_back(record.gyroscope_y);
        }

        double angleGxGy = angleBetweenVectors(gyroY, gyroX);
        window.angle_gx_gy = angleGxGy;

        // std::cout << "Window ID: " << window.window_id
        //           << " Angle between Gyro Y and X: " << window.angle_gx_gy << " radians" << std::endl;
    }
}
std::vector<WindowSummary> preprocess(const std::vector<SensorRecord> &sensorData)
{

    auto windows = applySlidingWindowWithTiming(sensorData, 98, 20, 0.5, 0);

    addStatisticsToWindows(windows);
    addEnergyToWindows(windows);
    addGyroscopeAngleFeature(windows);

    std::vector<WindowSummary> summaries;
    for (const auto &window : windows)
    {
        WindowSummary summary = {
            window.total_acc_mag,
            window.gyro_vect_mag,
            window.total_acc_mag_min,
            window.mean_gyro_energy,
            window.mean_acc_energy,
            window.angle_gx_gy,
            window.gyro_z_std_dev,
        };
        summaries.push_back(summary);
    }

    return summaries;
}

// int main() {
//     std::vector<SensorRecord> sensorData;
//     generateRandomSensorData(sensorData, 3000);

//     auto summaries = preprocess(sensorData);

//     for (size_t i = 0; i < summaries.size(); ++i) {
//         std::cout << "Window ID: " << i
//                   << ", Gyro Z Std Dev: " << summaries[i].gyro_z_std_dev
//                   << ", Total Acc Mag Std Dev: " << summaries[i].total_acc_mag_std_dev
//                   << ", Gyro Z Min: " << summaries[i].gyro_z_min
//                   << ", Total Acc Mag Min: " << summaries[i].total_acc_mag_min
//                   << ", Mean Acc Energy: " << summaries[i].mean_acc_energy
//                   << ", Mean Gyro Energy: " << summaries[i].mean_gyro_energy
//                   << ", Angle GX_GY: " << summaries[i].angle_gx_gy
//                   << std::endl;
//     }

//     return 0;
// }
