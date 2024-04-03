#ifndef BG_H
#define BG_H

#include <vector>

struct SensorRecord {
    double time;
    double seconds_elapsed;
    double accelerometer_z, accelerometer_y, accelerometer_x;
    double gyroscope_z, gyroscope_y, gyroscope_x;
    double total_acceleration_magnitude;
    double gyroscope_vector_magnitude;
};

struct Window {
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

struct WindowSummary {
    
    double total_acc_mag;
    double gyro_vect_mag;
    double total_acc_mag_min;
    double mean_gyro_energy;
    double mean_acc_energy;
    double angle_gx_gy;
    double gyro_z_std_dev;  
    
};

void generateRandomSensorData(std::vector<SensorRecord>& records, int numberOfRecords);
std::vector<Window> applySlidingWindowWithTiming(const std::vector<SensorRecord>& data, int sampling_rate = 98, int window_duration = 20, double overlap_fraction = 0.5, int start_window_id = 0);
void addStatisticsToWindows(std::vector<Window>& windows);
void addEnergyToWindows(std::vector<Window>& windows);
void addGyroscopeAngleFeature(std::vector<Window>& windows);
std::vector<WindowSummary> preprocess(const std::vector<SensorRecord>& sensorData);

#endif 
