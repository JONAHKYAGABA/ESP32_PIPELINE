#include "bg.h"
#include <iostream>

int main()
{
    std::vector<SensorRecord> sensorData;

    generateRandomSensorData(sensorData, 100);

    auto summaries = preprocess(sensorData);

    for (const auto &summary : summaries)
    {
        std::cout << "Total_acceleation_magnitude: " << summary.total_acc_mag
                  << "Gyroscope_Vector_Magnitude" << summary.gyro_vect_mag
                  << ", Total Acc Mag Min: " << summary.total_acc_mag_min
                  << ", Mean Gyro Energy: " << summary.mean_gyro_energy
                  << ", Mean Acc Energy: " << summary.mean_acc_energy
                  << ", Angle GX_GY: " << summary.angle_gx_gy
                  << ", gyro_z_std_dev " << summary.gyro_z_std_dev

                  << std::endl;
    }

    return 0;
}
