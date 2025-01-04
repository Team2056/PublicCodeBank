#pragma once
#include <frc/Counter.h>
#include <frc/smartdashboard/SmartDashboard.h>

class OPRLidar {
   public:
    OPRLidar();
    ~OPRLidar(void);

    static OPRLidar* GetInstance();
    void ShowOnDashboard();

    /**
     * @brief Retrieve raw value of lidar sensor
     *
     * @return raw value of lidar
     */
    double GetRawValue();

    /**
     * @brief Get the distance from the lidar to closest object
     *
     * @return distance
     */
    double GetDistance();

    /**
     * @brief Gets the linear distance
     *
     * @return linear distance
     */
    double GetLinearDistance();

   private:
    frc::Counter* lidar;
    const double offset = 12.0;
    static OPRLidar* instance;
};