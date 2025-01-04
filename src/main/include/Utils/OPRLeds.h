#pragma once

#include <frc/AddressableLED.h>

#include "../OPRIncludes.h"
#include "./IO/OPRControllers.h"
#include "./IO/OPRSensors.h"

struct ColourStruct {
    unsigned char R;
    unsigned char G;
    unsigned char B;
    int H;
    unsigned char S;
    unsigned char V;
};

class OPRLeds {
   public:
    static OPRLeds *GetInstance();

    void UpdateLEDTeleop();
    void UpdateLEDAuto();
    void UpdateLEDDisabled();
    void InitializeLEDController();
    void Rainbow();
    void Flash(struct ColourStruct Colour);
    void Solid(struct ColourStruct Colour);
    void Breathe(struct ColourStruct Colour);
    void Cylon();
    void ClearBuffer();
    void TeleopInit();

   private:
    OPRLeds();
    ~OPRLeds();
    static OPRLeds *instance;

    OPRControllers *GamePads;
    OPRCameras *_Cameras;
    OPRGyro * _Gyro;
    OPRSensors * _Sensors;

    frc::Timer *m_timer;
    
    static constexpr int kLength = 76;

    // Must be a PWM header, not MXP or DIO
    frc::AddressableLED m_led{LED_PORT};
    std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;  // Reuse the buffer
    
    // Store what the last hue of the first pixel is
    int firstPixelHue = 0;

    struct ColourStruct Blue {
        0, 0, 255, 120, 255, 255
    };
    struct ColourStruct Red {
        255, 0, 0, 0, 255, 255
    };
    struct ColourStruct Green {
        0, 255, 0, 60, 255, 128
    };
    struct ColourStruct White {
        255, 255, 128, 0, 0, 255
    };
    struct ColourStruct Yellow {
        255, 255, 0, 60, 255, 255
    };
    struct ColourStruct Magenta {
        255, 0, 255, 150, 255, 255
    };
    struct ColourStruct Grey {
        128, 128, 128, 0, 0, 128
    };
    struct ColourStruct LimeGreen {
        50, 205, 50, 60, 205, 50
    };
    struct ColourStruct Black {
        0, 0, 0, 0, 0, 0
    };
    struct ColourStruct Orange {
        255, 120, 0, 42, 255, 255
    };
};