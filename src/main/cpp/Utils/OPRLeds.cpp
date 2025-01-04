#include "./Utils/OPRLeds.h"

OPRLeds *OPRLeds::instance = NULL;

OPRLeds::OPRLeds() {
    m_timer = new frc::Timer();
    
    GamePads = OPRControllers::GetInstance();
    _Cameras = OPRCameras::GetInstance();
    _Sensors = OPRSensors::GetInstance();
    _Gyro = OPRGyro::GetInstance();
}  // Constructor

OPRLeds::~OPRLeds() {}

OPRLeds *OPRLeds::GetInstance() {
    if (instance == NULL) {
        instance = new OPRLeds();
    }
    return instance;
}  // GetInstance

void OPRLeds::InitializeLEDController() {
    m_led.SetLength(kLength);  // how many LEDs there are
    m_led.SetData(m_ledBuffer);
    m_led.Start();
    m_timer->Start();
}

void OPRLeds::UpdateLEDDisabled() {
    ClearBuffer();
    if (!_Cameras->AprilTag->IsConnected() || !_Cameras->GamePiece->IsConnected()) {
        Flash(Red);
    }
    else if (currAlliance == Alliances::RED) {
        Breathe(Red);
    }
    else if (currAlliance == Alliances::BLUE) {
        Breathe(Blue);
    }
    m_led.SetData(m_ledBuffer);
}

void OPRLeds::UpdateLEDAuto() {
    ClearBuffer();
    if (!_Cameras->AprilTag->IsConnected() || !_Cameras->GamePiece->IsConnected()) {
        Flash(Red);
    }

    m_led.SetData(m_ledBuffer);
}

void OPRLeds::TeleopInit() {
    ClearBuffer();
    Solid(Black);
    m_led.SetData(m_ledBuffer);
}

void OPRLeds::UpdateLEDTeleop() {
    ClearBuffer();

    double BSNumber = fabs(_Cameras->AprilTag->GetDistance() * cos(_Gyro->GetAngle().value() * DEG_TO_RAD));

    GamePads->Driver->SetRumble(0);
    if (!_Cameras->AprilTag->IsConnected() || !_Cameras->GamePiece->IsConnected()) {
        Flash(Red);
    }
    else if (GamePads->Driver->GetButton(D_INTAKE_ON)) {
        Flash(Orange);
    }
    m_led.SetData(m_ledBuffer);
}

/**
 * @brief Moving rainbow
 * 
 */
void OPRLeds::Rainbow() {
    for (int i = 0; i < kLength; i++) {
        const auto pixelHue = (firstPixelHue + (i * 100 / kLength)) % 180;
        m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
    }

    firstPixelHue += 1;
    firstPixelHue %= 180;
}

/**
 * @brief Breathe all LEDs one colour
 * 
 * @param Colour 
 */
void OPRLeds::Breathe(struct ColourStruct Colour) {
    static int j;
    unsigned char v;

    if ((double)m_timer->Get() > 0.015) {
        m_timer->Reset();
        m_timer->Start();
        j++;
    }
    if (j >= 360) {
        j = 0;
    }
    if ((j > 60) && (j < 300)) {
        j = 300;
    }
    v = (int)fabs(100 * sin(DEG_TO_RAD * j));
    for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetHSV(Colour.H, Colour.S, v);
    }
}

/**
 * @brief Flash all LEDs one colour ever 0.2s
 * 
 * @param Colour 
 */
void OPRLeds::Flash(struct ColourStruct Colour) {
    if ((double)m_timer->Get() > 0.2) {
        m_timer->Reset();
        m_timer->Start();
    }
    if ((double)m_timer->Get() < 0.1) {
        for (int i = 0; i < kLength; i++) {
            m_ledBuffer[i].SetRGB(Colour.R, Colour.G, Colour.B);
        }
    }
    else {
        for (int i = 0; i < kLength; i++) {
            m_ledBuffer[i].SetRGB(0, 0, 0);
        }
    }
}

/**
 * @brief Set all LEDs to one colour
 * 
 * @param Colour 
 */
void OPRLeds::Solid(struct ColourStruct Colour) {
    for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(Colour.R, Colour.G, Colour.B);
    }
}

/**
 * @brief clear the leds
 * 
 */
void OPRLeds::ClearBuffer() {
    for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(0, 0, 0);
    }
}

/**
 * @brief bouncing red LED
 * 
 */
void OPRLeds::Cylon() {
    static int redPos = 1;
    static int j = 2;

    if (m_timer->Get() > 40_ms) {
        m_timer->Reset();
        m_timer->Start();
        j++;
    }

    if (j >= kLength) {
        j = 2;
    }

    redPos = (int)kLength * (1.0 + cos((j * 360.0 / kLength) * DEG_TO_RAD)) / 2;
    if (redPos < 2) {
        redPos = 2;
    }
    else if (redPos > kLength - 2) {
        redPos = kLength - 2;
    }

    for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(0, 0, 0);
    }

    m_ledBuffer[redPos - 2].SetHSV(0, 255, 32);
    m_ledBuffer[redPos - 1].SetHSV(0, 255, 128);
    m_ledBuffer[redPos].SetHSV(0, 255, 255);
    m_ledBuffer[redPos + 1].SetHSV(0, 255, 128);
    m_ledBuffer[redPos + 2].SetHSV(0, 255, 32);
}