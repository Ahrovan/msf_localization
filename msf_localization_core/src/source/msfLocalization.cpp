
#include "msfLocalization.h"





StateEstimationCore::StateEstimationCore() :
    flagHasRobotState(false),
    flagHasSensorState(false),
    flagHasMeasurement(false)
{
    return;
}

StateEstimationCore::~StateEstimationCore()
{
    return;
}












MsfLocalizationCore::MsfLocalizationCore()
{

    return;
}

MsfLocalizationCore::~MsfLocalizationCore()
{
    return;
}


int MsfLocalizationCore::init()
{
    return 0;
}

int MsfLocalizationCore::close()
{
    return 0;
}

int MsfLocalizationCore::open()
{
    return 0;
}

int MsfLocalizationCore::run()
{
    return 0;
}


int MsfLocalizationCore::setPredictEnabled(bool predictEnabled)
{
    this->predictEnabled=predictEnabled;
    return 0;
}

int MsfLocalizationCore::predictThreadFunction()
{
    while(predictEnabled)
    {
        // Predict

        // Robot


        // Sensors


        // Sleep

    }

    return 0;
}
