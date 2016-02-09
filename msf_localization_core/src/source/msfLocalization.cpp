
#include "msfLocalization.h"











MsfLocalizationCore::MsfLocalizationCore()
{
    // Create Storage Core
    TheStateEstimationCore=std::make_shared<MsfStorageCore>();

    firstAvailableId=0;

    return;
}

MsfLocalizationCore::~MsfLocalizationCore()
{
    // Cleaning

    // TheListOfSensorCore
//    for(std::list<SensorCore*>::iterator it=TheListOfSensorCore.begin(); it!=TheListOfSensorCore.end(); ++it)
//        delete *it;
    TheListOfSensorCore.clear();



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
