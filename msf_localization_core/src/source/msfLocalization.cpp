
#include "msfLocalization.h"











MsfLocalizationCore::MsfLocalizationCore()
{
    // Flags
    predictEnabled=false;

    // Create Robot Core
    TheRobotCore=std::make_shared<RobotCore>();


    // Sensors
    //Id
    firstAvailableId=0;


    // Create Storage Core
    TheMsfStorageCore=std::make_shared<MsfStorageCore>();

    return;
}

MsfLocalizationCore::~MsfLocalizationCore()
{
    // Cleaning

    // TheListOfSensorCore
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
    std::cout<<"MsfLocalizationCore::predictThreadFunction()"<<std::endl;

    while(predictEnabled)
    {
        // TODO FINISH!

        // Predict
        TimeStamp TheTimeStamp;
        this->predict(TheTimeStamp);


        // Sleep
        //std::this_thread::sleep_until();
    }

    return 0;
}



int MsfLocalizationCore::bufferManagerThreadFunction()
{

    return 0;
}


int MsfLocalizationCore::predict(TimeStamp TheTimeStamp)
{
    //std::cout<<"MsfLocalizationCore::predict()"<<std::endl;

    // New element that is going to be added to the buffer
    StateEstimationCore PredictedState;


    // Get the last state from the buffer
    TimeStamp PreviousTimeStamp;
    StateEstimationCore PreviousState;
    TheMsfStorageCore->getLastElementWithStateEstimate(PreviousTimeStamp, PreviousState);

    /*
    for(int iElement=0; iElement<TheMsfStorageCore->getSize(); iElement++)
    {
        TheMsfStorageCore->getElementI(PreviousState, iElement);
        if(PreviousState.object.hasState())
        {
            std::cout<<"found!"<<std::endl;
            break;
        }
    }
    */

    // Check
    if(!PreviousState.hasState())
    {
        std::cout<<"!!error finding previous state"<<std::endl;
        return -1;
    }



    /////// State

    ///// Robot
    if(TheRobotCore->getRobotType() == RobotTypes::free_model)
    {
        std::shared_ptr<FreeModelRobotStateCore> predictedStateRobot;
        std::shared_ptr<FreeModelRobotStateCore> pastStateRobot=std::static_pointer_cast<FreeModelRobotStateCore>(PreviousState.TheRobotStateCore);

        if(!pastStateRobot)
        {
            std::cout<<"!!previous state for robot not found!"<<std::endl;
            return -2;
        }


        // Polymorphic
        std::shared_ptr<FreeModelRobotCore> TheFreeModelRobotCore=std::dynamic_pointer_cast<FreeModelRobotCore>(TheRobotCore);

        // State
        if(TheFreeModelRobotCore->predictState(PreviousTimeStamp, TheTimeStamp, pastStateRobot, predictedStateRobot))
        {
            std::cout<<"!!Error predicting state of the robot"<<std::endl;
            return 1;
        }

        // Jacobians
        if(TheFreeModelRobotCore->predictStateErrorStateJacobians(PreviousTimeStamp, TheTimeStamp, pastStateRobot, predictedStateRobot))
        {
            std::cout<<"!!Error predicting error state jacobians of the robot"<<std::endl;
            return 1;
        }


        // Add
        PredictedState.TheRobotStateCore=predictedStateRobot;

    }


    ///// Sensors
    for(std::list< std::shared_ptr<SensorCore> >::iterator itSens=TheListOfSensorCore.begin(); itSens!=TheListOfSensorCore.end(); ++itSens)
    {

        // IMU
        if((*itSens)->getSensorType() == SensorTypes::imu)
        {
            //std::cout<<"Imu sensor to predict"<<std::endl;

            std::shared_ptr<ImuSensorStateCore> predictedStateSensor;
            std::shared_ptr<ImuSensorStateCore> pastStateSensor;

            // Get past state of the sensor i
            for(std::list<std::shared_ptr<SensorStateCore> >::iterator itSensState=PreviousState.TheListSensorStateCore.begin();
                itSensState!=PreviousState.TheListSensorStateCore.end();
                ++itSensState)
            {
                //std::cout<<"matching previous sensor state"<<std::endl;

                if(!(*itSensState))
                {
                    std::cout<<"!!pointer not set properly"<<std::endl;
                    return -200;
                }

                if(!(*itSensState)->getTheSensorCore())
                {
                    std::cout<<"!!Error setting the core"<<std::endl;
                    return -100;
                }


                //std::cout<<"matching previous sensor state: going to check the type"<<std::endl;
                if((*itSensState)->getTheSensorCore()->getSensorType() == SensorTypes::imu)
                {
                    if((*itSensState)->getTheSensorCore()->getSensorId() == (*itSens)->getSensorId())
                    {
                        // Polymorphic
                        std::shared_ptr<ImuSensorStateCore> TheImuSensorStateCore=std::static_pointer_cast<ImuSensorStateCore>(*itSensState);

                        // Copy
                        pastStateSensor=TheImuSensorStateCore;


                        //std::cout<<"Found Past State!"<<std::endl;

                        break;
                    }
                }
            }

            // Check if not found
            if(!pastStateSensor)
            {
                std::cout<<"!!previous state for sensor not found!"<<std::endl;
                return -2;
            }

            // Polymorphic
            std::shared_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>(*itSens);

            // State
            if(TheImuSensorCore->predictState(PreviousTimeStamp, TheTimeStamp, pastStateSensor, predictedStateSensor))
            {
                std::cout<<"!!Error predicting state of sensor"<<std::endl;
                return 1;
            }

            // Jacobians
            if(TheImuSensorCore->predictStateErrorStateJacobians(PreviousTimeStamp, TheTimeStamp, pastStateSensor, predictedStateSensor))
            {
                std::cout<<"!!Error predicting error state jacobians of the sensor"<<std::endl;
                return 1;
            }

            // Add
            PredictedState.TheListSensorStateCore.push_back(predictedStateSensor);

        }


        // OTHER SENSORS
        // TODO

    }


    ///// Map
    // TODO



    /////// Covariances
    // TODO


    /////// Add element to the buffer
    TheMsfStorageCore->addElement(TheTimeStamp, PredictedState);


    return 0;
}



int MsfLocalizationCore::startThreads()
{
    // Prediction state thread
    predictThread=new std::thread(&MsfLocalizationCore::predictThreadFunction, this);

    // Buffer Manager thread
    bufferManagerThread=new std::thread(&MsfLocalizationCore::bufferManagerThreadFunction, this);


    return 0;
}


