
#include "msfLocalization.h"











MsfLocalizationCore::MsfLocalizationCore()
{
    // Flags
    stateEstimationEnabled=false;

    // Create Robot Core
    TheRobotCore=std::make_shared<RobotCore>();


    // Sensors
    //Id
    firstAvailableId=0;


    // Create Storage Core
    TheMsfStorageCore=std::make_shared<MsfStorageCore>();



    // LOG
    const char* env_p = std::getenv("FUSEON_STACK");

    logPath=std::string(env_p)+"/logs/"+"logMsfLocalizationCoreFile.txt";

    logFile.open(logPath);

    if(!logFile.is_open())
    {
        std::cout<<"unable to open log file"<<std::endl;
    }



    return;
}

MsfLocalizationCore::~MsfLocalizationCore()
{
    // Cleaning

    // TheListOfSensorCore
    TheListOfSensorCore.clear();


    // Log
    logFile.close();


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

int MsfLocalizationCore::setStateEstimationEnabled(bool predictEnabled)
{
    // Check
    if(this->stateEstimationEnabled==predictEnabled)
        return 0;

    // Clear the buffer except the last state estimation that will be used as init state
    TimeStamp InitTimeStamp;
    StateEstimationCore InitState;
    TheMsfStorageCore->getLastElementWithStateEstimate(InitTimeStamp, InitState);

    TheMsfStorageCore->purgeRingBuffer(-1);


    // Set the init time stamp and init state
    InitTimeStamp=getTimeStamp();
    TheMsfStorageCore->addElement(InitTimeStamp, InitState);


    // Change Flag in the core
    this->stateEstimationEnabled=predictEnabled;

    // Change flag in the sensors
    for(std::list< std::shared_ptr<SensorCore> >::iterator itListOfSensors=TheListOfSensorCore.begin();
        itListOfSensors!=TheListOfSensorCore.end();
        ++itListOfSensors)
    {
        (*itListOfSensors)->setSensorEnabled(predictEnabled);
    }


    return 0;
}

bool MsfLocalizationCore::isStateEstimationEnabled() const
{
    return this->stateEstimationEnabled;
}

TimeStamp MsfLocalizationCore::getTimeStamp()
{
    std::cout<<"MsfLocalizationCore::getTimeStamp()"<<std::endl;

    TimeStamp TheTimeStamp;
    return TheTimeStamp;
}

int MsfLocalizationCore::predictThreadFunction()
{
    std::cout<<"MsfLocalizationCore::predictThreadFunction()"<<std::endl;

    // TODO Finish

    return 0;
}



int MsfLocalizationCore::bufferManagerThreadFunction()
{

    // TODO Finish

    return 0;
}


int MsfLocalizationCore::predict(TimeStamp TheTimeStamp)
{
    //std::cout<<"MsfLocalizationCore::predict() TS: sec="<<TheTimeStamp.sec<<" s; nsec="<<TheTimeStamp.nsec<<" ns"<<std::endl;


    if(!isStateEstimationEnabled())
        return 0;


    // New element that is going to be added to the buffer
    StateEstimationCore PredictedState;


    // Get the last state from the buffer
    TimeStamp PreviousTimeStamp;
    StateEstimationCore PreviousState;
    TheMsfStorageCore->getLastElementWithStateEstimate(PreviousTimeStamp, PreviousState);


    // Check
    if(!PreviousState.hasState())
    {
        std::cout<<"!!error finding previous state"<<std::endl;
        return -1;
    }



    /////// State

    // Dimension of the state
    unsigned int dimensionOfState=0;
    unsigned int dimensionOfErrorState=0;



    ///// Robot

    // Dimension of state
    dimensionOfState+=TheRobotCore->getDimensionState();
    // Dimension of error state
    dimensionOfErrorState+=TheRobotCore->getDimensionErrorState();

    // Robot Type
    switch(TheRobotCore->getRobotType())
    {
        case RobotTypes::free_model:
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

            // End
            break;
        }

    }


    ///// Sensors
    for(std::list< std::shared_ptr<SensorCore> >::iterator itSens=TheListOfSensorCore.begin();
        itSens!=TheListOfSensorCore.end();
        ++itSens)
    {
        // Dimension of state
        dimensionOfState+=(*itSens)->getDimensionState();
        // Dimension of the error state
        dimensionOfErrorState+=(*itSens)->getDimensionErrorState();


        // Auxiliar
        std::shared_ptr<SensorStateCore> pastStateSensor;

        // Find
        if(findSensorStateCoreFromList(PreviousState.TheListSensorStateCore, (*itSens), pastStateSensor))
            return -2;


        // Switch according to the sensor type
        switch((*itSens)->getSensorType())
        {
            /// IMU
            case SensorTypes::imu:
            {
                //Auxiliar
                std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor;

                // Polymorphic
                std::shared_ptr<ImuSensorCore> TheImuSensorCore=std::dynamic_pointer_cast<ImuSensorCore>(*itSens);
                std::shared_ptr<ImuSensorStateCore> pastImuStateSensor=std::static_pointer_cast<ImuSensorStateCore>(pastStateSensor);

                // State
                if(TheImuSensorCore->predictState(PreviousTimeStamp, TheTimeStamp, pastImuStateSensor, predictedImuStateSensor))
                {
                    std::cout<<"!!Error predicting state of sensor"<<std::endl;
                    return 1;
                }

                // Jacobians
                if(TheImuSensorCore->predictStateErrorStateJacobians(PreviousTimeStamp, TheTimeStamp, pastImuStateSensor, predictedImuStateSensor))
                {
                    std::cout<<"!!Error predicting error state jacobians of the sensor"<<std::endl;
                    return 1;
                }

                // Add
                PredictedState.TheListSensorStateCore.push_back(predictedImuStateSensor);

                // End
                break;
            }

        }


        // OTHER SENSORS
        // TODO

    }


    ///// Map
    // TODO



    /////// Covariances
    // TODO

    // Auxiliar variables
    unsigned int previousDimensionErrorState_col;
    unsigned int previousDimensionErrorState_row;

    // Resize the covariance Matrix
    PredictedState.covarianceMatrix.resize(dimensionOfErrorState,dimensionOfErrorState);
    PredictedState.covarianceMatrix.setZero();


    /// Robot
    switch(TheRobotCore->getRobotType())
    {
        case RobotTypes::free_model:
        {
            // Aux variable
            //std::shared_ptr<FreeModelRobotStateCore> previousStateRobot=std::static_pointer_cast<FreeModelRobotStateCore>(PreviousState.TheRobotStateCore);
            std::shared_ptr<FreeModelRobotStateCore> predictedStateRobot=std::static_pointer_cast<FreeModelRobotStateCore>(PredictedState.TheRobotStateCore);

            //check
            if(!predictedStateRobot)
            {
                return -2;
            }

            // Covariances update
            // Linear part
            PredictedState.covarianceMatrix.block<9,9>(0,0)=predictedStateRobot->errorStateJacobian.linear*PreviousState.covarianceMatrix.block<9,9>(0,0)*predictedStateRobot->errorStateJacobian.linear.transpose();

            // Angular part
            PredictedState.covarianceMatrix.block<6,6>(9,9)=predictedStateRobot->errorStateJacobian.angular*PreviousState.covarianceMatrix.block<6,6>(9,9)*predictedStateRobot->errorStateJacobian.angular.transpose();

            // Linear - angular
            PredictedState.covarianceMatrix.block<9,6>(0,9)=predictedStateRobot->errorStateJacobian.linear*PreviousState.covarianceMatrix.block<9,6>(0,9)*predictedStateRobot->errorStateJacobian.angular.transpose();

            // Angular - linear
            PredictedState.covarianceMatrix.block<6,9>(9,0)=PredictedState.covarianceMatrix.block<9,6>(0,9).transpose();

            // End
            break;
        }
    }


    /// Robot-Sensors
    // TODO Today

    switch(TheRobotCore->getRobotType())
    {
        case RobotTypes::free_model:
        {
            // Aux variable
            //std::shared_ptr<FreeModelRobotStateCore> previousStateRobot=std::static_pointer_cast<FreeModelRobotStateCore>(PreviousState.TheRobotStateCore);
            std::shared_ptr<FreeModelRobotStateCore> predictedStateRobot=std::static_pointer_cast<FreeModelRobotStateCore>(PredictedState.TheRobotStateCore);

            //check
            if(!predictedStateRobot)
            {
                return -2;
            }


            // Iterate on the sensors
            for(std::list< std::shared_ptr<SensorCore> >::iterator it1Sens=TheListOfSensorCore.begin();
                it1Sens!=TheListOfSensorCore.end();
                ++it1Sens)
            {


                // Auxiliar
                std::shared_ptr<SensorStateCore> predictedStateSensor1;

                // Find
                if(findSensorStateCoreFromList(PredictedState.TheListSensorStateCore, (*it1Sens), predictedStateSensor1))
                    return -2;


                // Switch type of sensor it1Sens
                switch((*it1Sens)->getSensorType())
                {
                    /// Imu
                    case SensorTypes::imu:
                    {

                        // Polymorphic
                        std::shared_ptr<ImuSensorCore> TheImuSensor1Core=std::dynamic_pointer_cast<ImuSensorCore>(*it1Sens);
                        std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor1=std::static_pointer_cast<ImuSensorStateCore>(predictedStateSensor1);




                        // Covariances update


                        // R-Si
                        // Robot linear
                        // TODO
                        /*
                        // Position sensor wrt robot
                        if(TheImuSensor2Core->isEstimationPositionSensorWrtRobotEnabled())
                        {
                            // Update variances
                            PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.positionSensorWrtRobot.transpose();

                            // Update dimension for next
                            previousDimensionErrorState_col+=3;
                        }

                        // Attitude sensor wrt robot
                        if(TheImuSensor2Core->isEstimationAttitudeSensorWrtRobotEnabled())
                        {
                            // Update variances
                            PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.attitudeSensorWrtRobot.transpose();

                            // Update dimension for next
                            previousDimensionErrorState_col+=3;
                        }

                        // bias linear acceleration
                        if(TheImuSensor2Core->isEstimationBiasLinearAccelerationEnabled())
                        {
                            // Update variances
                            PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.biasesLinearAcceleration.transpose();

                            // Update dimension for next
                            previousDimensionErrorState_col+=3;
                        }

                        // bias angular velocity
                        if(TheImuSensor2Core->isEstimationBiasAngularVelocityEnabled())
                        {
                            // Update variances
                            PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.biasesAngularVelocity.transpose();

                            // Update dimension for next
                            previousDimensionErrorState_col+=3;
                        }
                        */

                        // Robot angular
                        // TODO




                        // Si-R
                        // Update the symmetric part
                        // TODO



                        // End
                        break;
                    }
                }

            }



            // End
            break;
        }
    }



    /// Sensors

    // Dimension
    previousDimensionErrorState_col=TheRobotCore->getDimensionErrorState();


    // Iterate
    for(std::list< std::shared_ptr<SensorCore> >::iterator it1Sens=TheListOfSensorCore.begin();
        it1Sens!=TheListOfSensorCore.end();
        ++it1Sens)
    {
        // Dimension
        previousDimensionErrorState_row=previousDimensionErrorState_col;

        // Iterate again
        for(std::list< std::shared_ptr<SensorCore> >::iterator it2Sens=it1Sens;
            it2Sens!=TheListOfSensorCore.end();
            ++it2Sens)
        {
            // it1Sens

            // Auxiliar
            std::shared_ptr<SensorStateCore> predictedStateSensor1;

            // Find
            if(findSensorStateCoreFromList(PredictedState.TheListSensorStateCore, (*it1Sens), predictedStateSensor1))
                return -2;


            // Switch type of sensor it1Sens
            switch((*it1Sens)->getSensorType())
            {
                /// Imu
                case SensorTypes::imu:
                {

                    // Polymorphic
                    std::shared_ptr<ImuSensorCore> TheImuSensor1Core=std::dynamic_pointer_cast<ImuSensorCore>(*it1Sens);
                    std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor1=std::static_pointer_cast<ImuSensorStateCore>(predictedStateSensor1);


                    // it2Sens

                    // Auxiliar
                    std::shared_ptr<SensorStateCore> predictedStateSensor2;

                    // Find
                    if(findSensorStateCoreFromList(PredictedState.TheListSensorStateCore, (*it2Sens), predictedStateSensor2))
                        return -2;


                    // Switch type of sensor it2Sens
                    switch((*it2Sens)->getSensorType())
                    {
                        /// Imu
                        case SensorTypes::imu:
                        {

                            // Polymorphic
                            std::shared_ptr<ImuSensorCore> TheImuSensor2Core=std::dynamic_pointer_cast<ImuSensorCore>(*it2Sens);
                            std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor2=std::static_pointer_cast<ImuSensorStateCore>(predictedStateSensor2);




                            // Covariances update

                            // Save the initDimension
                            unsigned int initDimension_row=previousDimensionErrorState_row;
                            unsigned int initDimension_col=previousDimensionErrorState_col;

                            // Position sensor wrt robot
                            if(TheImuSensor1Core->isEstimationPositionSensorWrtRobotEnabled())
                            {
                                // Reset col
                                previousDimensionErrorState_col=initDimension_col;

                                // Position sensor wrt robot
                                if(TheImuSensor2Core->isEstimationPositionSensorWrtRobotEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.positionSensorWrtRobot.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // Attitude sensor wrt robot
                                if(TheImuSensor2Core->isEstimationAttitudeSensorWrtRobotEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.attitudeSensorWrtRobot.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // bias linear acceleration
                                if(TheImuSensor2Core->isEstimationBiasLinearAccelerationEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.biasesLinearAcceleration.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // bias angular velocity
                                if(TheImuSensor2Core->isEstimationBiasAngularVelocityEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.biasesAngularVelocity.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // Update dimension for next
                                // Update row
                                previousDimensionErrorState_row+=3;
                            }

                            // Attitude sensor wrt robot
                            if(TheImuSensor1Core->isEstimationAttitudeSensorWrtRobotEnabled())
                            {
                                // Reset col
                                previousDimensionErrorState_col=initDimension_col;

                                // Position sensor wrt robot
                                if(TheImuSensor2Core->isEstimationPositionSensorWrtRobotEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.positionSensorWrtRobot.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // Attitude sensor wrt robot
                                if(TheImuSensor2Core->isEstimationAttitudeSensorWrtRobotEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.attitudeSensorWrtRobot.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // bias linear acceleration
                                if(TheImuSensor2Core->isEstimationBiasLinearAccelerationEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.biasesLinearAcceleration.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // bias angular velocity
                                if(TheImuSensor2Core->isEstimationBiasAngularVelocityEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.biasesAngularVelocity.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // Update dimension for next
                                // Update row
                                previousDimensionErrorState_row+=3;
                            }

                            // bias linear acceleration
                            if(TheImuSensor1Core->isEstimationBiasLinearAccelerationEnabled())
                            {
                                // Reset col
                                previousDimensionErrorState_col=initDimension_col;

                                // Position sensor wrt robot
                                if(TheImuSensor2Core->isEstimationPositionSensorWrtRobotEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.positionSensorWrtRobot.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // Attitude sensor wrt robot
                                if(TheImuSensor2Core->isEstimationAttitudeSensorWrtRobotEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.attitudeSensorWrtRobot.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // bias linear acceleration
                                if(TheImuSensor2Core->isEstimationBiasLinearAccelerationEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.biasesLinearAcceleration.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // bias angular velocity
                                if(TheImuSensor2Core->isEstimationBiasAngularVelocityEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.biasesAngularVelocity.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // Update dimension for next
                                // Update row
                                previousDimensionErrorState_row+=3;
                            }

                            // bias angular velocity
                            if(TheImuSensor1Core->isEstimationBiasAngularVelocityEnabled())
                            {
                                // Reset col
                                previousDimensionErrorState_col=initDimension_col;

                                // Position sensor wrt robot
                                if(TheImuSensor2Core->isEstimationPositionSensorWrtRobotEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.positionSensorWrtRobot.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // Attitude sensor wrt robot
                                if(TheImuSensor2Core->isEstimationAttitudeSensorWrtRobotEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.attitudeSensorWrtRobot.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // bias linear acceleration
                                if(TheImuSensor2Core->isEstimationBiasLinearAccelerationEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.biasesLinearAcceleration.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // bias angular velocity
                                if(TheImuSensor2Core->isEstimationBiasAngularVelocityEnabled())
                                {
                                    // Update variances
                                    PredictedState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)=predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity*PreviousState.covarianceMatrix.block<3,3>(previousDimensionErrorState_row,previousDimensionErrorState_col)*predictedImuStateSensor2->errorStateJacobian.biasesAngularVelocity.transpose();

                                    // Update dimension for next
                                    previousDimensionErrorState_col+=3;
                                }

                                // Update dimension for next
                                // Update row
                                previousDimensionErrorState_row+=3;
                            }


                            // Symmetric value of the covariance matrix
                            if(TheImuSensor1Core!=TheImuSensor1Core)
                            {
                                // Poner symmetric value
                                PredictedState.covarianceMatrix.block(previousDimensionErrorState_col,previousDimensionErrorState_row,previousDimensionErrorState_col-initDimension_row, previousDimensionErrorState_row-initDimension_row)=PredictedState.covarianceMatrix.block(initDimension_row,initDimension_row,previousDimensionErrorState_row-initDimension_row,previousDimensionErrorState_col-initDimension_row);
                            }


                            // End
                            break;
                        }
                    }


                    // End
                    break;
                }
            }



        // Update the dimension for the next sensor -> Not needed!
        //previousDimensionErrorState+=(*it1Sens)->getDimensionErrorState();


        }

    }


    /// Robot-Map
    // TODO

    /// Sensors-Map
    // TODO

    /// Map
    // TODO



    /// Display
    //logFile<<"Covariances Matrix of the predicted state="<<std::endl;
    //logFile<<PredictedState.covarianceMatrix<<std::endl;


//    //
//    TimeStamp newTimeStamp1=getTimeStamp();
//    TimeStamp duration1=newTimeStamp1-TheTimeStamp;
//    std::cout<<" before buffer duration: sec="<<duration1.sec<<" s; nsec="<<duration1.nsec<<" ns"<<std::endl;



    /////// Add element to the buffer
    TheMsfStorageCore->addElement(TheTimeStamp, PredictedState);


//    //
//    TimeStamp newTimeStamp2=getTimeStamp();
//    TimeStamp duration2=newTimeStamp2-newTimeStamp1;
//    std::cout<<" before buffer duration: sec="<<duration2.sec<<" s; nsec="<<duration2.nsec<<" ns"<<std::endl;


    return 0;
}



int MsfLocalizationCore::findSensorStateCoreFromList(std::list<std::shared_ptr<SensorStateCore> > TheListSensorStateCore, std::shared_ptr<SensorCore> TheSensorCore, std::shared_ptr<SensorStateCore>& TheSensorStateCore)
{

    // Match with the predicted state
    // Get predicted state of the sensor itSens
    for(std::list<std::shared_ptr<SensorStateCore> >::iterator itSensState=TheListSensorStateCore.begin();
        itSensState!=TheListSensorStateCore.end();
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
            std::cout<<"!!Error getting the core"<<std::endl;
            return -100;
        }


        //std::cout<<"matching previous sensor state: going to check the type"<<std::endl;
        if((*itSensState)->getTheSensorCore()->getSensorId() == TheSensorCore->getSensorId())
        {
            // Polymorphic
            TheSensorStateCore=(*itSensState);
            break;
        }
    }

    // Check if not found
    if(!TheSensorStateCore)
    {
        std::cout<<"!!predicted state for sensor not found!"<<std::endl;
        return -2;
    }




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


