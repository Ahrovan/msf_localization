
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

    // Auxiliar variables
//    unsigned int previousDimensionErrorState_col;
//    unsigned int previousDimensionErrorState_row;

    // Dimension
    MatrixPoint WorkingInitPoint;


    // Delta Time Stamp
    TimeStamp DeltaTime=TheTimeStamp-PreviousTimeStamp;


//    std::cout<<"Delta TS: sec="<<DeltaTime.sec<<" s; nsec="<<DeltaTime.nsec<<" ns"<<std::endl;
//    std::cout<<" ->dt="<<DeltaTime.get_double()<<std::endl;


    // Resize the covariance Matrix
    PredictedState.covarianceMatrix.resize(dimensionOfErrorState,dimensionOfErrorState);
    PredictedState.covarianceMatrix.setZero();


    /// Robot
    switch(TheRobotCore->getRobotType())
    {
        case RobotTypes::free_model:
        {
            // Covariance
            if(predictedFreeModelRobotCovariance(DeltaTime, std::static_pointer_cast<FreeModelRobotCore>(TheRobotCore), std::static_pointer_cast<FreeModelRobotStateCore>(PredictedState.TheRobotStateCore), &PreviousState.covarianceMatrix, &PredictedState.covarianceMatrix))
                return -2;

            // End
            break;
        }
    }


    /// Robot-Sensors

    // Dimensions
    MatrixPoint RobotSensorsInitPoint;
    MatrixPoint RobotSensorsEndPoint;

    // Init Point
    RobotSensorsInitPoint.col=TheRobotCore->getDimensionErrorState();
    RobotSensorsInitPoint.row=0;

    // Working Point
    WorkingInitPoint=RobotSensorsInitPoint;


    switch(TheRobotCore->getRobotType())
    {
        case RobotTypes::free_model:
        {

            // Dimension
            //previousDimensionErrorState_row=0;
            //previousDimensionErrorState_col=TheRobotCore->getDimensionErrorState();


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
                        // Covariances update
                        predictedFreeModelRobotImuCovariance(DeltaTime, std::dynamic_pointer_cast<FreeModelRobotCore>(TheRobotCore), std::dynamic_pointer_cast<ImuSensorCore>(*it1Sens),
                                                  std::static_pointer_cast<FreeModelRobotStateCore>(PredictedState.TheRobotStateCore), std::static_pointer_cast<ImuSensorStateCore>(predictedStateSensor1),
                                                  &PreviousState.covarianceMatrix, WorkingInitPoint, &PredictedState.covarianceMatrix, RobotSensorsEndPoint);

                        // End
                        break;
                    }
                }

                // Update Point
                WorkingInitPoint.col=RobotSensorsEndPoint.col;
            }

            // End
            break;
        }
    }



    /// Sensors

    // Dimension
    MatrixPoint SensorsInitPoint;
    MatrixPoint SensorsEndPoint;


    // Init Point
    SensorsInitPoint.col=TheRobotCore->getDimensionErrorState();
    SensorsInitPoint.row=SensorsInitPoint.col;

    // Working Point
    WorkingInitPoint=SensorsInitPoint;


    // Iterate
    for(std::list< std::shared_ptr<SensorCore> >::iterator it1Sens=TheListOfSensorCore.begin();
        it1Sens!=TheListOfSensorCore.end();
        ++it1Sens)
    {
        // it1Sens Auxiliar
        std::shared_ptr<SensorStateCore> predictedStateSensor1;

        // Find
        if(findSensorStateCoreFromList(PredictedState.TheListSensorStateCore, (*it1Sens), predictedStateSensor1))
            return -2;


        // Iterate again
        for(std::list< std::shared_ptr<SensorCore> >::iterator it2Sens=it1Sens;
            it2Sens!=TheListOfSensorCore.end();
            ++it2Sens)
        {
            // it2Sens Auxiliar
            std::shared_ptr<SensorStateCore> predictedStateSensor2;

            // Find
            if(findSensorStateCoreFromList(PredictedState.TheListSensorStateCore, (*it2Sens), predictedStateSensor2))
                return -2;


            // Switch type of sensor it1Sens
            switch((*it1Sens)->getSensorType())
            {
                /// Imu
                case SensorTypes::imu:
                {
                    // Switch type of sensor it2Sens
                    switch((*it2Sens)->getSensorType())
                    {
                        /// Imu
                        case SensorTypes::imu:
                        {
                            // Do the covariance update
                            predictedImuImuCovariance(DeltaTime, std::dynamic_pointer_cast<ImuSensorCore>(*it1Sens), std::dynamic_pointer_cast<ImuSensorCore>(*it2Sens),
                                                      std::static_pointer_cast<ImuSensorStateCore>(predictedStateSensor1), std::static_pointer_cast<ImuSensorStateCore>(predictedStateSensor2),
                                                      &PreviousState.covarianceMatrix, WorkingInitPoint, &PredictedState.covarianceMatrix, SensorsEndPoint);

                            // End
                            break;
                        }
                    }
                    // End
                    break;
                }
            }

            // Update Working Point
            WorkingInitPoint.col=SensorsEndPoint.col;

        }

        // Update the dimension for the next sensor
        WorkingInitPoint.row=SensorsEndPoint.row;
        WorkingInitPoint.col=WorkingInitPoint.row;

    }


    /// Robot-Map
    // TODO

    /// Sensors-Map
    // TODO

    /// Map
    // TODO



    /// Display
    logFile<<"Covariances Matrix of the predicted state="<<std::endl;
    logFile<<PredictedState.covarianceMatrix<<std::endl;



    /////// Add element to the buffer
    TheMsfStorageCore->addElement(TheTimeStamp, PredictedState);

    // End
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



int MsfLocalizationCore::predictedFreeModelRobotCovariance(TimeStamp DeltaTime, std::shared_ptr<FreeModelRobotCore> robotCore, std::shared_ptr<FreeModelRobotStateCore> predictedStateRobot, Eigen::MatrixXd* previousStateCovarianceMatrix, Eigen::MatrixXd* predictedStateCovarianceMatrix)
{
    //check
    if(!predictedStateRobot)
    {
        return -2;
    }

    // Covariances update
    // Linear part
    predictedStateCovarianceMatrix->block<9,9>(0,0)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,9>(0,0)*predictedStateRobot->errorStateJacobian.linear.transpose();

    // Angular part
    predictedStateCovarianceMatrix->block<6,6>(9,9)=predictedStateRobot->errorStateJacobian.angular*previousStateCovarianceMatrix->block<6,6>(9,9)*predictedStateRobot->errorStateJacobian.angular.transpose();

    // Linear - angular
    predictedStateCovarianceMatrix->block<9,6>(0,9)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,6>(0,9)*predictedStateRobot->errorStateJacobian.angular.transpose();

    // Angular - linear
    predictedStateCovarianceMatrix->block<6,9>(9,0)=predictedStateCovarianceMatrix->block<9,6>(0,9).transpose();



    // Adding noise

    // Noise in the linear acceleration * Dt
    predictedStateCovarianceMatrix->block<3,3>(6,6)+=robotCore->getNoiseLinearAcceleration()*DeltaTime.get_double();

    // Noise in the angular velocity * Dt
    predictedStateCovarianceMatrix->block<3,3>(12,12)+=robotCore->getNoiseAngularVelocity()*DeltaTime.get_double();


    // end
    return 0;

}


int MsfLocalizationCore::predictedImuImuCovariance(TimeStamp DeltaTime, std::shared_ptr<ImuSensorCore> TheImuSensor1Core, std::shared_ptr<ImuSensorCore> TheImuSensor2Core, std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor1, std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor2, Eigen::MatrixXd* previousStateCovarianceMatrix, MatrixPoint InitPoint, Eigen::MatrixXd* predictedStateCovarianceMatrix, MatrixPoint& EndPoint)
{
    // Check
    if(!predictedImuStateSensor1)
        return -2;
    if(!predictedImuStateSensor2)
        return -2;



    // Points
    MatrixPoint WorkingPoint=InitPoint;

    // Covariances update

    // Position sensor wrt robot
    if(TheImuSensor1Core->isEstimationPositionSensorWrtRobotEnabled())
    {
        // Reset col
        //previousDimensionErrorState_col=initDimension_col;
        WorkingPoint.col=InitPoint.col;

        // Position sensor wrt robot
        if(TheImuSensor2Core->isEstimationPositionSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.positionSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Attitude sensor wrt robot
        if(TheImuSensor2Core->isEstimationAttitudeSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.attitudeSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias linear acceleration
        if(TheImuSensor2Core->isEstimationBiasLinearAccelerationEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesLinearAcceleration.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias angular velocity
        if(TheImuSensor2Core->isEstimationBiasAngularVelocityEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesAngularVelocity.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Update dimension for next
        // Update row
        WorkingPoint.row+=3;
    }

    // Attitude sensor wrt robot
    if(TheImuSensor1Core->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        // Reset col
        WorkingPoint.col=InitPoint.col;

        // Position sensor wrt robot
        if(TheImuSensor2Core->isEstimationPositionSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.positionSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Attitude sensor wrt robot
        if(TheImuSensor2Core->isEstimationAttitudeSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.attitudeSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias linear acceleration
        if(TheImuSensor2Core->isEstimationBiasLinearAccelerationEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesLinearAcceleration.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias angular velocity
        if(TheImuSensor2Core->isEstimationBiasAngularVelocityEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesAngularVelocity.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Update dimension for next
        // Update row
        WorkingPoint.row+=3;
    }

    // bias linear acceleration
    if(TheImuSensor1Core->isEstimationBiasLinearAccelerationEnabled())
    {
        // Reset col
        WorkingPoint.col=InitPoint.col;

        // Position sensor wrt robot
        if(TheImuSensor2Core->isEstimationPositionSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.positionSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Attitude sensor wrt robot
        if(TheImuSensor2Core->isEstimationAttitudeSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.attitudeSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias linear acceleration
        if(TheImuSensor2Core->isEstimationBiasLinearAccelerationEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesLinearAcceleration.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias angular velocity
        if(TheImuSensor2Core->isEstimationBiasAngularVelocityEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesAngularVelocity.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Update dimension for next
        // Update row
        WorkingPoint.row+=3;
    }

    // bias angular velocity
    if(TheImuSensor1Core->isEstimationBiasAngularVelocityEnabled())
    {
        // Reset col
        WorkingPoint.col=InitPoint.col;

        // Position sensor wrt robot
        if(TheImuSensor2Core->isEstimationPositionSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.positionSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Attitude sensor wrt robot
        if(TheImuSensor2Core->isEstimationAttitudeSensorWrtRobotEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.attitudeSensorWrtRobot.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias linear acceleration
        if(TheImuSensor2Core->isEstimationBiasLinearAccelerationEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesLinearAcceleration.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // bias angular velocity
        if(TheImuSensor2Core->isEstimationBiasAngularVelocityEnabled())
        {
            // Update variances
            predictedStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)=predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity*previousStateCovarianceMatrix->block<3,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor2->errorStateJacobian.biasesAngularVelocity.transpose();

            // Update dimension for next
            WorkingPoint.col+=3;
        }

        // Update dimension for next
        // Update row
        WorkingPoint.row+=3;
    }

    // Update End Point
    EndPoint=WorkingPoint;


    // Symmetric value of the covariance matrix
    if(TheImuSensor1Core!=TheImuSensor2Core)
    {
        // Poner symmetric value
        predictedStateCovarianceMatrix->block(InitPoint.col,
                                              InitPoint.row,
                                              EndPoint.col-InitPoint.col,
                                              EndPoint.row-InitPoint.row) = predictedStateCovarianceMatrix->block(InitPoint.row,
                                                                                                                     InitPoint.col,
                                                                                                                     EndPoint.row-InitPoint.row,
                                                                                                                     EndPoint.col-InitPoint.col).transpose();

    }


    // Add noise
    if(TheImuSensor1Core==TheImuSensor2Core)
    {
        // Previous dimensions
        unsigned int previousErrorStateDimension=0;

        if(TheImuSensor1Core->isEstimationPositionSensorWrtRobotEnabled())
        {
            previousErrorStateDimension+=3;
        }

        if(TheImuSensor1Core->isEstimationAttitudeSensorWrtRobotEnabled())
        {
            previousErrorStateDimension+=3;
        }

        if(TheImuSensor1Core->isEstimationBiasLinearAccelerationEnabled())
        {
            predictedStateCovarianceMatrix->block<3,3>(InitPoint.col+previousErrorStateDimension,InitPoint.row+previousErrorStateDimension)+=TheImuSensor1Core->getNoiseBiasLinearAcceleration()*DeltaTime.get_double();
            previousErrorStateDimension+=3;
        }

        if(TheImuSensor1Core->isEstimationBiasAngularVelocityEnabled())
        {
            predictedStateCovarianceMatrix->block<3,3>(InitPoint.col+previousErrorStateDimension,InitPoint.row+previousErrorStateDimension)+=TheImuSensor1Core->getNoiseBiasAngularVelocity()*DeltaTime.get_double();
        }


    }




    return 0;
}



int MsfLocalizationCore::predictedFreeModelRobotImuCovariance(TimeStamp DeltaTime, std::shared_ptr<FreeModelRobotCore> robotCore, std::shared_ptr<ImuSensorCore> TheImuSensor1Core, std::shared_ptr<FreeModelRobotStateCore> predictedStateRobot, std::shared_ptr<ImuSensorStateCore> predictedImuStateSensor1, Eigen::MatrixXd* previousStateCovarianceMatrix, MatrixPoint InitPoint, Eigen::MatrixXd* predictedStateCovarianceMatrix, MatrixPoint& EndPoint)
{
    //check
    if(!predictedStateRobot)
    {
        return -2;
    }
    if(!predictedImuStateSensor1)
        return -2;



    // Dimension
    MatrixPoint WorkingPoint=InitPoint;

    // R-Si
    // Robot linear

    // Position sensor wrt robot
    if(TheImuSensor1Core->isEstimationPositionSensorWrtRobotEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // Attitude sensor wrt robot
    if(TheImuSensor1Core->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // bias linear acceleration
    if(TheImuSensor1Core->isEstimationBiasLinearAccelerationEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // bias angular velocity
    if(TheImuSensor1Core->isEstimationBiasAngularVelocityEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.linear*previousStateCovarianceMatrix->block<9,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // Dimension
    WorkingPoint.row+=9;


    // Robot angular

    // Reset col
    WorkingPoint.col=InitPoint.col;


    // Position sensor wrt robot
    if(TheImuSensor1Core->isEstimationPositionSensorWrtRobotEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<6,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.angular*previousStateCovarianceMatrix->block<6,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.positionSensorWrtRobot.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // Attitude sensor wrt robot
    if(TheImuSensor1Core->isEstimationAttitudeSensorWrtRobotEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<6,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.angular*previousStateCovarianceMatrix->block<6,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.attitudeSensorWrtRobot.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // bias linear acceleration
    if(TheImuSensor1Core->isEstimationBiasLinearAccelerationEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<6,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.angular*previousStateCovarianceMatrix->block<6,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.biasesLinearAcceleration.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // bias angular velocity
    if(TheImuSensor1Core->isEstimationBiasAngularVelocityEnabled())
    {
        // Update variances
        predictedStateCovarianceMatrix->block<6,3>(WorkingPoint.row, WorkingPoint.col)=predictedStateRobot->errorStateJacobian.angular*previousStateCovarianceMatrix->block<6,3>(WorkingPoint.row, WorkingPoint.col)*predictedImuStateSensor1->errorStateJacobian.biasesAngularVelocity.transpose();

        // Update dimension for next
        WorkingPoint.col+=3;
    }

    // Dimension
    WorkingPoint.row+=6;


    // Final dimension
    EndPoint=WorkingPoint;



    // Si-R
    // Update the symmetric part
    predictedStateCovarianceMatrix->block(InitPoint.col,
                                          InitPoint.row,
                                          EndPoint.col-InitPoint.col,
                                          EndPoint.row-InitPoint.row) = predictedStateCovarianceMatrix->block(InitPoint.row,
                                                                                                             InitPoint.col,
                                                                                                             EndPoint.row-InitPoint.row,
                                                                                                             EndPoint.col-InitPoint.col).transpose();


    // End
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


