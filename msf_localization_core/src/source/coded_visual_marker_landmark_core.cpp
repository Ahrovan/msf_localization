
#include "msf_localization_core/coded_visual_marker_landmark_core.h"

#include "msf_localization_core/map_element_state_core.h"
#include "msf_localization_core/coded_visual_marker_landmark_state_core.h"


CodedVisualMarkerLandmarkCore::CodedVisualMarkerLandmarkCore():
    MapElementCore()
{
    init();

    return;
}

CodedVisualMarkerLandmarkCore::CodedVisualMarkerLandmarkCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore) :
    MapElementCore(TheMsfStorageCore)
{
    init();

    return;
}

CodedVisualMarkerLandmarkCore::~CodedVisualMarkerLandmarkCore()
{
    return;
}

int CodedVisualMarkerLandmarkCore::init()
{
    // Dimensions
    dimension_state_=0;
    dimension_error_state_=0;

    dimension_parameters_+=3+4;
    dimension_error_parameters_+=3+3;

    // Flags
    flag_estimation_position_visual_marker_wrt_world=false;
    flag_estimation_attitude_visual_marker_wrt_world=false;

    // Noises
    covariancePositionVisualMarkerWrtWorld.setZero();
    covarianceAttitudeVisualMarkerWrtWorld.setZero();

    // Id
    id_=-1;

    // Map Type
    setMapElementType(MapElementTypes::coded_visual_marker);


    return 0;
}

int CodedVisualMarkerLandmarkCore::getId() const
{
    return this->id_;
}

int CodedVisualMarkerLandmarkCore::setId(int id)
{
    this->id_=id;
    return 0;
}

int CodedVisualMarkerLandmarkCore::readConfig(pugi::xml_node map_element, std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& MapElementInitStateCore)
{
    // Map Element Core Pointer
    //std::shared_ptr<MapElementCore> TheMapElementCore(this);
    //std::shared_ptr<MapElementCore> TheMapElementCore=std::dynamic_pointer_cast<MapElementCore>(this->getMsfElementCoreSharedPtr());

    // Set pointer to the SensorCore
    //TheMapElementCore->setMsfElementCore(TheMapElementCore);

    // Create a class for the SensorStateCore
    if(!MapElementInitStateCore)
        MapElementInitStateCore=std::make_shared<CodedVisualMarkerLandmarkStateCore>(this->getMsfElementCoreSharedPtr());

    // Set pointer to the SensorCore
    //MapElementInitStateCore->setTheMapElementCore(TheMapElementCore);


    // Set the access to the Storage core
    //TheRosSensorImuInterface->setTheMsfStorageCore(std::make_shared<MsfStorageCore>(this->TheStateEstimationCore));
    //TheMapElementCore->setMsfStorageCore(TheMsfStorageCore);


    // Visual landmark
    pugi::xml_node visual_landmark=map_element.child("visual_landmark");


    // Auxiliar reading value
    std::string readingValue;


    /// id
    std::string idString=visual_landmark.child_value("id");
    this->setId(std::stoi(idString));


    /// Name
    this->setMapElementName("visual_marker_"+idString);


    //// Configs

    /// Pose of the map element wrt worl
    pugi::xml_node pose_in_world=visual_landmark.child("pose_in_world");

    // Position of the sensor wrt robot
    readingValue=pose_in_world.child("position").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationPositionVisualMarkerWrtWorld();

    // Attitude of the sensor wrt robot
    readingValue=pose_in_world.child("attitude").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationAttitudeVisualMarkerWrtWorld();



    //// Init State

    /// Pose of the visual marker wrt world

    // Position of the visual marker wrt world
    readingValue=pose_in_world.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        MapElementInitStateCore->setPosition(init_estimation);
    }

    // Attitude of the visual marker wrt world
    readingValue=pose_in_world.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2]>>init_estimation[3];
        MapElementInitStateCore->setAttitude(init_estimation);
    }


    /// Parameters

    // None


    //// Init Variances


    /// Pose of the visual marker wrt world

    // Position of the visual marker wrt world
    readingValue=pose_in_world.child("position").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setCovariancePositionVisualMarkerWrtWorld(variance.asDiagonal());
    }


    // Attitude of the visual marker wrt world
    readingValue=pose_in_world.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setCovarianceAttitudeVisualMarkerWrtWorld(variance.asDiagonal());
    }



    /// Other Parameters

    // None


    // Noises in the estimation (if enabled)

    // None


    // Prepare covariance matrix
    this->prepareCovarianceInitErrorState();


    /// Finish


    // End
    return 0;
}

bool CodedVisualMarkerLandmarkCore::isEstimationPositionVisualMarkerWrtWorldEnabled()
{
    return flag_estimation_position_visual_marker_wrt_world;
}

int CodedVisualMarkerLandmarkCore::enableEstimationPositionVisualMarkerWrtWorld()
{
    if(!flag_estimation_position_visual_marker_wrt_world)
    {
        flag_estimation_position_visual_marker_wrt_world=true;
        dimension_state_+=3;
        dimension_error_state_+=3;
        dimension_parameters_-=3;
        dimension_error_parameters_-=3;
    }
    return 0;
}

int CodedVisualMarkerLandmarkCore::enableParameterPositionVisualMarkerWrtWorld()
{
    if(flag_estimation_position_visual_marker_wrt_world)
    {
        flag_estimation_position_visual_marker_wrt_world=false;
        dimension_state_-=3;
        dimension_error_state_-=3;
        dimension_parameters_+=3;
        dimension_error_parameters_+=3;
    }
    return 0;
}

Eigen::Matrix3d CodedVisualMarkerLandmarkCore::getCovariancePositionVisualMarkerWrtWorld() const
{
    return this->covariancePositionVisualMarkerWrtWorld;
}

int CodedVisualMarkerLandmarkCore::setCovariancePositionVisualMarkerWrtWorld(Eigen::Matrix3d covariancePositionVisualMarkerWrtWorld)
{
    this->covariancePositionVisualMarkerWrtWorld=covariancePositionVisualMarkerWrtWorld;
    return 0;
}


bool CodedVisualMarkerLandmarkCore::isEstimationAttitudeVisualMarkerWrtWorldEnabled()
{
    return this->flag_estimation_attitude_visual_marker_wrt_world;
}

int CodedVisualMarkerLandmarkCore::enableEstimationAttitudeVisualMarkerWrtWorld()
{
    if(!flag_estimation_attitude_visual_marker_wrt_world)
    {
        flag_estimation_attitude_visual_marker_wrt_world=true;
        dimension_state_+=4;
        dimension_error_state_+=3;
        dimension_parameters_-=4;
        dimension_error_parameters_-=3;
    }

    return 0;
}

int CodedVisualMarkerLandmarkCore::enableParameterAttitudeVisualMarkerWrtWorld()
{
    if(flag_estimation_attitude_visual_marker_wrt_world)
    {
        flag_estimation_attitude_visual_marker_wrt_world=false;
        dimension_state_-=4;
        dimension_error_state_-=3;
        dimension_parameters_+=4;
        dimension_error_parameters_+=3;
    }
    return 0;
}


Eigen::Matrix3d CodedVisualMarkerLandmarkCore::getCovarianceAttitudeVisualMarkerWrtWorld() const
{
    return this->covarianceAttitudeVisualMarkerWrtWorld;
}

int CodedVisualMarkerLandmarkCore::setCovarianceAttitudeVisualMarkerWrtWorld(Eigen::Matrix3d covarianceAttitudeVisualMarkerWrtWorld)
{
    this->covarianceAttitudeVisualMarkerWrtWorld=covarianceAttitudeVisualMarkerWrtWorld;
    return 0;
}




int CodedVisualMarkerLandmarkCore::prepareCovarianceInitErrorStateSpecific()
{
    int dimension_i=0;

    if(flag_estimation_position_visual_marker_wrt_world)
    {
        this->covariance_init_error_state_.block<3,3>(dimension_i, dimension_i)=covariancePositionVisualMarkerWrtWorld;
        dimension_i+=3;
    }

    if(flag_estimation_attitude_visual_marker_wrt_world)
    {
        this->covariance_init_error_state_.block<3,3>(dimension_i, dimension_i)=covarianceAttitudeVisualMarkerWrtWorld;
        dimension_i+=3;
    }

    return 0;
}

/*
Eigen::SparseMatrix<double> CodedVisualMarkerLandmarkCore::getCovarianceMeasurement()
{
    Eigen::SparseMatrix<double> covariance;


    return covariance;
}
*/

Eigen::SparseMatrix<double> CodedVisualMarkerLandmarkCore::getCovarianceParameters()
{
    Eigen::SparseMatrix<double> covariance;

    covariance.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());
    covariance.reserve(this->getDimensionErrorParameters());

    std::vector<Eigen::Triplet<double> > tripletCovarianceParameters;


    unsigned int dimension=0;
    if(!this->isEstimationPositionVisualMarkerWrtWorldEnabled())
    {
        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,covariancePositionVisualMarkerWrtWorld(i,i)));

        dimension+=3;
    }
    if(!this->isEstimationAttitudeVisualMarkerWrtWorldEnabled())
    {
        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,covarianceAttitudeVisualMarkerWrtWorld(i,i)));

        dimension+=3;
    }


    covariance.setFromTriplets(tripletCovarianceParameters.begin(), tripletCovarianceParameters.end());



    return covariance;
}


Eigen::SparseMatrix<double> CodedVisualMarkerLandmarkCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
{
    Eigen::SparseMatrix<double> covariance;

    covariance.resize(0,0);

    // Do nothing

    return covariance;
}

int CodedVisualMarkerLandmarkCore::predictState(//Time
                 const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                 // Previous State
                 const std::shared_ptr<StateEstimationCore> pastState,
                 // Inputs
                 const std::shared_ptr<InputCommandComponent> inputCommand,
                 // Predicted State
                 std::shared_ptr<StateCore>& predictedState)
{

    // Checks

    // Past State
    if(!pastState)
        return -1;

    // TODO

    // Search for the past map State Core
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> past_map_state;

    for(std::list< std::shared_ptr<StateCore> >::iterator it_map_state=pastState->TheListMapElementStateCore.begin();
        it_map_state!=pastState->TheListMapElementStateCore.end();
        ++it_map_state)
    {
        if((*it_map_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            past_map_state=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(*it_map_state);
            break;
        }
    }
    if(!past_map_state)
    {
        std::cout<<"CodedVisualMarkerLandmarkCore::predictState() unable to find past_map_state"<<std::endl;
        return -10;
    }


    // Predicted State
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> predicted_map_state;
    if(!predictedState)
        predicted_map_state=std::make_shared<CodedVisualMarkerLandmarkStateCore>(past_map_state->getMsfElementCoreWeakPtr());
    else
        predicted_map_state=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(predictedState);





    // Predict State
    int error_predict_state=predictStateSpecific(previousTimeStamp, currentTimeStamp,
                                         past_map_state,
                                         predicted_map_state);

    // Check error
    if(error_predict_state)
    {
        std::cout<<"CodedVisualMarkerLandmarkCore::predictState() error_predict_state"<<std::endl;
        return error_predict_state;
    }


    // Set predicted state
    predictedState=predicted_map_state;



    // End
    return 0;
}

int CodedVisualMarkerLandmarkCore::predictStateSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                                        const std::shared_ptr<CodedVisualMarkerLandmarkStateCore> pastState,
                                                        std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& predictedState)
{
    //std::cout<<"CodedVisualMarkerLandmarkCore::predictState()"<<std::endl;

    // Poly
    //std::shared_ptr<CodedVisualMarkerLandmarkStateCore> pastState=std::static_pointer_cast<CodedVisualMarkerLandmarkStateCore>(pastStateI);
    //std::shared_ptr<CodedVisualMarkerLandmarkStateCore> predictedState=std::static_pointer_cast<CodedVisualMarkerLandmarkStateCore>(predictedStateI);



    // Checks in the past state
    if(!pastState->isCorrect())
    {
        return -5;
        std::cout<<"FreeModelRobotCore::predictState() error !pastState->getTheRobotCore()"<<std::endl;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=std::make_shared<CodedVisualMarkerLandmarkStateCore>(pastState->getMsfElementCoreWeakPtr());
    }

//    // Set The robot core if it doesn't exist
//    if(predictedState->getTheMapElementCoreWeak().expired())
//    {
//        predictedState->setTheMapElementCore(pastState->getTheMapElementCoreWeak());
//    }


    // Equations


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.get_double();


    /// Position
    if(flag_estimation_position_visual_marker_wrt_world)
        predictedState->position_=pastState->position_;
    else
        predictedState->position_=pastState->position_;


    /// Attitude
    if(flag_estimation_attitude_visual_marker_wrt_world)
//        if(pastState->attitude_[0]<0)
//        {
//            predictedState->attitude_=-pastState->attitude_;
//            std::cout<<"CodedVisualMarkerLandmarkCore::predictState() quaternion!!"<<std::endl;
//        }
//        else
            predictedState->attitude_=pastState->attitude_;
    else
//        if(pastState->attitude_[0]<0)
//        {
//            predictedState->attitude_=-pastState->attitude_;
//            std::cout<<"CodedVisualMarkerLandmarkCore::predictState() quaternion!!"<<std::endl;
//        }
//        else
            predictedState->attitude_=pastState->attitude_;


    // Finish
    //predictedStateI=predictedState;

    return 0;
}

// Jacobian
int CodedVisualMarkerLandmarkCore::predictErrorStateJacobian(//Time
                             const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                             // Previous State
                             const std::shared_ptr<StateEstimationCore> past_state,
                            // Inputs
                            const std::shared_ptr<InputCommandComponent> input_command,
                             // Predicted State
                             std::shared_ptr<StateCore> &predicted_state)
{
    // Checks

    // Past State
    if(!past_state)
        return -1;

    // Predicted State
    if(!predicted_state)
        return -1;

    // TODO


    // Search for the past map State Core
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> past_map_state;

    for(std::list< std::shared_ptr<StateCore> >::iterator it_map_state=past_state->TheListMapElementStateCore.begin();
        it_map_state!=past_state->TheListMapElementStateCore.end();
        ++it_map_state)
    {
        if((*it_map_state)->getMsfElementCoreSharedPtr() == this->getMsfElementCoreSharedPtr())
        {
            past_map_state=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(*it_map_state);
            break;
        }
    }
    if(!past_map_state)
        return -10;


    //// Init Jacobians
    int error_init_jacobians=predictErrorStateJacobianInit(// Past State
                                                           past_state,
                                                           // Input commands
                                                           input_command,
                                                           // Predicted State
                                                           predicted_state);

    if(error_init_jacobians)
        return error_init_jacobians;


    /// predicted map  State
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> predicted_map_state=std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>(predicted_state);



    /// Get iterators to fill jacobians

    // Fx & Fp
    // Map Element
    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_state_wrt_map_error_state;
    it_jacobian_error_state_wrt_map_error_state=predicted_map_state->jacobian_error_state_.map_elements.begin();

    std::vector<Eigen::SparseMatrix<double> >::iterator it_jacobian_error_state_wrt_map_error_parameters;
    it_jacobian_error_state_wrt_map_error_parameters=predicted_map_state->jacobian_error_parameters_.map_elements.begin();

    for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementStateCore=past_state->TheListMapElementStateCore.begin();
        itMapElementStateCore!=past_state->TheListMapElementStateCore.end();
        ++itMapElementStateCore, ++it_jacobian_error_state_wrt_map_error_state, ++it_jacobian_error_state_wrt_map_error_parameters
        )
    {
        if( std::dynamic_pointer_cast<CodedVisualMarkerLandmarkStateCore>((*itMapElementStateCore)) == past_map_state )
            break;
    }


    // Fu
    // Nothing


    // Fn
    // Nothing



    /// Predict State Jacobians
    int error_predict_state_jacobians=predictErrorStateJacobiansSpecific(previousTimeStamp, currentTimeStamp,
                                                                         past_map_state,
                                                                         predicted_map_state,
                                                                         // Jacobians Error State: Fx, Fp
                                                                         (*it_jacobian_error_state_wrt_map_error_state),
                                                                         (*it_jacobian_error_state_wrt_map_error_parameters)
                                                                         // Jacobian Error Noise
                                                                         // TODO
                                                                         );

    // Check error
    if(error_predict_state_jacobians)
        return error_predict_state_jacobians;


    /// Set predicted state
    predicted_state=predicted_map_state;


    // End
    return 0;
}

int CodedVisualMarkerLandmarkCore::predictErrorStateJacobiansSpecific(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp,
                                                                      const std::shared_ptr<CodedVisualMarkerLandmarkStateCore> pastState,
                                                                      std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& predictedState,
                                                                      // Jacobians Error State: Fx, Fp
                                                                      // Map
                                                                      Eigen::SparseMatrix<double>& jacobian_error_state_wrt_map_error_state,
                                                                      Eigen::SparseMatrix<double>& jacobian_error_state_wrt_map_error_parameters
                                                                      // Jacobians Noise: Hn
                                                                      // TODO
                                                                      )
{
    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        return -1;
    }


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.get_double();



    ///// Jacobian Error State - Error State: Fx & Jacobian Error State - Error Parameters: Fp

    /// World
    {
        // Nothing to do
    }

    /// Robot
    {
        // Nothing to do
    }

    /// Inputs
    {
        // Nothing to do
    }

    /// Sensors
    {
        // Nothing to do
    }

    /// Map Elements
    {
        // Resize and init
        jacobian_error_state_wrt_map_error_state.resize(dimension_error_state_, dimension_error_state_);
        jacobian_error_state_wrt_map_error_parameters.resize(dimension_error_state_, dimension_error_parameters_);

        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_state;
        std::vector<Eigen::Triplet<double>> triplet_list_jacobian_error_state_wrt_error_parameters;


        // Fill
        int dimension_error_state_i=0;
        int dimension_error_parameters_i=0;



        // posi / posi
        if(isEstimationPositionVisualMarkerWrtWorldEnabled())
        {
            //Eigen::MatrixXd::Identity(3,3);
            for(int i=0; i<3; i++)
                triplet_list_jacobian_error_state_wrt_error_state.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_i+i,1));
            dimension_error_state_i+=3;
        }


        // att / att
        if(isEstimationAttitudeVisualMarkerWrtWorldEnabled())
        {
            //Eigen::MatrixXd::Identity(3,3);
            for(int i=0; i<3; i++)
                triplet_list_jacobian_error_state_wrt_error_state.push_back(Eigen::Triplet<double>(dimension_error_state_i+i,dimension_error_state_i+i,1));
            dimension_error_state_i+=3;
        }


        // Set From Triplets
        jacobian_error_state_wrt_map_error_state.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_state.begin(), triplet_list_jacobian_error_state_wrt_error_state.end());
        jacobian_error_state_wrt_map_error_parameters.setFromTriplets(triplet_list_jacobian_error_state_wrt_error_parameters.begin(), triplet_list_jacobian_error_state_wrt_error_parameters.end());

    }




    //// Jacobian Error State - Error Input

    {
        // Nothing to do
    }


    //// Jacobian Error State - Noise Estimation: Fn

    {
        // Fill
        // Nothing to do
    }



    // End
    return 0;
}

int CodedVisualMarkerLandmarkCore::resetErrorStateJacobian(// Time
                                                const TimeStamp& current_time_stamp,
                                                // Increment Error State
                                                const Eigen::VectorXd& increment_error_state,
                                                // Current State
                                                std::shared_ptr<StateCore>& current_state
                                                )
{
    // Checks
    if(!current_state)
        return -1;


    // Resize Jacobian
    current_state->jacobian_error_state_reset_.resize(this->dimension_error_state_, this->dimension_error_state_);

    // Fill
    std::vector< Eigen::Triplet<double> > triplets_jacobian_error_reset;

    int dimension_error_state_i=0;

    // Position Map Element wrt World
    if(this->isEstimationPositionVisualMarkerWrtWorldEnabled())
    {
        for(int i=0; i<3; i++)
            triplets_jacobian_error_reset.push_back(Eigen::Triplet<double>(dimension_error_state_i+i, dimension_error_state_i+i, 1.0));

        dimension_error_state_i+=3;
    }

    // Attitude Map Element wrt World
    if(this->isEstimationAttitudeVisualMarkerWrtWorldEnabled())
    {
        // Error Reset Matrixes
        Eigen::Matrix3d G_update_theta_robot=Eigen::Matrix3d::Identity(3,3);


        // Ojo, signo cambiado por la definicion de incrementError!
        G_update_theta_robot-=Quaternion::skewSymMat(0.5*increment_error_state.block<3,1>(dimension_error_state_i,0));

        // Triplets
        BlockMatrix::insertVectorEigenTripletFromEigenDense(triplets_jacobian_error_reset, G_update_theta_robot, dimension_error_state_i, dimension_error_state_i);

        dimension_error_state_i+=3;
    }


    current_state->jacobian_error_state_reset_.setFromTriplets(triplets_jacobian_error_reset.begin(), triplets_jacobian_error_reset.end());

    // End
    return 0;
}
