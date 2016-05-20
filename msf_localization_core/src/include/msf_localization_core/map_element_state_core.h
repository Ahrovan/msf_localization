
#ifndef _MAP_ELEMENT_STATE_CORE_H
#define _MAP_ELEMENT_STATE_CORE_H


#include <Eigen/Dense>
#include <Eigen/Sparse>


#include "msf_localization_core/state_core.h"
//#include "msf_localization_core/map_element_core.h"



class MapElementStateCore : public StateCore
{
public:
    MapElementStateCore();
    MapElementStateCore(const std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~MapElementStateCore();


protected:
    int init();





    //// Jacobians mapping: G

    /// Jacobian mapping error state: Gx
public:
    struct JacobianMappingErrorState
    {
        Eigen::MatrixXd jacobian_mapping_global_parameters_error_state_;
        Eigen::MatrixXd jacobian_mapping_robot_error_state_;
        // TODO Inputs
        Eigen::MatrixXd jacobian_mapping_sensor_error_state_;
        // TODO Map elements
    } jacobian_mapping_error_state_;


public:
    Eigen::MatrixXd getJacobianMappingGlobalParametersErrorState();
    Eigen::MatrixXd getJacobianMappingRobotErrorState();
    Eigen::MatrixXd getJacobianMappingSensorErrorState();



    /// Jacobian mapping error parameters: Gp
    // TODO



    /// Jacobian mapping noise: Gn

protected:
public:
    Eigen::SparseMatrix<double> jacobian_mapping_error_state_noise_;


public:
    Eigen::SparseMatrix<double> getJacobianMappingErrorStateNoise();



};



#endif
