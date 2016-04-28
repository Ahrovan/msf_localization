
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
    MapElementStateCore(std::weak_ptr<MsfElementCore> msf_element_core_ptr);
    ~MapElementStateCore();


protected:
    int init();






public:
    virtual Eigen::MatrixXd getJacobianMappingRobotErrorState()=0;
    virtual Eigen::MatrixXd getJacobianMappingGlobalParametersErrorState()=0;
    virtual Eigen::MatrixXd getJacobianMappingSensorErrorState()=0;

    virtual Eigen::MatrixXd getJacobianMappingErrorStateNoise()=0;



};



#endif
