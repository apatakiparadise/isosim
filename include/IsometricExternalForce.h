/**************************************************************
 * ***********DEPRECATED - Use isosim.cpp*******************
 * ISOSIM Isometric External Force header
 * Attaches external force to the ID model
 * 
 *
 *
 * Author: Joshua Rolls
 * Date: 17-September-2021
**************************************************************/


#include <OpenSim/OpenSim.h>

namespace isosim {

class IsometricExternalForce : OpenSim::ExternalForce {
    OpenSim_DECLARE_CONCRETE_OBJECT(IsometricExternalForce, OpenSim::ExternalForce);

    public:
        IsometricExternalForce(void);

    private:
        OpenSim::Storage _storage;



};




}; //namespace isosim
