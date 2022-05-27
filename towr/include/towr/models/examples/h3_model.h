#ifndef H3_MODEL_H
#define H3_MODEL_H

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of H3
 */
class H3KinematicModel : public KinematicModel {
public:
  H3KinematicModel () : KinematicModel(2)
  {
    const double z_nominal_b =  -0.95;
    const double y_nominal_b =  0.1825;

    nominal_stance_.at(L) << 0.15,  y_nominal_b, z_nominal_b;
    nominal_stance_.at(R) << 0.15, -y_nominal_b, z_nominal_b;
    
    max_dev_from_nominal_  << 0.25, 0.0, 0.15;
  }
};

/**
 * @brief The Dynamics of a H3
 */
class H3DynamicModel : public SingleRigidBodyDynamics {
public:
  H3DynamicModel()
  : SingleRigidBodyDynamics(16.133,
                    2.81354,2.4824,0.841546, 0.0092366,0.422698,0.0459433,
                    2) {}
};

} /* namespace towr */

#endif /* end of include guard: H3_MODEL_H */
