#include <iostream>
#include <rbdl/rbdl.h>
#include "Dynamics.h"
#include "test_models.h"
#include <string>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Dynamics::Dynamics(std::string robot_model)
{
    if (robot_model == "kuka_lbr")
    {
        Four_bar_linkage four_bar_obj_;
        KUKA_LBR kuka_obj_;
        Raven_II Raven_2a_;
        MTM_closed mtm_;
        model = kuka_obj_.model;
    }
    else if (robot_model == "four_bar_linkage")
    {
        Four_bar_linkage four_bar_obj_;
        KUKA_LBR kuka_obj_;
        Raven_II Raven_2_;
        MTM_closed mtm_;
        model = four_bar_obj_.model;
    }
    else if (robot_model == "mtm")
    {
        Four_bar_linkage four_bar_obj_;
        KUKA_LBR kuka_obj_;
        Raven_II Raven_2_;
        MTM_closed mtm_;
        model = mtm_.model;
    }
    else if (robot_model == "Raven_II")
    {
        Four_bar_linkage four_bar_obj_;
        KUKA_LBR kuka_obj_;
        Raven_II Raven_2_;
        MTM_closed mtm_;
        model = Raven_2_.model;
    }
    else
    {
        std::cerr << "Invalid entry!\n";
        std::cerr << "To create the KUKA model enter: kuka_lbr\n";
        std::cerr << "To create the four-bar linkage model enter: four_bar_linkage\n";
        std::cerr << "To create the MTM model enter: mtm\n";
        std::cerr << "To create the Raven_II model enter: Raven_II\n";
    }
};

unsigned int Dynamics::get_num_joints()
{
    return model.q_size;
};

MatrixNd Dynamics::calc_EndEffectorJacobian(const VectorNd &Q, unsigned int body_id, const Vector3d &body_point_position)
{
    MatrixNd J = MatrixNd::Zero(3, model.qdot_size);
    CalcPointJacobian(model, Q, body_id, body_point_position, J);
    return J;
};

Vector3d Dynamics::calc_EndEffectorVel(const VectorNd &Q, const VectorNd &QDot, unsigned int body_id, const Vector3d &body_point_position)
{
    return CalcPointVelocity(model, Q, QDot, body_id, body_point_position);
};

VectorNd Dynamics::calc_InverseKinematics(const VectorNd &Qinit, const std::vector<unsigned int> body_id, const std::vector<Vector3d> &body_point, const std::vector<Vector3d> &target_pos, VectorNd &Qres)
{
    bool success = InverseKinematics(model, Qinit, body_id, body_point, target_pos, Qres);
    if (success == 0)
    {
        std::cerr << "The operation was not successful\n";
        exit(0);
    }
    else if (success == 1)
    {
        std::cout << "The operation was successful\n";
    }
    return Qres;
};

Vector3d Dynamics::calc_ForwardKinematics(const Math::VectorNd &Q, unsigned int body_id, const Math::Vector3d &body_point_position)
{
    return CalcBodyToBaseCoordinates(model, Q, body_id, body_point_position);
};

MatrixNd Dynamics::calc_M(const VectorNd &Q)
{
    Math::MatrixNd M = Math::MatrixNd::Zero(model.qdot_size, model.qdot_size);
    CompositeRigidBodyAlgorithm(model, Q, M);
    return M;
};

VectorNd Dynamics::calc_G(const VectorNd &Q) // For serial manipulators, for parallel systems use InverseDynamicsConstrained method
{
    VectorNd QDot = VectorNd::Zero(model.qdot_size);
    VectorNd QDDot = VectorNd::Zero(model.qdot_size);
    VectorNd Tau = VectorNd::Zero(model.q_size);

    InverseDynamics(model, Q, QDot, QDDot, Tau);
    return Tau;
};

VectorNd Dynamics::calc_InverseDynamics(const VectorNd &Q, const VectorNd &QDot, const VectorNd &QDDot, VectorNd &Tau) // For serial manipulators, for parallel systems use InverseDynamicsConstrained method
{
    InverseDynamics(model, Q, QDot, QDDot, Tau);
    return Tau;
};

VectorNd Dynamics::calc_ForwardDynamics(const VectorNd &Q, const VectorNd &QDot, const VectorNd &Tau, VectorNd &QDDot)
{

    ForwardDynamics(model, Q, QDot, Tau, QDDot);
    return QDDot;
};

VectorNd Dynamics::calc_InverseDynamicsConstrained(const VectorNd &Q, const VectorNd &QDot, const VectorNd &QDDotDesired, ConstraintSet &CS, VectorNd &QDDotOutput, VectorNd &TauOutput)
{
    bool is_fully_actuated = isConstrainedSystemFullyActuated(model, Q, QDot, CS); //returns success if the model is fully actuated.;
    if (is_fully_actuated == 1)
    {
        std::vector<bool> actuatedDof(model.q_size, true);
        actuatedDof.at(3) = false; // setting the virual joint(loop constraint) as the passive joint
        CS.SetActuationMap(model, actuatedDof);
        InverseDynamicsConstraints(model, Q, QDot, QDDotDesired, CS, QDDotOutput, TauOutput);
        return TauOutput;
    }

    else if (is_fully_actuated != 1)
    {
        std::cerr << "The model is not fully actuated! Try using other InverseDynamicsRelaxed method.\n";
        exit(0);
    }
};

VectorNd Dynamics::calc_ForwardDynamicsConstrained(const VectorNd &Q, const VectorNd &QDot, const VectorNd &Tau, ConstraintSet &CS, VectorNd &QDDotOutput)
{
    ForwardDynamicsConstraintsDirect(model, Q, QDot, Tau, CS, QDDotOutput);
    return QDDotOutput;
};

VectorNd Dynamics::calc_MInvTimesTau(const VectorNd &Q, const VectorNd &Tau, VectorNd &QDDot)
{
    CalcMInvTimesTau(model, Q, Tau, QDDot);
    return QDDot;
};
