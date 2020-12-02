#pragma once
#include <rbdl/rbdl.h>
#include <string>
#include "test_models.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
class Dynamics
{
public:
    //constructor
    Dynamics(std::string robot_model);
    Four_bar_linkage four_bar_obj_;
    KUKA_LBR kuka_obj_;
    Raven_II raven_2;
    MTM_closed MTM;
    Model model;

    //methods
    unsigned int
    get_num_joints();                                                                                                                                                                                  //Returns the degrees of freedom of the model.
    MatrixNd calc_EndEffectorJacobian(const VectorNd &Q, unsigned int body_id, const Vector3d &body_point_position);                                                                                   //Computes the point jacobian for a point on a body.
    Vector3d calc_EndEffectorVel(const VectorNd &Q, const VectorNd &QDot, unsigned int body_id, const Vector3d &body_point_position);                                                                  //Computes the cartesian velocity of a point on a body(can be used for calculating end_effector's cartesian velocity).
    VectorNd calc_InverseKinematics(const VectorNd &Qinit, const std::vector<unsigned int> body_id, const std::vector<Vector3d> &body_point, const std::vector<Vector3d> &target_pos, VectorNd &Qres); //Computes the inverse kinematics iteratively using a damped Levenberg-Marquardt method (also known as Damped Least Squares method).
    Vector3d calc_ForwardKinematics(const Math::VectorNd &Q, unsigned int body_id, const Math::Vector3d &body_point_position);                                                                         //Method for calculating the cartesian position of a point on a body relative to the base (world).
    MatrixNd calc_M(const VectorNd &Q);                                                                                                                                                                //Computes the joint space inertia matrix by using the Composite Rigid Body Algorithm.
    VectorNd calc_G(const VectorNd &Q);                                                                                                                                                                //Computes the N by 1 gravity matrix of the dynamics model by equating the velocity and acceleration to zero. Note: This method is only for serial manipulators, for parallel systems use InverseDynamicsConstrained
    VectorNd calc_InverseDynamics(const VectorNd &Q, const VectorNd &QDot, const VectorNd &QDDot, VectorNd &Tau);                                                                                      //Computes inverse dynamics with the Newton-Euler Algorithm.
    VectorNd calc_ForwardDynamics(const VectorNd &Q, const VectorNd &QDot, const VectorNd &Tau, VectorNd &QDDot);                                                                                      //Computes forward dynamics with the Articulated Body Algorithm.
    VectorNd calc_InverseDynamicsConstrained(const VectorNd &Q, const VectorNd &QDot, const VectorNd &QDDotDesired, ConstraintSet &CS, VectorNd &QDDotOutput, VectorNd &TauOutput);                    //An inverse-dynamics operator that can be applied to fully-actuated constrained systems.
    VectorNd calc_ForwardDynamicsConstrained(const VectorNd &Q, const VectorNd &QDot, const VectorNd &Tau, ConstraintSet &CS, VectorNd &QDDotOutput);                                                  //Computes forward dynamics with contact by constructing and solving the full lagrangian equation.
    VectorNd calc_MInvTimesTau(const VectorNd &Q, const VectorNd &Tau, VectorNd &QDDot);                                                                                                               //Computes the effect of multiplying the inverse of the joint space inertia matrix with a vector in linear time.
};