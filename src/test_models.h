#pragma once
#include <rbdl/rbdl.h>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
struct Four_bar_linkage
{
    Four_bar_linkage() : model(), Q(), QDot(), QDDot(), mass(1), Tau(), body_a_id(), body_b_id(), body_c_id(), body_d_id(), body_virtual_id(), X_s(Xtrans(Vector3d(0., 0., 0.))), X_p(), cs(), virtual_mass(0)
    {

        model.gravity = Vector3d(0., 0., -9.81);

        Vector3d com(0., -0.344, 0.);
        Vector3d inertia(0.0589073140693, 0.00329549139377, 0.0589073140693);
        Body body_a, body_b, body_c, body_d;
        Joint joint_a, joint_b, joint_c, joint_d, joint_v;

        // body a
        joint_a = Joint(JointTypeFixed);
        body_a = Body(mass, com, inertia); /* mass, com, inertia*/
                                           /* **note** the rotation matrix is a 3*3 concise representation of a 6*6 spacial matrix.
          The rotation matrix should be inserted as a transpose of the rotation matrix.*/
        Matrix3_t body_a_rot;
        body_a_rot << 0., 0., 1.,
            0., -1., 0.,
            1., 0., 0.;
        Vector3d body_a_trans(0.001, -0.36, -0.222);
        SpatialTransform body_a_tf(body_a_rot, body_a_trans);
        body_a_id = model.AddBody(0, body_a_tf, joint_a, body_a);
        // body b
        body_b = Body(mass, com, inertia);
        joint_b = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));
        Matrix3_t body_b_rot;
        body_b_rot << 0., 1., 0.,
            -1., 0., 0.,
            0., 0., 1.;
        Vector3d body_b_trans(0.139, 0.138, 0.);
        SpatialTransform body_b_tf(body_b_rot, body_b_trans);
        body_b_id = model.AddBody(body_a_id, body_b_tf, joint_b, body_b);
        // body c
        body_c = Body(mass, com, inertia);
        joint_c = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));
        Matrix3_t body_c_rot;
        body_c_rot << 0., -1., 0.,
            1., 0., 0.,
            0., 0., 1.;
        Vector3d body_c_trans(-0.141, -0.832, 0.);
        SpatialTransform body_c_tf(body_c_rot, body_c_trans);
        body_c_id = model.AddBody(body_b_id, body_c_tf, joint_c, body_c);
        // body d
        body_d = Body(mass, com, inertia);
        joint_d = Joint(
            JointTypeRevolute,
            Vector3d(0., 0., 1.));

        Matrix3_t body_d_rot;
        body_d_rot << 0., -1., 0.,
            1., 0., 0.,
            0., 0., 1.;
        Vector3d body_d_trans(-0.14, -0.83, 0.);
        SpatialTransform body_d_tf(body_d_rot, body_d_trans);
        body_d_id = model.AddBody(body_c_id, body_d_tf, joint_d, body_d);
        // virtual body
        Vector3d vector3d_zero = Vector3d::Zero();
        Body body_v(virtual_mass, vector3d_zero, vector3d_zero);  // creating the virtual body
        joint_v = Joint(JointTypeRevolute, Vector3d(0., 0., 1.)); // revolute about z, also Joint(JointTypeRevoluteZ); is OK.
        Matrix3_t body_v_rot;
        body_v_rot << 1., 0., 0.,
            0., 1., 0.,
            0., 0., 1.;
        Vector3d body_v_trans(-0.062, -0.761, 0.); // child pivot is used Joint l1-l4
        SpatialTransform body_v_tf(body_v_rot, body_v_trans);
        body_virtual_id = model.AddBody(body_d_id, body_v_tf, joint_v, body_v);
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        Matrix3_t p_rot;
        p_rot << 0., -1., 0.,
            1., 0., 0.,
            0., 0., 1.;
        Vector3d p_trans(0.07, -0.77, 0.);
        SpatialTransform p_tf(p_rot, p_trans); // predecessor body is l1
        X_p = p_tf;

        cs.AddLoopConstraint(body_a_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 0, 1, 0, 0), true, 0.05);
        cs.AddLoopConstraint(body_a_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 0, 0, 1, 0), true, 0.05);
        cs.AddLoopConstraint(body_a_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 1, 0, 0, 0), true, 0.05);

        cs.Bind(model);
        //
        Q = VectorNd::Zero(model.dof_count);
        QDot = VectorNd::Zero(model.dof_count);
        Tau = VectorNd::Zero(model.dof_count);
        QDDot = VectorNd::Zero(model.dof_count);
    }

    Model model;
    ConstraintSet cs;
    VectorNd Q;
    VectorNd QDot;
    VectorNd QDDot;
    VectorNd Tau;
    SpatialTransform X_p;
    SpatialTransform X_s;
    double mass, virtual_mass;
    unsigned int body_a_id, body_b_id, body_c_id, body_d_id, body_virtual_id;
};
struct KUKA_LBR
{
    KUKA_LBR() : model(), mass(1), Q(), QDot(), QDDot(), Tau(), body_base_id(), body_l1_id(), body_l2_id(), body_l3_id(), body_l4_id(), body_l5_id(), body_l6_id(), body_l7_id()
    {

        model.gravity = Vector3d(0., 0., -9.81);
        Body body_base, body_l1, body_l2, body_l3, body_l4, body_l5, body_l6, body_l7;
        Joint Joint_RevZ, Joint_Fixed;
        Joint_Fixed = Joint(JointTypeFixed);
        Joint_RevZ = Joint(JointTypeRevoluteZ);

        // body base
        body_base = Body(mass, Vector3d(0., 0., 0.), Vector3d(0., 0., 0.)); /* mass, com, inertia*/
        Matrix3_t world_base_rot;
        /* **note** the rotation matrix is a 3*3 concise representation of a 6*6 spacial matrix.
          The rotation matrix should be inserted as a transpose of the rotation matrix.*/
        world_base_rot << 1., 0., 0.,
            0., 1., 0.,
            0., 0., 1.;
        Vector3d world_base_trans(0., 0., 0.);
        SpatialTransform world_base_tf(world_base_rot, world_base_trans);
        body_base_id = model.AddBody(0, world_base_tf, Joint_Fixed, body_base);
        // body l1
        body_l1 = Body(mass, Vector3d(0.0, -0.017, 0.134), Vector3d(0.00815814, 0.007363868, 0.003293455)); /* mass, com, inertia*/
        Matrix3_t base_l1_rot;
        base_l1_rot << 1., 0., 0.,
            0., 1., 0.,
            0., 0., 1.;
        Vector3d base_l1_trans(0., 0., 0.103);
        SpatialTransform base_l1_tf(base_l1_rot, base_l1_trans);
        body_l1_id = model.AddBody(body_base_id, base_l1_tf, Joint_RevZ, body_l1);
        // body l2
        body_l2 = Body(mass, Vector3d(0.0, -0.074, 0.009), Vector3d(0.00812252, 0.00329668, 0.00733904)); /* mass, com, inertia*/
        Matrix3_t l1_l2_rot;
        l1_l2_rot << 1., 0., 0.,
            0., 0., -1.,
            0., 1., 0.;
        Vector3d l1_l2_trans(0.0, 0.013, 0.209);
        SpatialTransform l1_l2_tf(l1_l2_rot, l1_l2_trans);
        body_l2_id = model.AddBody(body_l1_id, l1_l2_tf, Joint_RevZ, body_l2);
        // body l3
        body_l3 = Body(mass, Vector3d(0.0, 0.017, 0.134), Vector3d(0.008159, 0.007421, 0.00330)); /* mass, com, inertia*/
        Matrix3_t l2_l3_rot;
        l2_l3_rot << 1., 0., 0.,
            0., 0., 1.,
            0., -1., 0.;
        Vector3d l2_l3_trans(0.0, -0.194, -0.009);
        SpatialTransform l2_l3_tf(l2_l3_rot, l2_l3_trans);
        body_l3_id = model.AddBody(body_l2_id, l2_l3_tf, Joint_RevZ, body_l3);
        // body l4
        body_l4 = Body(mass, Vector3d(-0.001, 0.081, 0.008), Vector3d(0.0081471, 0.003297, 0.0073715)); /* mass, com, inertia*/
        Matrix3_t l3_l4_rot;
        l3_l4_rot << 1., 0., 0.,
            0., 0., 1.,
            0., -1., 0.;
        Vector3d l3_l4_trans(0.0, -0.013, 0.202);
        SpatialTransform l3_l4_tf(l3_l4_rot, l3_l4_trans);
        body_l4_id = model.AddBody(body_l3_id, l3_l4_tf, Joint_RevZ, body_l4);
        // body l5
        body_l5 = Body(mass, Vector3d(0.0, -0.017, 0.129), Vector3d(0.0077265, 0.006950, 0.00329)); /* mass, com, inertia*/
        Matrix3_t l4_l5_rot;
        l4_l5_rot << 1., 0., 0.,
            0., 0., -1.,
            0., 1., 0.;
        Vector3d l4_l5_trans(-0.002, 0.202, -0.008);
        SpatialTransform l4_l5_tf(l4_l5_rot, l4_l5_trans);
        body_l5_id = model.AddBody(body_l4_id, l4_l5_tf, Joint_RevZ, body_l5);
        // body l6
        body_l6 = Body(mass, Vector3d(0.0, 0.007, 0.068), Vector3d(0.002983, 0.003299, 0.003146)); /* mass, com, inertia*/
        Matrix3_t l5_l6_rot;
        l5_l6_rot << 1., 0., 0.,
            0., 0., -1.,
            0., 1., 0.;
        Vector3d l5_l6_trans(0.002, -0.052, 0.204);
        SpatialTransform l5_l6_tf(l5_l6_rot, l5_l6_trans);
        body_l6_id = model.AddBody(body_l5_id, l5_l6_tf, Joint_RevZ, body_l6);
        // body l7
        body_l7 = Body(mass, Vector3d(0.006, 0.0, 0.015), Vector3d(0.000651, 0.0006512, 0.00112)); /* mass, com, inertia*/
        Matrix3_t l6_l7_rot;
        l6_l7_rot << 1., 0., 0.,
            0., 0., 1.,
            0., -1., 0.;
        Vector3d l6_l7_trans(-0.003, -0.05, 0.053);
        SpatialTransform l6_l7_tf(l6_l7_rot, l6_l7_trans);
        body_l7_id = model.AddBody(body_l6_id, l6_l7_tf, Joint_RevZ, body_l7);
        Q = VectorNd::Zero(model.dof_count);
        QDot = VectorNd::Zero(model.dof_count);
        Tau = VectorNd::Zero(model.dof_count);
        QDDot = VectorNd::Zero(model.dof_count);
    }

    Model model;
    VectorNd Q;
    VectorNd QDot;
    VectorNd QDDot;
    VectorNd Tau;
    double mass;
    unsigned int body_l1_id, body_l2_id, body_l3_id, body_l4_id, body_l5_id, body_l6_id, body_l7_id, body_base_id;
};
struct MTM_closed
{
    MTM_closed() : model(), Q(), QDot(), QDDot(), Tau(), body_a_id(), body_b_id(), body_c_id(), body_d_id(), body_e_id(), body_f_id(), body_g_id(), body_h_id(), body_i_id(), body_j_id(), body_virtual_id(), X_s(Xtrans(Vector3d(0., 0., 0.))), X_p(), cs(), virtual_mass(0)
    {
        using namespace RigidBodyDynamics;
        using namespace RigidBodyDynamics::Math;
        model.gravity = Vector3d(0., 0., -9.81);
        Body body_a, body_b, body_c, body_d, body_e, body_f, body_g, body_h, body_i, body_j;
        Joint joint_rev_z, joint_fix, joint_v;
        joint_fix = Joint(JointTypeFixed);
        joint_rev_z = Joint(JointTypeRevoluteZ);
        joint_v = Joint(JointTypeRevoluteZ);

        // body a top panel
        Vector3d com_a(-0.008, 0.064, 0.028);
        Vector3d inertia_a(0.01, 0.01, 0.01);
        body_a = Body(1., com_a, inertia_a); /* mass, com, inertia*/
        Matrix3_t body_a_rot;
        body_a_rot << 1., 0., 0.,
            0., 1., 0.,
            0., 0., 1.;
        Vector3d body_a_trans(0.0, 0.0, 0.0);
        SpatialTransform body_a_tf(body_a_rot, body_a_trans);
        body_a_id = model.AddBody(0, body_a_tf, joint_fix, body_a);
        // body b shoulder pitch
        Vector3d com_b(0.003, 0.035, -0.119);
        Vector3d inertia_b(0.00917612905846, 0.00935165844593, 0.00447358060957);
        body_b = Body(1., com_b, inertia_b);
        Matrix3_t body_b_rot;
        body_b_rot << 0.0, 1.0, 0.0,
            -1.0, 0.0, 0.0,
            0.0, 0.0, 1.0;
        Vector3d body_b_trans(0.0, 0.0, 0.);
        SpatialTransform body_b_tf(body_b_rot, body_b_trans);
        body_b_id = model.AddBody(body_a_id, body_b_tf, joint_rev_z, body_b);
        // body c arm parallel
        Vector3d com_c(-0.109, 0.012, 0.02);
        Vector3d inertia_c(0.0028003998026, 0.0134169293445, 0.0113575925399);
        body_c = Body(0.9, com_c, inertia_c);
        Matrix3_t body_c_rot;
        body_c_rot << 0.0, 0.0, 1.0,
            1.0, 0.005, 0.0,
            -0.005, 1.0, 0.0;
        Vector3d body_c_trans(0.0, 0.0, -0.19);
        SpatialTransform body_c_tf(body_c_rot, body_c_trans);
        body_c_id = model.AddBody(body_b_id, body_c_tf, joint_rev_z, body_c);
        // body d arm parallel 1
        Vector3d com_d(0.038, 0.0, -0.007);
        Vector3d inertia_d(0.000104513350282, 0.000324014608013, 0.000373281422509);
        body_c = Body(0.15, com_d, inertia_d);
        Matrix3_t body_d_rot;
        body_d_rot << -1, 0.0, 0.0,
            0.0, 0.0, 1.0,
            0.0, 1.0, 0.0;
        Vector3d body_d_trans(-0.0004, 0.065, -0.19);
        SpatialTransform body_d_tf(body_d_rot, body_d_trans);
        body_d_id = model.AddBody(body_b_id, body_d_tf, joint_rev_z, body_d);
        // body e arm front
        Vector3d com_e(0.0, -0.14, -0.007);
        Vector3d inertia_e(0.00110227006662, 0.00000772914692154, 0.00110194245711);
        body_e = Body(0.15, com_e, inertia_e);
        Matrix3_t body_e_rot;
        body_e_rot << 1.0, 0.0, 0.,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0;
        Vector3d body_e_trans(0.101, 0.0, -0.03);
        SpatialTransform body_e_tf(body_e_rot, body_e_trans);
        body_e_id = model.AddBody(body_d_id, body_e_tf, joint_rev_z, body_e);
        // virtual body end of arm front joint location with bottom arm
        Vector3d vector3d_zero = Vector3d::Zero();
        Body body_v(virtual_mass, vector3d_zero, vector3d_zero);  // creating the virtual body
        joint_v = Joint(JointTypeRevolute, Vector3d(0., 0., 1.)); // revolute about z, also Joint(JointTypeRevoluteZ); is OK.
        Matrix3_t body_v_rot;                                     //changed
        body_v_rot << -1.0, -0.0004, -0.002,
            0.0004, -1.0, 0.0,
            -0.002, 0.0, 1.0;
        Vector3d body_v_trans(0.0, -0.28, -0.035);
        SpatialTransform body_v_tf(body_v_rot, body_v_trans);
        body_virtual_id = model.AddBody(body_e_id, body_v_tf, joint_v, body_v);
        // body f(bottom Arm)
        Vector3d com_f(-0.188, -0.008, 0.0);
        Vector3d inertia_f(0.000255941352746, 0.0105760140742, 0.0105499806308);
        body_f = Body(0.8, com_f, inertia_f);
        Matrix3_t body_f_rot;
        body_f_rot << -0.0002, 1.0, 0.0,
            -1.0, -0.0002, 0.0,
            0.0, 0.0, 1.0;
        Vector3d body_f_trans(-0.279, -0.0, 0.0);
        SpatialTransform body_f_tf(body_f_rot, body_f_trans);
        body_f_id = model.AddBody(body_c_id, body_f_tf, joint_rev_z, body_f);
        // body g wrist platform
        Vector3d com_g(0.0, -0.055, -0.054);
        Vector3d inertia_g(0.00154079803571, 0.000741331628791, 0.000993883092147);
        body_g = Body(0.4, com_g, inertia_g);
        Matrix3_t body_g_rot;
        body_g_rot << 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0,
            0.0, -1.0, 0.0;
        Vector3d body_g_trans(-0.364, -0.15, -0.0);
        SpatialTransform body_g_tf(body_g_rot, body_g_trans);
        body_g_id = model.AddBody(body_f_id, body_g_tf, joint_rev_z, body_g);
        // body h wrist pitch
        Vector3d com_h(0.0, 0.041, -0.037);
        Vector3d inertia_h(0.000397858856814, 0.000210744513123, 0.000223359835719);
        body_h = Body(0.2, com_h, inertia_h);
        Matrix3_t body_h_rot;
        body_h_rot << 1.0, 0.0, 0.0,
            0.0, 0.0, -1.0,
            0.0, 1.0, 0.0;
        Vector3d body_h_trans(0.0, 0.0, 0.0);
        SpatialTransform body_h_tf(body_h_rot, body_h_trans);
        body_h_id = model.AddBody(body_g_id, body_h_tf, joint_rev_z, body_h);
        // body i wrist yaw
        Vector3d com_i(-0.109, 0.012, 0.02);
        Vector3d inertia_i(0.000249325233144, 0.000131620327094, 0.000131620327094);
        body_i = Body(0.2, com_i, inertia_i);
        Matrix3_t body_i_rot;
        body_i_rot << -0.0002, 0.0, 1.0,
            -1.0, 0.0, -0.0002,
            0.0, -1.0, 0.0;
        Vector3d body_i_trans(0.0, 0.002, 0.);
        SpatialTransform body_i_tf(body_i_rot, body_i_trans);
        body_i_id = model.AddBody(body_h_id, body_i_tf, joint_rev_z, body_i);
        // body j wrist roll
        Vector3d com_j(0.0, 0.0, -0.036);
        Vector3d inertia_j(0.00006, 0.000056, 0.000029);
        body_j = Body(0.1, com_j, inertia_j);
        Matrix3_t body_j_rot;
        body_j_rot << -0.0002, 0.0, -1.0,
            1.0, 0.009, -0.0002,
            0.009, -1.0, 0.0;
        Vector3d body_j_trans(0.0, -0.039, -0.0);
        SpatialTransform body_j_tf(body_j_rot, body_j_trans);
        body_j_id = model.AddBody(body_i_id, body_j_tf, joint_rev_z, body_j);

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        Matrix3_t p_rot;
        p_rot << 1., 0., 0.,
            0., 1., 0.,
            0., 0., 1.;
        Vector3d p_trans(-0.101, 0.001, 0.0);
        SpatialTransform p_tf(p_rot, p_trans); // predecessor body is Bottom arm
        X_p = p_tf;

        cs.AddLoopConstraint(body_f_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 0, 1, 0, 0), true, 0.1);
        cs.AddLoopConstraint(body_f_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 0, 0, 1, 0), true, 0.1);
        cs.AddLoopConstraint(body_f_id, body_virtual_id, X_p, X_s,
                             SpatialVector(0, 0, 1, 0, 0, 0), true, 0.1);

        cs.Bind(model);
        //
        Q = VectorNd::Zero(model.dof_count);
        QDot = VectorNd::Zero(model.dof_count);
        Tau = VectorNd::Zero(model.dof_count);
        QDDot = VectorNd::Zero(model.dof_count);
        std::cout << model.dof_count << "is dof size" << std::endl;
    }

    RigidBodyDynamics::Model model;
    ConstraintSet cs;
    RigidBodyDynamics::Math::VectorNd Q;
    RigidBodyDynamics::Math::VectorNd QDot;
    RigidBodyDynamics::Math::VectorNd QDDot;
    RigidBodyDynamics::Math::VectorNd Tau;
    SpatialTransform X_p;
    SpatialTransform X_s;
    double mass, virtual_mass;
    unsigned int body_a_id, body_b_id, body_c_id, body_d_id, body_virtual_id, body_e_id, body_f_id, body_g_id, body_h_id, body_i_id, body_j_id;
};
struct Raven_II
{
    Raven_II() : model(), Q(), QDot(), QDDot(), Tau(), body_a_id(), body_b_id(), body_c_id(), body_d_id(), body_e_id(), body_f_id(), body_g_id(), body_h_id(), body_i_id(), mass_c(0.5031), mass_d(0.7503), mass_e(0.4066), mass_f(0.03), mass_g(0.005), mass_h(0.005), mass_i(0.005)
    {
        using namespace RigidBodyDynamics;
        using namespace RigidBodyDynamics::Math;
        model.gravity = Vector3d(0., 0., -9.81);
        Body body_a, body_b, body_c, body_d, body_e, body_f, body_g, body_h, body_i;
        Joint joint_rev_z, joint_fix, joint_prismatic;
        joint_fix = Joint(JointTypeFixed);
        joint_rev_z = Joint(JointTypeRevoluteZ);
        Math::Vector3d prismatic_joint_Z_axis(1, 0, 0);
        joint_prismatic = Joint(JointTypePrismatic, prismatic_joint_Z_axis);

        // body a 0_link
        Vector3d com_a(0.0, 0.0, 0.0);
        Vector3d inertia_a(0.01, 0.01, 0.01); // Fixed Link; Set to an arbitrary value.
        body_a = Body(1., com_a, inertia_a);  /*mass, com, inertia*/
        Matrix3_t body_a_rot;
        body_a_rot << 1., 0., 0.,
            0., 1., 0.,
            0., 0., 1.;
        Vector3d body_a_trans(0.0, 0.0, 0.0);
        SpatialTransform body_a_tf(body_a_rot, body_a_trans);
        body_a_id = model.AddBody(0, body_a_tf, joint_fix, body_a); // world_frame -> Top_Panel
        // body b BaseLink
        Vector3d com_b(0.101, 0.003, -0.343);
        Vector3d inertia_b(0.01, 0.01, 0.01); // Fixed Link; Set to an arbitrary value.
        body_b = Body(1., com_b, inertia_b);
        Matrix3_t body_b_rot;
        body_b_rot << 0.0, -.4223, 0.9064,
            0.0, 0.9064, 0.4223,
            -1.0, 0.0, 0.0;
        Vector3d body_b_trans(0.68, 0.0, 0.0);
        SpatialTransform body_b_tf(body_b_rot, body_b_trans);
        body_b_id = model.AddBody(body_a_id, body_b_tf, joint_fix, body_b);
        // body c link1_L
        Vector3d com_c(0.00691, -0.09869, -0.13755);
        Vector3d inertia_c(0.0033749403655041078, 0.001977863356079949, 0.0021270204561928616);
        body_c = Body(mass_c, com_c, inertia_c);
        Matrix3_t body_c_rot;
        body_c_rot << 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0;
        Vector3d body_c_trans(0.0, 0.0, 0.0);
        SpatialTransform body_c_tf(body_c_rot, body_c_trans);
        body_c_id = model.AddBody(body_b_id, body_c_tf, joint_rev_z, body_c);
        // body d Link2_L
        Vector3d com_d(-0.1791, -0.0012, -0.2339);
        Vector3d inertia_d(0.00967712207073078, 0.022389501955926035, 0.017408573487153342);
        body_c = Body(mass_d, com_d, inertia_d);
        Matrix3_t body_d_rot;
        body_d_rot << 0.2591, 0.3577, -0.8972,
            -0.9013, -0.2443, -0.3577,
            -0.3471, 0.9013, 0.2591;
        Vector3d body_d_trans(0.0, 0.0, 0.0);
        SpatialTransform body_d_tf(body_d_rot, body_d_trans);
        body_d_id = model.AddBody(body_c_id, body_d_tf, joint_rev_z, body_d);
        // body e Link3_L
        Vector3d com_e(0.02, 0.0002, -0.013);
        Vector3d inertia_e(0.0004467903226185586, 0.0009051225993916976, 0.0008211608194419769);
        body_e = Body(mass_e, com_e, inertia_e);
        Matrix3_t body_e_rot;
        body_e_rot << -0.732, -0.2910, -0.6160,
            0.6667, -0.1197, -0.7357,
            0.1403, -0.9492, 0.2816;
        Vector3d body_e_trans(0.0, 0.0, 0.0);
        SpatialTransform body_e_tf(body_e_rot, body_e_trans);
        body_e_id = model.AddBody(body_d_id, body_e_tf, joint_fix, body_e);

        // body f instrument_shaft_L
        Vector3d com_f(0.0, 0.0, 0.203);
        Vector3d inertia_f(0.00042169818913851244, 0.0004217093404704866, 0.000000724249554848345);
        body_f = Body(mass_f, com_f, inertia_f);
        Matrix3_t body_f_rot;
        body_f_rot << 0.0, 0.9964, 0.0851,
            0.0, -0.0851, 0.9964,
            1.0, 0.0, 0.0;
        Vector3d body_f_trans(0.0, -0.0, 0.0);
        SpatialTransform body_f_tf(body_f_rot, body_f_trans);
        body_f_id = model.AddBody(body_e_id, body_f_tf, joint_rev_z, body_f);
        // body g wrist_L
        Vector3d com_g(0.003, 0.005, 0.0);
        Vector3d inertia_g(0.00000020095525644163873, 0.00000013945284783322136, 0.00000030822521083055677);
        body_g = Body(mass_g, com_g, inertia_g);
        Matrix3_t body_g_rot;
        body_g_rot << 0.8211, 0.2840, -0.4951,
            -0.4671, -0.1640, -0.8689,
            -0.3279, 0.9447, -0.002;
        Vector3d body_g_trans(0.0, 0.0, 0.0);
        SpatialTransform body_g_tf(body_g_rot, body_g_trans);
        body_g_id = model.AddBody(body_f_id, body_g_tf, joint_rev_z, body_g);
        // body h grasper1_L
        Vector3d com_h(-0.001, 0.008, -0.001);
        Vector3d inertia_h(0.00000031410672396849284, 0.0000009660967365269243, 0.000000362228221890901);
        body_h = Body(mass_h, com_h, inertia_h);
        Matrix3_t body_h_rot;
        body_h_rot << -0.13120, -0.2302, 0.9643,
            0.4773, 0.8378, 0.2650,
            -0.8689, 0.4950, 0.0;
        Vector3d body_h_trans(0.005, 0.011, -0.001);
        SpatialTransform body_h_tf(body_h_rot, body_h_trans);
        body_h_id = model.AddBody(body_g_id, body_h_tf, joint_rev_z, body_h);
        // body i grasper2_L
        Vector3d com_i(-0.109, 0.012, 0.02);
        Vector3d inertia_i(0.0000023204375151742027, 0.00000019345183535918817, 0.0000003762236684889556);
        body_i = Body(mass_i, com_i, inertia_i);
        Matrix3_t body_i_rot;
        body_i_rot << 0.3427, 0.6016, -0.7215,
            0.3571, 0.6269, 0.6924,
            0.8689, -0.4950, 0.0;
        Vector3d body_i_trans(0.005, 0.011, -0.001);
        SpatialTransform body_i_tf(body_i_rot, body_i_trans);
        body_i_id = model.AddBody(body_g_id, body_i_tf, joint_rev_z, body_i);
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        Q = VectorNd::Zero(model.dof_count);
        QDot = VectorNd::Zero(model.dof_count);
        Tau = VectorNd::Zero(model.dof_count);
        QDDot = VectorNd::Zero(model.dof_count);
        std::cout << "\n"
                  << model.dof_count;
    }

    Model model;
    VectorNd Q;
    VectorNd QDot;
    VectorNd QDDot;
    VectorNd Tau;
    double mass_c, mass_d, mass_e, mass_f, mass_g, mass_h, mass_i;
    unsigned int body_a_id, body_b_id, body_c_id, body_d_id, body_e_id, body_f_id, body_g_id, body_h_id, body_i_id;
};