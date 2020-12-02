#include "ros/ros.h"
#include "ambf_msgs/ObjectState.h"
#include "ambf_msgs/ObjectCmd.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Header.h"
#include <iostream>
#include "Dynamics.h"
#include <rbdl/rbdl.h>
#include <vector>
#include <string>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class Ambf_sub_pub
{
public:
    Ambf_sub_pub(const ros::Publisher &pub) : pub_(pub), rbdl_obj_("kuka_lbr"), pos_d(), cmd_msg(), header_()
    {
        cmd_msg.enable_position_controller = 0;
        cmd_msg.position_controller_mask = {0};
        pos_d = VectorNd::Zero(rbdl_obj_.model.dof_count);
    }

    void StateCallback(const ambf_msgs::ObjectState::ConstPtr &msg)
    {
        std::vector<float> pos = msg->joint_positions;
        if (pos.size() == pos_d.size())
        {
            for (int i = 0; i < rbdl_obj_.model.dof_count; i++)
            {
                pos_d(i) = static_cast<double>(pos.at(i));
            }
        }
        else if (pos.size() < pos_d.size())
        {
            for (int i = 0; i < rbdl_obj_.model.dof_count - 1; i++)
            {
                pos_d(i) = static_cast<double>(pos.at(i));
            }
        }
        std::vector<float> tau_cmd(8, 0.);
        VectorNd tau_d = VectorNd::Zero(rbdl_obj_.model.q_size);
        VectorNd qdot = VectorNd::Zero(rbdl_obj_.model.qdot_size);
        // tau_d = rbdl_obj_.calc_InverseDynamicsConstrained(pos_d, qdot, qdot, rbdl_obj_.four_bar_obj_.cs, qdot, tau_d);

        tau_d = rbdl_obj_.calc_G(pos_d);

        if (tau_cmd.size() == tau_d.size())
        {
            for (int i = 0; i < rbdl_obj_.model.dof_count; i++)
            {
                tau_cmd.at(i) = static_cast<float>(tau_d(i));
            }
        }
        else if (tau_cmd.size() == tau_d.size())
        {
            for (int i = 0; i < rbdl_obj_.model.dof_count - 1; i++)
            {
                tau_cmd.at(i) = static_cast<float>(tau_d(i));
            }
        };

        header_.stamp = ros::Time::now();
        cmd_msg.header = header_;
        cmd_msg.joint_cmds = tau_cmd;
        pub_.publish(cmd_msg);
    }

private:
    VectorNd pos_d;
    ros::Publisher pub_;
    Dynamics rbdl_obj_;
    ambf_msgs::ObjectCmd cmd_msg;
    std_msgs::Header header_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kuka_lbr");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<ambf_msgs::ObjectCmd>("/ambf/env/base/Command", 1000);
    Ambf_sub_pub obj1(pub);
    ros::Subscriber sub = nh.subscribe("/ambf/env/base/State", 1, &Ambf_sub_pub::StateCallback, &obj1);

    ros::spin();

    return 0;
}