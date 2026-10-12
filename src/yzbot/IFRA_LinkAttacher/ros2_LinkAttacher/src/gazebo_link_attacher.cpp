/*
# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: May, 2023.                                                                     #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) IFRA Gazebo-ROS2 Link Attacher. URL: https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.
*/

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Light.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#include <gazebo_ros/node.hpp>
#include <memory>
#include <mutex>

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"

#include "ros2_linkattacher/gazebo_link_attacher.hpp"   // INCLUDE HADER FILE.
#include <linkattacher_msgs/srv/attach_link.hpp>        // INCLUDE ROS2 SERVICE.
#include <linkattacher_msgs/srv/detach_link.hpp>        // INCLUDE ROS2 SERVICE.

// GLOBAL VARIABLE:
std::vector<JointSTRUCT> GV_joints;
JointSTRUCT GV_jointSTR;

// IsAttached variable:
// Gazebo breaks if -> An attachment request is done between 2 links, and the joint attachment has already been created and not removed!
// Therefore, we have added this variable to make sure the attachment is only requested when the previous attachment has already been removed.
bool IsAttached = false;

// JointName:
std::string JointName = "None";

namespace gazebo_ros
{

class GazeboLinkAttacherPrivate
{
public:

  // ATTACH (ROS2 service):
  void Attach(
    linkattacher_msgs::srv::AttachLink::Request::SharedPtr _req,
    linkattacher_msgs::srv::AttachLink::Response::SharedPtr _res);

  // DETACH (ROS2 service):
  void Detach(
    linkattacher_msgs::srv::DetachLink::Request::SharedPtr _req,
    linkattacher_msgs::srv::DetachLink::Response::SharedPtr _res);

  // Keep the carried object at its grasp-relative pose without creating a
  // force-transmitting constraint between the arm and the mobile base.
  void OnUpdate();

  // World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// ROS node for communication, managed by gazebo_ros.
  gazebo_ros::Node::SharedPtr ros_node_;

  // ROS services to handle requests for attach/detach.
  rclcpp::Service<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_link_service_;
  rclcpp::Service<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_link_service_;

  gazebo::event::ConnectionPtr update_connection_;
  std::mutex attachment_mutex_;
  JointSTRUCT active_attachment_;
  ignition::math::Pose3d relative_pose_;

  // getJoint function:
  bool getJoint(std::string M1, std::string L1, std::string M2, std::string L2, JointSTRUCT &joint);

};

GazeboLinkAttacher::GazeboLinkAttacher()
: impl_(std::make_unique<GazeboLinkAttacherPrivate>())
{
}

GazeboLinkAttacher::~GazeboLinkAttacher()
{
}

void GazeboLinkAttacher::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  
  // Gazebo WORLD:
  impl_->world_ = _world;

  // ROS2 NODE:
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // ROS2 SERVICE SERVERS:
  impl_->attach_link_service_ =
    impl_->ros_node_->create_service<linkattacher_msgs::srv::AttachLink>(
    "ATTACHLINK", std::bind(
      &GazeboLinkAttacherPrivate::Attach, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));
  impl_->detach_link_service_ =
    impl_->ros_node_->create_service<linkattacher_msgs::srv::DetachLink>(
    "DETACHLINK", std::bind(
      &GazeboLinkAttacherPrivate::Detach, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboLinkAttacherPrivate::OnUpdate, impl_.get()));

}

void GazeboLinkAttacherPrivate::OnUpdate()
{
  std::lock_guard<std::mutex> lock(attachment_mutex_);
  if (!IsAttached || !active_attachment_.l1 || !active_attachment_.l2 ||
      !active_attachment_.m2) {
    return;
  }

  const auto target_pose = active_attachment_.l1->WorldPose() * relative_pose_;
  active_attachment_.m2->SetWorldPose(target_pose);
  active_attachment_.l2->SetLinearVel(ignition::math::Vector3d::Zero);
  active_attachment_.l2->SetAngularVel(ignition::math::Vector3d::Zero);
}

void GazeboLinkAttacherPrivate::Attach(
  linkattacher_msgs::srv::AttachLink::Request::SharedPtr _req,
  linkattacher_msgs::srv::AttachLink::Response::SharedPtr _res)
{

  // CHECK if -> Joint already exists in GV_joints:
  /* THIS IS NO LONGER NEEDED, SINCE THE JOINT IS REMOVED AFTER DETACHING!
  JointSTRUCT j;
  if (this->getJoint(_req->model1_name, _req->link1_name, _req->model2_name, _req->link2_name, j)){
    j.joint->Attach(j.l1, j.l2);
    _res->success = true;
    _res->message = "ATTACHED: {MODEL , LINK} -> {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.";
    return;
  }
  */

  // Get the first link:
  gazebo::physics::ModelPtr model1 = world_->ModelByName(_req->model1_name);
  if (!model1) {
    _res->success = false;
    _res->message = "Failed to find model with name: " + _req->model1_name;
    return;
  }
  gazebo::physics::LinkPtr link1 = model1->GetLink(_req->link1_name);
  if (!link1) {
    _res->success = false;
    _res->message = "Failed to find link with name: " + _req->link1_name;
    return;
  }

  // Get the second link:
  gazebo::physics::ModelPtr model2 = world_->ModelByName(_req->model2_name);
  if (!model2) {
    _res->success = false;
    _res->message = "Failed to find model with name: " + _req->model2_name;
    return;
  }
  gazebo::physics::LinkPtr link2 = model2->GetLink(_req->link2_name);
  if (!link2) {
    _res->success = false;
    _res->message = "Failed to find link with name: " + _req->link2_name;
    return;
  }

  if (IsAttached == true){

    _res->success = false;
    _res->message = "Both links have already been attached, aborting new attachment.";

  } else {

    std::lock_guard<std::mutex> lock(attachment_mutex_);

    // A fixed joint between an articulated arm and a mobile base can become
    // numerically unstable in Gazebo. Use a kinematic follower while keeping
    // the same service contract and all pre-attach verification.
    link2->SetCollideMode("none");
    link2->SetGravityMode(false);
    link2->SetKinematic(true);
    JointName = _req->model1_name + "_" + _req->link1_name + "_" + _req->model2_name + "_" + _req->link2_name + "_joint";

    GV_jointSTR.model1 = _req->model1_name;
    GV_jointSTR.model2 = _req->model2_name;
    GV_jointSTR.link1 = _req->link1_name;
    GV_jointSTR.link2 = _req->link2_name;
    GV_jointSTR.m1 = model1;
    GV_jointSTR.m2 = model2;
    GV_jointSTR.l1 = link1;
    GV_jointSTR.l2 = link2;
    GV_jointSTR.joint.reset();

    relative_pose_ = link1->WorldPose().Inverse() * link2->WorldPose();
    active_attachment_ = GV_jointSTR;
    
    GV_joints.push_back(GV_jointSTR);

    // Set the success and message in the response:
    _res->success = true;
    _res->message = "ATTACHED: {MODEL , LINK} -> {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.";

    IsAttached = true;

  }

}

void GazeboLinkAttacherPrivate::Detach(
  linkattacher_msgs::srv::DetachLink::Request::SharedPtr _req,
  linkattacher_msgs::srv::DetachLink::Response::SharedPtr _res)
{

  // CHECK if -> Joint already exists in GV_joints:
  JointSTRUCT j;
  if (this->getJoint(_req->model1_name, _req->link1_name, _req->model2_name, _req->link2_name, j)){
    std::lock_guard<std::mutex> lock(attachment_mutex_);
    IsAttached = false;
    j.l2->SetKinematic(false);
    j.l2->SetGravityMode(true);
    j.l2->SetCollideMode("all");
    j.l2->SetLinearVel(ignition::math::Vector3d::Zero);
    j.l2->SetAngularVel(ignition::math::Vector3d::Zero);
    active_attachment_ = JointSTRUCT();
    _res->success = true;
    _res->message = "DETACHED: {MODEL , LINK} -> {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.";

    // Removing a fixed joint inside the service callback can invalidate
    // Gazebo event connections and crash gzserver. Competition cubes are
    // attached once per world, so detached joint objects can live until the
    // world is torn down.

    return;
  } else {
    _res->success = false;
    _res->message = "DETACHED -- ERROR (Joint does not exist!): {MODEL , LINK} -> {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.";
  }

}

bool GazeboLinkAttacherPrivate::getJoint(std::string M1, std::string L1, std::string M2, std::string L2, JointSTRUCT &joint)
  {
    JointSTRUCT j;
    for(std::vector<JointSTRUCT>::iterator it = GV_joints.begin(); it != GV_joints.end(); ++it){
      j = *it;
      if ((j.model1.compare(M1) == 0) && (j.model2.compare(M2) == 0) && (j.link1.compare(L1) == 0) && (j.link2.compare(L2) == 0)){
        joint = j;
        return true;
      }
    }
    return false;
  }

GZ_REGISTER_WORLD_PLUGIN(GazeboLinkAttacher)

}  // namespace gazebo_ros
