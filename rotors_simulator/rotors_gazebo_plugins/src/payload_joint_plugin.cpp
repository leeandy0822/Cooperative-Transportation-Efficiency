#include <ros/ros.h>

#include <string>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/physics/ModelState.hh>
#include <gazebo/physics/LinkState.hh>




geometry_msgs::Point break_joint, uav2_psi_groundtruth;
void break_joint_cb(const geometry_msgs::Point::ConstPtr& msg){
  break_joint = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tttt");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}




namespace gazebo
{

class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
  }

  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    this->world = _world;
    this->updateRate = common::Time(0,common::Time::SecToNano(3));
    this->prevUpdateTime = common::Time::GetWallTime();
    this->rosSub = this->rosNode.subscribe<geometry_msgs::Point>("/break_joint", 2, break_joint_cb);
    this->rosPub = this->rosNode.advertise<geometry_msgs::Point>("/uav2_psi_groundtruth", 2);
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
               boost::bind(&WorldPluginTutorial::OnUpdate, this, _1));
    //this->iris_model = this->world->GetModel("iris1");
    //this->iris_base_link = this->iris_model->GetLink("iris1::base_link");
  }


  public: void OnUpdate(const common::UpdateInfo & /*_info*/)
  { 

    if(common::Time::GetWallTime() - this->prevUpdateTime > 15){
      this->payload_model =  this->world->ModelByName("payload");
      this->payload_g = this->payload_model->GetLink("payload::payload_rec_g_box");
      this->payload_g1 = this->payload_model->GetLink("payload::payload_rec_g1_box");

    }

  //Double drones add joint

    double x;


    if(common::Time::GetWallTime() - this->prevUpdateTime > 10 && add_inv_2){
    ignition::math::Vector3d joint_axis(0,1,0), joint_axis2(1,0,0),joint_axis3(0,0,1);

    this->iris_model =  this->world->ModelByName("firefly1");
    this->iris_base_link = this->iris_model->GetLink("firefly1::firefly1/base_link");
    this->payload_model =  this->world->ModelByName("payload");
    this->payload_model->SetStatic(false);
    this->payload_link = this->payload_model->GetLink("payload::payload_link1_box");
    this->joint_ = this->world.get()->Physics()->CreateJoint("fixed", this->iris_model);
    this->joint_->Load(this->iris_base_link,this->payload_link,  ignition::math::Pose3d(0,0,0,0,0,0));
    this->joint_->Attach(this->iris_base_link,this->payload_link);
    this->joint_->SetAxis(0, joint_axis2);
    this->joint_->SetAxis(1, joint_axis);
    this->joint_->SetAxis(2, joint_axis3);
    this->joint_->SetName("payload_drone_joint");
    ROS_INFO("add joint1");

    this->iris_model2=  this->world->ModelByName("firefly2");
    this->iris_base_link2 = this->iris_model2->GetLink("firefly2::firefly2/base_link");
    this->payload_model2  =  this->world->ModelByName("payload");
    this->payload_model2->SetStatic(false);
    this->payload_link2 = this->payload_model2->GetLink("payload::payload_link2_box");
    this->joint_2 = this->world.get()->Physics()->CreateJoint("fixed", this->iris_model2);
    this->joint_2->Load(this->payload_link2, this->iris_base_link2,  ignition::math::Pose3d(0,0,0,0,0,0));
    this->joint_2->Attach(this->payload_link2,this->iris_base_link2);
    this->joint_2->SetAxis(0, joint_axis2);
    this->joint_2->SetAxis(1, joint_axis);
    this->joint_2->SetAxis(2, joint_axis3);
    this->joint_2->SetName("payload_drone_joint2");
    ROS_INFO("add joint2");

    this->iris_model3=  this->world->ModelByName("firefly3");
    this->iris_base_link3 = this->iris_model3->GetLink("firefly3::firefly3/base_link");
    this->payload_model3  =  this->world->ModelByName("payload");
    this->payload_model3->SetStatic(false);
    this->payload_link3 = this->payload_model3->GetLink("payload::payload_link3_box");
    this->joint_3 = this->world.get()->Physics()->CreateJoint("fixed", this->iris_model3);
    this->joint_3->Load(this->payload_link3, this->iris_base_link3,  ignition::math::Pose3d(0,0,0,0,0,0));
    this->joint_3->Attach(this->payload_link3,this->iris_base_link3);
    this->joint_3->SetAxis(0, joint_axis2);
    this->joint_3->SetAxis(1, joint_axis);
    this->joint_3->SetAxis(2, joint_axis3);
    this->joint_3->SetName("payload_drone_joint3");
    ROS_INFO("add joint3");


    this->iris_model4=  this->world->ModelByName("firefly4");
    this->iris_base_link4 = this->iris_model4->GetLink("firefly4::firefly4/base_link");
    this->payload_model4  =  this->world->ModelByName("payload");
    this->payload_model4->SetStatic(false);
    this->payload_link4 = this->payload_model4->GetLink("payload::payload_link4_box");
    this->joint_4 = this->world.get()->Physics()->CreateJoint("fixed", this->iris_model4);
    this->joint_4->Load(this->payload_link4, this->iris_base_link4,  ignition::math::Pose3d(0,0,0,0,0,0));
    this->joint_4->Attach(this->payload_link4,this->iris_base_link4);
    this->joint_4->SetAxis(0, joint_axis2);
    this->joint_4->SetAxis(1, joint_axis);
    this->joint_4->SetAxis(2, joint_axis3);
    this->joint_4->SetName("payload_drone_joint4");
    ROS_INFO("add joint4");


    this->sensor_model =  this->world->ModelByName("sensor_pack");
    this->sensor_base_link = this->sensor_model->GetLink("sensor_pack::payload_sensor_imu");
    this->payload_model5 =  this->world->ModelByName("payload");
    this->payload_model5->SetStatic(false);
    this->payload_link5 = this->payload_model5->GetLink("payload::payload_rec");
    this->joint_5 = this->world.get()->Physics()->CreateJoint("fixed", this->sensor_model);
    this->joint_5->Load(this->sensor_base_link,this->payload_link5,  ignition::math::Pose3d(0,0,0,0,0,0));
    this->joint_5->Attach(this->sensor_base_link,this->payload_link5);
    this->joint_5->SetAxis(0, joint_axis2);
    this->joint_5->SetAxis(1, joint_axis);
    this->joint_5->SetAxis(2, joint_axis3);
    this->joint_5->SetName("payload_sensor_joint");
    ROS_INFO("sensor attach");

    this->item_model =  this->world->ModelByName("payload_item");
    this->item_base_link = this->item_model->GetLink("payload_item::payload_item_box");
    this->payload_model6 =  this->world->ModelByName("payload");
    this->payload_model6->SetStatic(false);
    this->payload_link6 = this->payload_model6->GetLink("payload::payload_rec");
    this->joint_6 = this->world.get()->Physics()->CreateJoint("fixed", this->item_model);
    this->joint_6->Load(this->item_base_link,this->payload_link6,  ignition::math::Pose3d(0,0,0,0,0,0));
    this->joint_6->Attach(this->item_base_link,this->payload_link6);
    this->joint_6->SetAxis(0, joint_axis2);
    this->joint_6->SetAxis(1, joint_axis);
    this->joint_6->SetAxis(2, joint_axis3);
    this->joint_6->SetName("payload_item_joint");
    ROS_INFO("payload_item attach");
    ROS_INFO("attach done");

     add_inv_2 = false;
    ros::spinOnce();

}

  }
    private: physics::WorldPtr world;

    private: event::ConnectionPtr updateConnection;
    common::Time updateRate;
    common::Time prevUpdateTime;
    //For double drones
    private: physics::JointPtr joint_, joint_2, joint_3, joint_4, joint_5, joint_6;
    private: physics::ModelPtr iris_model, iris_model2, iris_model3, iris_model4,sensor_model,item_model;
    private: physics::ModelPtr payload_model, payload_model2, payload_model3, payload_model4,payload_model5,payload_model6;
    private: physics::LinkPtr iris_base_link, iris_base_link2,iris_base_link3,iris_base_link4, sensor_base_link, item_base_link;
    private: physics::LinkPtr payload_link, payload_link2, payload_link3, payload_link4, payload_link5,payload_link6, payload, payload_box2, payload_g, payload_g1;
    bool add_inv = true;
    bool add_inv_2 = true;
    // //For single drones
    // private: physics::JointPtr joint_3, joint_4, joint_5;
    // private: physics::ModelPtr iris_model3;

    private: ros::NodeHandle rosNode;
    private: ros::Subscriber rosSub;
    private: ros::Publisher rosPub;


};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
