#include <iostream>
#include "eigen3/Eigen/Dense"
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <math.h>


//#include ""

//X=Red, Y=Green, Z=Blue
using Eigen::MatrixXd;
class compute_jacobian
{
private:
  MatrixXd jacob;
  MatrixXd jacob_inv;
  MatrixXd factor;
  std::vector<double> joint_states;
  ros::Subscriber joint_val,end_eff_vel;
  ros::Publisher vel_joint1,vel_joint2,vel_joint3,vel_joint4,vel_joint5,vel_joint6;
  ros::Duration timestep;
  ros::Time begin;
  double q1,q2,q3,q4,q5,q6;
    std::vector<double> vel_limits;
public:
  compute_jacobian(ros::NodeHandle);
  ~compute_jacobian()
  {
  }
  void simplify_jacobian();
  void pseudo_inv(MatrixXd);
  void jointCB(const sensor_msgs::JointStateConstPtr &msg);
  void endvelCB(const geometry_msgs::TwistConstPtr &msg);
  void go_home();
};

compute_jacobian::compute_jacobian(ros::NodeHandle nh)
{
    jacob.resize(6,6);
    jacob_inv.resize(6,6);
    factor.resize(6,1);
    ros::param::get("/factor/joint_1",factor(0,0));
    ros::param::get("/factor/joint_2",factor(1,0));
    ros::param::get("/factor/joint_3",factor(2,0));
    ros::param::get("/factor/joint_4",factor(3,0));
    ros::param::get("/factor/joint_5",factor(4,0));
    ros::param::get("/factor/joint_6",factor(5,0));
    joint_states.clear();
    joint_val = nh.subscribe("/irb120/joint_states",1000,&compute_jacobian::jointCB,this);
    end_eff_vel = nh.subscribe("/velocities",1000,&compute_jacobian::endvelCB,this);
    // end_eff_vel = nh.subscribe("/irb120/end_effector_velocity/command",1,&compute_jacobian::endvelCB,this);
    vel_joint1 = nh.advertise<std_msgs::Float64>("/irb120/joint_1_position_controller/command",1000);
    vel_joint2 = nh.advertise<std_msgs::Float64>("/irb120/joint_2_position_controller/command",1000);
    vel_joint3 = nh.advertise<std_msgs::Float64>("/irb120/joint_3_position_controller/command",1000);
    vel_joint4 = nh.advertise<std_msgs::Float64>("/irb120/joint_4_position_controller/command",1000);
    vel_joint5 = nh.advertise<std_msgs::Float64>("/irb120/joint_5_position_controller/command",1000);
    vel_joint6 = nh.advertise<std_msgs::Float64>("/irb120/joint_6_position_controller/command",1000);
//    simplify_jacobian();
    begin = ros::Time::now();
    timestep = ros::Time::now() - begin;
    vel_limits.clear();
    vel_limits.push_back(4.36332);vel_limits.push_back(4.36332);vel_limits.push_back(4.36332);vel_limits.push_back(5.58505);vel_limits.push_back(5.58505);vel_limits.push_back(7.33038);
    ROS_INFO("in home");
    go_home();
}

void compute_jacobian::endvelCB(const geometry_msgs::TwistConstPtr &msg)
{
  MatrixXd end_vel(6,1);
  end_vel(0,0) = msg->linear.x;
  end_vel(1,0) = msg->linear.y;
  end_vel(2,0) = msg->linear.z;
  end_vel(3,0) = msg->angular.x;
  end_vel(4,0) = msg->angular.y;
  end_vel(5,0) = msg->angular.z;
//  ROS_INFO("end effect received");
  pseudo_inv(end_vel);
}
void compute_jacobian::go_home()
{
    ROS_INFO("in hime");
    std_msgs::Float64 msg1,msg2,msg3,msg4,msg5,msg6;

    msg1.data = 0.5;
    msg2.data = 0.1;
    msg3.data = 0.1;
    msg4.data = 0.1;
    msg5.data = 0.1;
    msg6.data = 0.1;

  for(int i=0;i<1000;i++)
  {
     ROS_INFO("pubs %d",i);
    vel_joint1.publish(msg1);
    vel_joint2.publish(msg2);
    vel_joint3.publish(msg3);
    vel_joint4.publish(msg4);
    vel_joint5.publish(msg5);
    vel_joint6.publish(msg6);
  }
  ROS_INFO("published");
//  ROS_INFO("ERROR 1 : %f",msg1.data-joint_states[0]);
//  ROS_INFO("ERROR 2 : %f",msg2.data-joint_states[1]);
//  ROS_INFO("ERROR 3 : %f",msg3.data-joint_states[2]);
//  ROS_INFO("ERROR 4 : %f",msg4.data-joint_states[3]);
//  ROS_INFO("ERROR 5 : %f",msg5.data-joint_states[4]);
//  ROS_INFO("ERROR 6 : %f",msg6.data-joint_states[5]);

}

void compute_jacobian::jointCB(const sensor_msgs::JointStateConstPtr &msg)
{
    // ROS_INFO("receiving joint data");
    joint_states.clear();
    for(int i=0;i<msg->position.size();i++)
        joint_states.push_back(msg->position[i]);
    /*std::cout<<" "<<joint_states[0];
    std::cout<<" "<<joint_states[1];
    std::cout<<" "<<joint_states[2];
    std::cout<<" "<<joint_states[3];
    std::cout<<" "<<joint_states[4];
    std::cout<<" "<<joint_states[5];*/
    //static int i=0;
    /*MatrixXd end_vel(6,1);
    end_vel(0,0) = -14.0318;
    end_vel(1,0) = 288.78;
    end_vel(2,0) = 0;
    end_vel(3,0) = -0.0351;
    end_vel(4,0) = 0.9993;
    end_vel(5,0) = 0;

    if (i>500)
        end_vel(1,0) = -1;
    if(i>1000)
        end_vel(1,0) = 0;
    ROS_INFO("i= %d",i);
    pseudo_inv(end_vel);
    i++;*/
  //  simplify_jacobian();
}

void compute_jacobian::pseudo_inv(MatrixXd end_vel)
{

  double tolerance = 0.001;
  static int  ii= 0;
  if (ii==0)
  {
      begin = ros::Time::now();
  }
  // ROS_INFO("Entering simplification");
  simplify_jacobian();
  // ROS_INFO("simplified jacobian");
  // std::cout << jacob << std::endl;
  // ROS_INFO("simplified jacobian____1");
  Eigen::JacobiSVD<MatrixXd> svd(jacob, Eigen::ComputeThinU | Eigen::ComputeThinV);
//  MatrixXd Cp = svd.matrixU() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
//  MatrixXd diff = Cp - jacob;
//  MatrixXd D = svd.singularValues().asDiagonal();
  MatrixXd D1 = svd.singularValues();
  for(int i=0;i<D1.size();++i)
  {
    if(D1(i) < tolerance)
      D1(i) = 0;
    else
      D1(i) = 1/D1(i);
  }
  MatrixXd D2=D1.asDiagonal();
  // std::cout << "U" << svd.matrixU()<<'\n';
  // std::cout << "S" << D1<<'\n';
  // std::cout << "inv S" << D2<<'\n';
  // std::cout << "V" << svd.matrixV()<<'\n';

  MatrixXd inv =svd.matrixV() * D2 * svd.matrixU().transpose();
  for (int i = 0; i <6; i++)
       for(int j=0;j<6;j++)
           inv(i,j) = floorf(inv(i,j)*10000)/10000;
  std::cout<<"inv jacobian\n"<<inv<<std::endl;
  end_vel(0,0) = end_vel(0,0);//10*factor(0,0)*end_vel(0,0);
  end_vel(1,0) = end_vel(1,0);//10*factor(1,0)*end_vel(1,0);
  end_vel(2,0) = end_vel(2,0);//10*factor(2,0)*end_vel(2,0);
  end_vel(3,0) = end_vel(3,0);//10*factor(3,0)*end_vel(3,0);
  end_vel(4,0) = end_vel(4,0);//10*factor(4,0)*end_vel(4,0);
  end_vel(5,0) = end_vel(5,0);//10*factor(5,0)*end_vel(5,0);
  std::cout<<"end vel\n"<<end_vel<<std::endl;
  MatrixXd joint_vel = inv * end_vel;

  std_msgs::Float64 msg1,msg2,msg3,msg4,msg5,msg6;

  timestep = ros::Time::now() - begin;

  if (ii<=1000)
  {
      for(int i=0;i<6;i++)
      {
    //      ROS_WARN("Joint %d velocity: %f",(i+1),joint_vel(i,0));
          if( -vel_limits[i]>joint_vel(i,0)||joint_vel(i,0)>vel_limits[i])
          {
              ROS_WARN("Joint %d velocity limit reached",i);
              joint_vel(i,0) = 0.0;
          }
      }
      std::cout << "Joint data" << joint_vel<<'\n';
      ii++;

  msg1.data = q1+joint_vel(0,0)*(0.001);
  msg2.data = q2+joint_vel(1,0)*((0.001));
  msg3.data = q3+joint_vel(2,0)*((0.001));
  msg4.data = q4+joint_vel(3,0)*((0.001));
  msg5.data = q5+joint_vel(4,0)*((0.001));
  msg6.data = q6+joint_vel(5,0)*((0.001));

  msg1.data = 0.5;
  msg2.data = 0.5;
  msg3.data = 0.5;
  msg4.data = 0.5;
  msg5.data = 0.5;
  msg6.data = 0.5;


  vel_joint1.publish(msg1);
  vel_joint2.publish(msg2);
  vel_joint3.publish(msg3);
  vel_joint4.publish(msg4);
  vel_joint5.publish(msg5);
  vel_joint6.publish(msg6);


  joint_states[0] = msg1.data;
  joint_states[1] = msg2.data;
  joint_states[2] = msg3.data;
  joint_states[3] = msg4.data;
  joint_states[4] = msg5.data;
  joint_states[5] = msg6.data;



//  begin = ros::Time::now();

  ROS_INFO("Publishing joint pos");
  ROS_INFO("1:%f",msg1.data);
  ROS_INFO("2:%f",msg2.data);
  ROS_INFO("3:%f",msg3.data);
  ROS_INFO("4:%f",msg4.data);
  ROS_INFO("5:%f",msg5.data);
  ROS_INFO("6:%f",msg6.data);

  }
//  std::cout << "diff:\n" << diff.array().abs().sum() << "\n";
//  std::cout << "inv:\n" << Cp*end_vel << "\n";
}

void compute_jacobian::simplify_jacobian()
{
  q1 = joint_states[0];
  q2 = joint_states[1];
  q3 = joint_states[2];
  q4 = joint_states[3];
  q5 = joint_states[4];
  q6 = joint_states[5];
 /*double pi = 3.1415;
//  std::cout<<"\nq1 "<<q1;
  double q1 = (pi/180)*60;
  double q2 = (pi/180)*50;
  double q3 = (pi/180)*40;
  double q4 = (pi/180)*30;
  double q5 = (pi/180)*20;
  double q6 = (pi/180)*10;*/
//   std::cout<<"\nq1 "<<q1;
  // std::cout<<" q2 "<<q2;
  // std::cout<<" q3 "<<q3;
  // std::cout<<" q4 "<<q4;
  // std::cout<<" q5 "<<q5;
  // std::cout<<" q6 "<<q6<<std::endl;

  jacob(0,0) = -270*sin(q1)*sin(q2) + 72*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 70*sin(q1)*sin(q2 + q3) - 72*sin(q1)*cos(q5)*cos(q2 + q3) - 302*sin(q1)*cos(q2 + q3) - 72*sin(q4)*sin(q5)*cos(q1);
  jacob(0,1) = 2*(-36*sin(q5)*cos(q4)*cos(q2 + q3) - 36*sin(q2 + q3)*cos(q5) - 151*sin(q2 + q3) + 135*cos(q2) + 35*cos(q2 + q3))*cos(q1);
  jacob(0,2) =  2*(-36*sin(q5)*cos(q4)*cos(q2 + q3) - 36*sin(q2 + q3)*cos(q5) - 151*sin(q2 + q3) + 35*cos(q2 + q3))*cos(q1);
  jacob(0,3) = 72*(-sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q5);
  jacob(0,4) = -72*sin(q1)*sin(q4)*cos(q5) - 72*sin(q5)*cos(q1)*cos(q2 + q3) - 72*sin(q2 + q3)*cos(q1)*cos(q4)*cos(q5);
  jacob(0,5) = 0;

  jacob(1,0) = -72*sin(q1)*sin(q4)*sin(q5) + 270*sin(q2)*cos(q1) - 72*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) + 70*sin(q2 + q3)*cos(q1) + 72*cos(q1)*cos(q5)*cos(q2 + q3) + 302*cos(q1)*cos(q2 + q3);
  jacob(1,1) =  2*(-36*sin(q5)*cos(q4)*cos(q2 + q3) - 36*sin(q2 + q3)*cos(q5) - 151*sin(q2 + q3) + 135*cos(q2) + 35*cos(q2 + q3))*sin(q1);
  jacob(1,2) =  2*(-36*sin(q5)*cos(q4)*cos(q2 + q3) - 36*sin(q2 + q3)*cos(q5) - 151*sin(q2 + q3) + 35*cos(q2 + q3))*sin(q1);
  jacob(1,3) =   72*(sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q5);
  jacob(1,4) = -72*sin(q1)*sin(q5)*cos(q2 + q3) - 72*sin(q1)*sin(q2 + q3)*cos(q4)*cos(q5) + 72*sin(q4)*cos(q1)*cos(q5);
  jacob(1,5) = 0;

  jacob(2,0) = 0;
  jacob(2,1) = -270*sin(q2) + 72*sin(q5)*sin(q2 + q3)*cos(q4) - 70*sin(q2 + q3) - 72*cos(q5)*cos(q2 + q3) - 302*cos(q2 + q3);
  jacob(2,2) =  72*sin(q5)*sin(q2 + q3)*cos(q4) - 70*sin(q2 + q3) - 72*cos(q5)*cos(q2 + q3) - 302*cos(q2 + q3);
  jacob(2,3) =  72*sin(q4)*sin(q5)*cos(q2 + q3);
  jacob(2,4) =    72*sin(q5)*sin(q2 + q3) - 72*cos(q4)*cos(q5)*cos(q2 + q3);
  jacob(2,5) = 0;

  jacob(3,0) =  -sin(q1);
  jacob(3,1) =  -sin(q1);
  jacob(3,2) =   cos(q1)*cos(q2 + q3);
  jacob(3,3) =  -sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1);
  jacob(3,4) =  -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3);
  jacob(3,5) =  -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3);

  jacob(4,0) =  cos(q1);
  jacob(4,1) =  cos(q1);
  jacob(4,2) =  sin(q1)*cos(q2 + q3);
  jacob(4,3) =  sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4);
  jacob(4,4) = -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3);
  jacob(4,5) = -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3);

  jacob(5,0) =  0;
  jacob(5,1) =  0;
  jacob(5,2) = -sin(q2 + q3);
  jacob(5,3) =  sin(q4)*cos(q2 + q3);
  jacob(5,4) = -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5);
  jacob(5,5) = -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5);

/*
  jacob(0,0) =-80*sin(q1) - 40*sin(q1 + q2) - 20*sin(q1 + q2 + q3);
  jacob(1,0) = 80*cos(q1) + 40*cos(q1 + q2) + 20*cos(q1 + q2 + q3);
  jacob(2,0) = 0;
  jacob(3,0) = 0;
  jacob(4,0) = 0;
  jacob(5,0) = 1;

  jacob(0,1) = -40*sin(q1 + q2) - 20*sin(q1 + q2 + q3);
  jacob(1,1) =  40*cos(q1 + q2) + 20*cos(q1 + q2 + q3);
  jacob(2,1) =  0;
  jacob(3,1) =  0;
  jacob(4,1) =  0;
  jacob(5,1) =  1;

  jacob(0,2) = -20*sin(q1 + q2 + q3);
  jacob(1,2) =  20*cos(q1 + q2 + q3);
  jacob(2,2) =  0;
  jacob(3,2) =  0;
  jacob(4,2) =  0;
  jacob(5,2) =  1;
*/
 for (int i = 0; i <6; i++)
      for(int j=0;j<6;j++)
          jacob(i,j) = floorf(jacob(i,j)*10000)/10000;
   std::cout<<"Jacobian\n"<< jacob << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"robot_interface");
  ros::NodeHandle nh;
  compute_jacobian obj(nh);
  // ros::Publisher vel_joint1;
  // vel_joint1 = nh.advertise<std_msgs::Float64>("/irb120/joint_1_velocity_controller/command",1);
  // std_msgs::Float64 cmd;
  // cmd.data = 0.1;

  ros::Rate freq(1000); //100hz
  while(ros::ok())
  {
      ros::spinOnce();
      freq.sleep();
  }
  return 0;
}
