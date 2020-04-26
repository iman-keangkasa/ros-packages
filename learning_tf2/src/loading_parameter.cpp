#include <ros/ros.h>
#include <map>
#include <string>
#include <vector>

using namespace std;



void setting_param(ros::NodeHandle & nh_param)
{
  map <string,float> joints_zeros;
  vector <string> joint_names;
  unsigned iterator;
  //check if zeros exist
  if (nh_param.hasParam("zeros"))
  {
    nh_param.getParam("zeros", joints_zeros);
    
    //iterate through the dictionary 
    //this is a C++11 standard so make change to 
    //CMakeList
    //int index = 0;

    cout << "Printing Element Keys From Map"<<endl;
    for(auto & element : joints_zeros)
    {
      //the key of the dictionary
      //x.firsts
      cout << element.first<< endl;
      //cout << joint_names[index];
      //the value of the dictionary. I am setting the all to 1
      element.second = 2;

      //since the joint_names size is uncertain, we use push_back()
      joint_names.push_back(element.first);
      
      //++index;
    }
    
    cout<< "Printing Vector Values from Map Key" << endl;
    for (iterator=0; iterator < joint_names.size(); ++iterator)
    {
      cout << joint_names[iterator] << endl;
    }
    //reload a new joints_zero values
    nh_param.setParam("zeros", joints_zeros);

  }
}

int main(int argc, char **argv)
{
  ros::init(argc,argv, "loading_parameter");
  ros::NodeHandle nh;
  setting_param(nh);
  return 0;
}
