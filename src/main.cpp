#include <cross_covariance/cross_covariance.h>
#include <limits.h>

enum RSUName : int { LILEE = 0, WISTRON = 1 };

bool OBU = false, lilee = false, wistron = false;
autoware_msgs::DetectedObjectArray OBU_objects, lilee_objects, wistron_objects;

void OBU_callback(const autoware_msgs::DetectedObjectArray &input) {
  OBU = true;
  OBU_objects = input;
  cout << "Receive OBU tracking list, size : " << OBU_objects.objects.size()
       << endl;
}

void lileeRSU_callback(const autoware_msgs::DetectedObjectArray &input) {
  lilee = true;
  lilee_objects = input;
  cout << "Receive lilee RSU tracking list, size : "
       << lilee_objects.objects.size() << endl;
}

void wistronRSU_callback(const autoware_msgs::DetectedObjectArray &input) {
  wistron = true;
  wistron_objects = input;
  cout << "Receive wistron OBU tracking list, size : "
       << wistron_objects.objects.size() << endl;
}

void data_association(autoware_msgs::DetectedObjectArray &result,
                      autoware_msgs::DetectedObjectArray &data) {
  if (result.objects.size() == 0) {
    result.objects.insert(result.objects.begin(), data.objects.begin(),
                          data.objects.end());
    return;
  }
  for (int i = 0; i < data.objects.size(); i++) {
    bool close = false;
    int min_index = -1;
    double min = DBL_MAX;
    float x = data.objects[i].pose.position.x;
    float y = data.objects[i].pose.position.y;
    for (int j = 0; j < result.objects.size(); j++) {
      double distance = pow((x - result.objects[i].pose.position.x), 2) + pow((y - result.objects[i].pose.position.y), 2);
      if(min > distance && distance < 1){
          close = true;
          min_index = j;
          min = distance;
      }
    }
    if(close){
        // TODO cross covariance !!
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cross_covariance");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<autoware_msgs::DetectedObjectArray>(
      "t2t_fusion/cross_cov/objects", 1);
  ros::Subscriber sub1 = n.subscribe(
      "/detection/lidar_detector/objects_filtered", 1, OBU_callback);
  ros::Subscriber sub2 =
      n.subscribe("/simulator/ground_truth/objects", 1, lileeRSU_callback);
  ros::Subscriber sub3 =
      n.subscribe("/detection/fusion_tools/objects", 1, wistronRSU_callback);
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    if (OBU) {
      autoware_msgs::DetectedObjectArray result = OBU_objects;
      if (lilee) {
      }
      if (wistron) {
      }
      pub.publish(result);
    }
    OBU = false, lilee = false, wistron = false;
    OBU_objects.objects.clear();
    lilee_objects.objects.clear();
    wistron_objects.objects.clear();
    loop_rate.sleep();
  }

  return 0;
}
