#include <cross_covariance/cross_covariance.h>
#include <limits.h>

bool OBU = false, lilee = false, wistron = false;
autoware_msgs::DetectedObjectArray OBU_objects, lilee_objects, wistron_objects;
double DISTANCE_THRESHOLD_;
float BETA_;
string OBU_topic_name_;
string RSU_topic_name_1_;
string RSU_topic_name_2_;
string pub_topic_name_;


void OBU_callback(const autoware_msgs::DetectedObjectArray &input)
{
    OBU = true;
    OBU_objects = input;
    cout << "Receive OBU tracking list, size : " << OBU_objects.objects.size() << endl;
}

void lileeRSU_callback(const autoware_msgs::DetectedObjectArray &input)
{
    lilee = true;
    lilee_objects = input;
    cout << "Receive lilee RSU tracking list, size : " << lilee_objects.objects.size() << endl;
}

void wistronRSU_callback(const autoware_msgs::DetectedObjectArray &input)
{
    wistron = true;
    wistron_objects = input;
    cout << "Receive wistron OBU tracking list, size : " << wistron_objects.objects.size() << endl;
}

void data_association(autoware_msgs::DetectedObjectArray &result, autoware_msgs::DetectedObjectArray &data)
{
    if (result.objects.size() == 0) {
        result.objects.insert(result.objects.begin(), data.objects.begin(), data.objects.end());
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
            if (min > distance && distance < DISTANCE_THRESHOLD_) {
                close = true;
                min_index = j;
                min = distance;
            }
        }
        if (close) {
            cross_covariance fusion(result.objects[min_index], data.objects[i], BETA_);
            fusion.cross_cov();
            result.objects[min_index].pose.position.x = fusion.c_state(0);
            result.objects[min_index].pose.position.y = fusion.c_state(1);
            result.objects[min_index].velocity.linear.x = fusion.c_state(2);
            result.objects[min_index].acceleration.linear.y = fusion.c_state(4);
            tf::Quaternion q = tf::createQuaternionFromYaw(fusion.c_state(3));
            if (!std::isnan(q[0]))
                result.objects[min_index].pose.orientation.x = q[0];
            if (!std::isnan(q[1]))
                result.objects[min_index].pose.orientation.y = q[1];
            if (!std::isnan(q[2]))
                result.objects[min_index].pose.orientation.z = q[2];
            if (!std::isnan(q[3]))
                result.objects[min_index].pose.orientation.w = q[3];
        } else {
            result.objects.push_back(data.objects[i]);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cross_covariance");

    ros::NodeHandle private_nh_("~");
    private_nh_.param<double>("DISTANCE_THRESHOLD", DISTANCE_THRESHOLD_, 3);
    private_nh_.param<float>("BETA", BETA_, 0.4);
    private_nh_.param<string>("OBU_topic_name", OBU_topic_name_, "/");
    private_nh_.param<string>("RSU_topic_name_1", RSU_topic_name_1_, "/");
    private_nh_.param<string>("RSU_topic_name_2", RSU_topic_name_2_, "/");
    private_nh_.param<string>("pub_topic_name", pub_topic_name_, "/");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<autoware_msgs::DetectedObjectArray>(pub_topic_name_, 1);
    ros::Subscriber sub1 = n.subscribe(OBU_topic_name_, 1, OBU_callback);
    ros::Subscriber sub2 = n.subscribe(RSU_topic_name_1_, 1, lileeRSU_callback);
    ros::Subscriber sub3 = n.subscribe(RSU_topic_name_2_, 1, wistronRSU_callback);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        if (OBU) {
            autoware_msgs::DetectedObjectArray result = OBU_objects;
            if (lilee)
                data_association(result, lilee_objects);
            if (wistron)
                data_association(result, wistron_objects);
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
