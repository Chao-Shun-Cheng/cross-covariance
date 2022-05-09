#include <cross_covariance/cross_covariance.h>
#include <limits.h>

bool OBU = false, RSU = false, ground_truth = false, output_result_ = false;
ofstream logfile;
string OBU_topic_name_, RSU_topic_name_, pub_topic_name_, groundTruth_topic_name_, logfile_name_, save_path_;
autoware_msgs::DetectedObjectArray OBU_objects, RSU_objects, ground_truth_objects;
double weight_d_, weight_v_, weight_h_, weight_l_;
double DISTANCE_THRESHOLD_;
float BETA_;
tf::StampedTransform rsu2map, obu2map, groundTruth2map;

void data_association(autoware_msgs::DetectedObjectArray &globalTrackingList, autoware_msgs::DetectedObjectArray &RSU);

void OBU_callback(const autoware_msgs::DetectedObjectArray &input)
{
    if (input.objects.size() == 0)
        return;

    bool success = updateNecessaryTransform(input.header.frame_id, obu2map);
    if (!success) {
        ROS_INFO("Could not find coordiante transformation from OBU to map");
        return;
    }
    transformPoseToGlobal(input, OBU_objects, obu2map);
    OBU = true;
}

void RSU_callback(const autoware_msgs::DetectedObjectArray &input)
{
    if (input.objects.size() == 0)
        return;

    bool success = updateNecessaryTransform(input.header.frame_id, rsu2map);
    if (!success) {
        ROS_INFO("Could not find coordiante transformation from RSU to map");
        return;
    }
    transformPoseToGlobal(input, RSU_objects, rsu2map);
    RSU = true;
}

void groundTruth_callback(const autoware_msgs::DetectedObjectArray &input)
{
    if (input.objects.size() == 0)
        return;

    bool success = updateNecessaryTransform(input.header.frame_id, groundTruth2map);
    if (!success) {
        ROS_INFO("Could not find coordiante transformation from groundTruth2map to map");
        return;
    }
    transformPoseToGlobal(input, ground_truth_objects, groundTruth2map);
    ground_truth = true;
}

void show_status()
{
    static int frame = 1;
    cout <<  "----------- Frame " << frame << " -----------" << endl;
    if (OBU)
        cout << GREEN << "Receive OBU" << RESET << endl;
    else
        cout << RED << "Without OBU" << RESET << endl;
    if (RSU)
        cout << GREEN << "Receive RSU" << RESET << endl;
    else
        cout << RED << "Without RSU" << RESET << endl;
    if (output_result_) {
        if (ground_truth)
            cout << GREEN << "Receive ground_truth" << RESET << endl;
        else
            cout << RED << "Without ground_truth" << RESET << endl;
    }
    frame++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cross_covariance");

    ros::NodeHandle private_nh_("~");
    private_nh_.param<double>("DISTANCE_THRESHOLD", DISTANCE_THRESHOLD_, 3);
    private_nh_.param<float>("BETA", BETA_, 0.4);
    private_nh_.param<string>("OBU_topic_name", OBU_topic_name_, "/");
    private_nh_.param<string>("RSU_topic_name", RSU_topic_name_, "/");
    private_nh_.param<string>("pub_topic_name", pub_topic_name_, "/");

    private_nh_.param<bool>("output_result", output_result_, false);
    if (output_result_) {
        private_nh_.param<string>("groundTruth_topic_name", groundTruth_topic_name_, "/");
        private_nh_.param<string>("logfile_name", logfile_name_, "/");
        private_nh_.param<string>("save_path", save_path_, "/");
        get_logfilename(logfile, save_path_, logfile_name_);
    }

    private_nh_.param<double>("weight_d", weight_d_, 1);
    private_nh_.param<double>("weight_v", weight_v_, 1);
    private_nh_.param<double>("weight_h", weight_h_, 1);
    private_nh_.param<double>("weight_l", weight_l_, 1);

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<autoware_msgs::DetectedObjectArray>(pub_topic_name_, 1);
    ros::Subscriber sub1 = n.subscribe(OBU_topic_name_, 1, OBU_callback);
    ros::Subscriber sub2 = n.subscribe(RSU_topic_name_, 1, RSU_callback);
    ros::Subscriber sub3 = n.subscribe(groundTruth_topic_name_, 2, groundTruth_callback);
    
    ros::Rate loop_rate(15);

    while (ros::ok()) {
        ros::spinOnce();
        show_status();
        if (!OBU || !RSU) {
            loop_rate.sleep();
            continue;
        }
        autoware_msgs::DetectedObjectArray globalTrackingList = OBU_objects;
        if (RSU && RSU_objects.objects.size() != 0) {
            cout << GREEN << "Start t2t fusion " << RESET << endl;
            data_association(globalTrackingList, RSU_objects);

        }
            
        pub.publish(globalTrackingList);
        if (output_result_ && ground_truth_objects.objects.size() != 0) {
            cout << YELLOW << "Save LOG !!!!" << RESET << endl;
            saveResult(logfile, save_path_, logfile_name_, ground_truth_objects, OBU_objects, RSU_objects, globalTrackingList);
            ground_truth = false;
            ground_truth_objects.objects.clear();
        }

        OBU = RSU = false;
        OBU_objects.objects.clear();
        RSU_objects.objects.clear();

        loop_rate.sleep();
    }
    return 0;
}

void data_association(autoware_msgs::DetectedObjectArray &globalTrackingList, autoware_msgs::DetectedObjectArray &RSU)
{
    if (globalTrackingList.objects.size() == 0) {
        globalTrackingList = RSU;
        return;
    }

    for (int i = 0; i < globalTrackingList.objects.size(); i++) {
        double min_cost = DBL_MAX;
        int min_index = -1;

        for (int j = 0; j < RSU.objects.size(); j++) {
            double distance = distance2points(globalTrackingList.objects[i], RSU.objects[j]);
            if (distance > DISTANCE_THRESHOLD_)
                continue;

            double velocity = abs(globalTrackingList.objects[i].velocity.linear.x - RSU.objects[j].velocity.linear.x);
            double heading = abs(tf::getYaw(globalTrackingList.objects[i].pose.orientation) - tf::getYaw(RSU.objects[j].pose.orientation));
            double label = globalTrackingList.objects[i].label == RSU.objects[j].label ? 0 : 1;
            double cost = distance * weight_d_ + velocity * weight_v_ + heading * weight_h_ + label * weight_l_;

            if (cost < min_cost) {
                min_cost = cost;
                min_index = j;
            }
        }

        if (min_index != -1) {
            cross_covariance fusion;
            fusion.init(globalTrackingList.objects[i], RSU.objects[min_index], BETA_);
            fusion.cross_cov();
            globalTrackingList.objects[i].pose.position.x = fusion.c_state(0);
            globalTrackingList.objects[i].pose.position.y = fusion.c_state(1);
            globalTrackingList.objects[i].velocity.linear.x = fusion.c_state(2);
            globalTrackingList.objects[i].acceleration.linear.y = fusion.c_state(4);
            tf::Quaternion q = tf::createQuaternionFromYaw(fusion.c_state(3));
            if (!std::isnan(q[0]))
                globalTrackingList.objects[i].pose.orientation.x = q[0];
            if (!std::isnan(q[1]))
                globalTrackingList.objects[i].pose.orientation.y = q[1];
            if (!std::isnan(q[2]))
                globalTrackingList.objects[i].pose.orientation.z = q[2];
            if (!std::isnan(q[3]))
                globalTrackingList.objects[i].pose.orientation.w = q[3];

            for (int j = 0; j < 5; j++) {
                for (int k = 0; k < 5; k++)
                    globalTrackingList.objects[i].covariance[j * 5 + k] = fusion.c_cov(j, k);
            }

            globalTrackingList.objects[i].dimensions.x =
                globalTrackingList.objects[i].dimensions.x * BETA_ + RSU.objects[min_index].dimensions.x * (1 - BETA_);
            globalTrackingList.objects[i].dimensions.y =
                globalTrackingList.objects[i].dimensions.y * BETA_ + RSU.objects[min_index].dimensions.y * (1 - BETA_);
            globalTrackingList.objects[i].dimensions.z =
                globalTrackingList.objects[i].dimensions.z * BETA_ + RSU.objects[min_index].dimensions.z * (1 - BETA_);

            RSU.objects.erase(RSU.objects.begin() + min_index);
        }
    }
    globalTrackingList.objects.insert(globalTrackingList.objects.end(), RSU.objects.begin(), RSU.objects.end());
}