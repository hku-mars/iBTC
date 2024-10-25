#include "include/ibtc.h"

#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <unordered_set>
#include <opencv2/xfeatures2d.hpp>

//#define disable_rgb  // Uncomment to make the code become the adapted version of btc


void pub_keyframe_ids(std::vector<pcl::PointXYZI> &id_vec, const ros::Publisher &pub_handle, const Eigen::Vector3d &pos, const int id, const std::string frame_id)
{
  visualization_msgs::MarkerArray text_msg;
  visualization_msgs::Marker a_text;
  a_text.header.stamp = ros::Time::now();
  a_text.header.frame_id = frame_id;
  a_text.ns = "pose_ids";
  a_text.action = visualization_msgs::Marker::ADD;
  a_text.pose.orientation.w = 1.0;
  a_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  a_text.scale.z = 0.4;
  pcl::PointXYZI apt;
  apt.x = pos[0], apt.y = pos[1], apt.z = pos[2], apt.intensity = id;
  id_vec.push_back(apt);

  for (int i = 0; i < id_vec.size(); i++){
    a_text.id = i;
    a_text.color.g = 1.0f;
    a_text.color.r = 1.0f;
    a_text.color.a = 1.0f;
    geometry_msgs::Pose pose;
    pose.position.x = id_vec[i].x;
    pose.position.y = id_vec[i].y;
    pose.position.z = id_vec[i].z;
    a_text.pose = pose;
    a_text.text = std::to_string(int(id_vec[i].intensity));
    text_msg.markers.push_back(a_text);
  }

  pub_handle.publish(text_msg);
}

void pyrDown_multiple(const cv::Mat& img_cur, cv::Mat& img_ds, int layer_num)
{
    assert(layer_num <= 8);
    cv::Mat img_tmp, img_tmp2, img_tmp3, img_tmp4;
    cv::Mat img_tmp5,img_tmp6, img_tmp7, img_tmp8;

    if (layer_num == 0)
    {
        img_ds = img_cur;
    }
    cv::pyrDown(img_cur, img_tmp);
    if (layer_num == 1)
    {
        img_ds = img_tmp;
        return;
    }
    cv::pyrDown(img_tmp, img_tmp2);
    if (layer_num == 2)
    {
        img_ds = img_tmp2;
        return;
    }
    cv::pyrDown(img_tmp2, img_tmp3);
    if (layer_num == 3)
    {
        img_ds = img_tmp3;
        return;
    }
    cv::pyrDown(img_tmp3, img_tmp4);
    if (layer_num == 4)
    {
        img_ds = img_tmp4;
        return;
    }
    cv::pyrDown(img_tmp4, img_tmp5);
    if (layer_num == 5)
    {
        img_ds = img_tmp5;
        return;
    }
    cv::pyrDown(img_tmp5, img_tmp6);
    if (layer_num == 6)
    {
        img_ds = img_tmp6;
        return;
    }
    cv::pyrDown(img_tmp6, img_tmp7);
    if (layer_num == 7)
    {
        img_ds = img_tmp7;
        return;
    }
    cv::pyrDown(img_tmp7, img_tmp8);
    if (layer_num == 8)
    {
        img_ds = img_tmp8;
        return;
    }
}

void split_string(const std::string& s, std::vector<std::string>& v, const std::string& c)
{
     std::string::size_type pos1, pos2;
     pos2 = s.find(c);
     pos1 = 0;
     while(std::string::npos != pos2)
     {
         v.push_back(s.substr(pos1, pos2-pos1));

         pos1 = pos2 + c.size();
         pos2 = s.find(c, pos1);
     }
     if(pos1 != s.length())
         v.push_back(s.substr(pos1));
}

bool load_poses(std::map<std::string, imu_lidar_camera>& sorter, const std::string &file_name, const std::string &file_name2)
{
    std::ifstream infile;
    infile.open(file_name);
    if (!infile.is_open())
        ROS_ERROR("cam_pose_file not found!!!");
    if (!infile.is_open())
        return false;
    assert(infile.is_open());
    std::string s;
    int offset = 0;
    while(getline(infile, s))
    {
        std::vector<std::string> tmp;
        split_string(s, tmp, " ");
        std::string pose_time = tmp[offset+0];
        Eigen::Quaterniond pose_c2w_q, pose_w2c_q;
        Eigen::Vector3d pose_c2w_t, pose_w2c_t;
        if (tmp.size() < 8)
            ROS_ERROR("cam_pose_file data format not right!!");
        if (tmp.size() < 8)
            return false;
        if (tmp.size() == 8)
        {
            pose_w2c_q = Eigen::Quaterniond(atof(tmp[offset+7].c_str()), atof(tmp[offset+4].c_str()),
                atof(tmp[offset+5].c_str()), atof(tmp[offset+6].c_str()));
            pose_w2c_t = Eigen::Vector3d(atof(tmp[offset+1].c_str()), atof(tmp[offset+2].c_str()),
                atof(tmp[offset+3].c_str()));
            pose_c2w_q = pose_w2c_q.inverse();
            pose_c2w_t = - (pose_c2w_q*pose_w2c_t);
        }
        else
        {
            pose_c2w_q = Eigen::Quaterniond(atof(tmp[offset+7].c_str()), atof(tmp[offset+4].c_str()),
                atof(tmp[offset+5].c_str()), atof(tmp[offset+6].c_str()));
            pose_c2w_t = Eigen::Vector3d(atof(tmp[offset+1].c_str()), atof(tmp[offset+2].c_str()),
                atof(tmp[offset+3].c_str()));
            pose_w2c_q = Eigen::Quaterniond(atof(tmp[offset+14].c_str()), atof(tmp[offset+11].c_str()),
                atof(tmp[offset+12].c_str()), atof(tmp[offset+13].c_str()));
            pose_w2c_t = Eigen::Vector3d(atof(tmp[offset+8].c_str()), atof(tmp[offset+9].c_str()),
                atof(tmp[offset+10].c_str()));
        }

        std::string pose_t_str;
        if (pose_time.length() >= 15) pose_t_str = pose_time.erase(15);
        else  pose_t_str = pose_time;
        sorter[pose_t_str].m_pose_c_2_w_q = pose_c2w_q;
        sorter[pose_t_str].m_pose_c_2_w_t = pose_c2w_t;
        sorter[pose_t_str].m_pose_w_2_c_q = pose_w2c_q;
        sorter[pose_t_str].m_pose_w_2_c_t = pose_w2c_t;
        sorter[pose_t_str].camera_poseset = true;
    }
    infile.close();
    offset = 0;
    infile.open(file_name2);
    if (!infile.is_open())
        ROS_ERROR("imu_pose_file not found!!!");
    if (!infile.is_open())
        return false;
    assert(infile.is_open());
    while(getline(infile, s))
    {
        std::vector<std::string> tmp;
        split_string(s, tmp, " ");
        if (tmp.size() < 8)
            ROS_ERROR("imu_pose_file data format not right!!");
        if (tmp.size() < 8)
            return false;
        std::string pose_time = tmp[offset+0];

        Eigen::Quaterniond w_q_l(atof(tmp[offset+7].c_str()), atof(tmp[offset+4].c_str()),
                        atof(tmp[offset+5].c_str()), atof(tmp[offset+6].c_str()));
        Eigen::Vector3d w_t_l(atof(tmp[offset+1].c_str()), atof(tmp[offset+2].c_str()),
                        atof(tmp[offset+3].c_str()));

        sorter[pose_time.erase(15)].m_lidar_q = w_q_l;
        sorter[pose_time.erase(15)].m_lidar_t = w_t_l;
    }
    return true;
}

cv::Mat extract_image_from_msg(const sensor_msgs::CompressedImage::ConstPtr &img_ptr, const int &pre_ds_image_level)
{
    cv::Mat temp_img = cv_bridge::toCvCopy( img_ptr, sensor_msgs::image_encodings::BGR8 )->image.clone();
    cv::Mat img_tmp;
    pyrDown_multiple(temp_img, img_tmp, pre_ds_image_level);  // pre downsampling image
    return img_tmp;
}

cv::Mat set_used_region_mask(cv::Mat &img_gray, const int &IMG_HEI, const int &IMG_WID)
{
    cv::Mat mask(IMG_HEI, IMG_WID, CV_8UC1);
    int check_range = 20;
    for (int r_cen = 0; r_cen < IMG_HEI; r_cen++)
    {
        for (int c_cen = 0; c_cen < IMG_WID; c_cen++)
        {
            bool ok_flag = true;

            for(int r = r_cen - check_range ; r < r_cen + check_range; r++)
            {
                for(int c = c_cen - check_range ; c < c_cen + check_range; c++)
                {
                    if (r >= 0 && r < IMG_HEI && c >= 0 && c < IMG_WID)
                    {
                        if (img_gray.at<uchar>(r,c) < 3)
                        {
                            ok_flag = false;
                            goto break_out_tmp;
                        }
                    }
                }
            }
            break_out_tmp:
            if (ok_flag) //rectified image have undefined areas
                mask.at<uchar>(r_cen,c_cen) = 255;
            else
                mask.at<uchar>(r_cen,c_cen) = 0;
        }
    }

    return mask;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ibtc");
    ros::NodeHandle nh;
    std::string data_name = "";
    std::string setting_path = "";
    std::string bag_file = "";
    std::string imu_pose_fname = "";
    std::string cam_pose_fname = "";
    std::string gt_file = "";
    double icp_threshold = 0.5;
    // hku mb r3live config
    int IMG_WID = 1280;
    int IMG_HEI = 1024;
    float fx = 863.4241;
    float fy = 863.4171;
    float cx = 640.6808;
    float cy = 518.3392;
    int layer_num = 2;
    int pre_ds_image_level = 0;
    int set_used_region = 0;
    nh.param<std::string>("data_name", data_name, "");
    nh.param<std::string>("setting_path", setting_path, "");
    nh.param<std::string>("bag_file", bag_file, "");
    nh.param<std::string>("imu_pose_fname", imu_pose_fname, "");
    nh.param<std::string>("cam_pose_fname", cam_pose_fname, "");

    nh.getParam("IMG_WID", IMG_WID);
    nh.getParam("IMG_HEI", IMG_HEI);
    nh.getParam("fx", fx);
    nh.getParam("fy", fy);
    nh.getParam("cx", cx);
    nh.getParam("cy", cy);
    nh.getParam("pre_ds_image_level", pre_ds_image_level);
    nh.getParam("set_used_region", set_used_region);
    std::cout << "Camera Parameters: " << std::endl;
    std::cout << "- IMG_WID: " << IMG_WID << std::endl;
    std::cout << "- IMG_HEI: " << IMG_HEI << std::endl;
    std::cout << "- fx: " << fx << std::endl;
    std::cout << "- fy: " << fy << std::endl;
    std::cout << "- cx: " << cx << std::endl;
    std::cout << "- cy: " << cy << std::endl;
    std::cout << "- pre_ds_image_level: " << pre_ds_image_level << std::endl;

    float pre_ds_scalar = pow(2, pre_ds_image_level);
    IMG_WID /= pre_ds_scalar; IMG_HEI /= pre_ds_scalar; fx /= pre_ds_scalar; fy /= pre_ds_scalar; cx /= pre_ds_scalar; cy /= pre_ds_scalar;
    std::cout << "- IMG_WID: " << IMG_WID << std::endl;
    std::cout << "- IMG_HEI: " << IMG_HEI << std::endl;
    std::cout << "- fx: " << fx << std::endl;
    std::cout << "- fy: " << fy << std::endl;
    std::cout << "- cx: " << cx << std::endl;
    std::cout << "- cy: " << cy << std::endl;
    double R00, R01, R02, R10, R11, R12, R20, R21, R22, t0, t1, t2;
    nh.getParam("R00", R00);  nh.getParam("R01", R01);   nh.getParam("R02", R02);
    nh.getParam("R10", R10);  nh.getParam("R11", R11);   nh.getParam("R12", R12);
    nh.getParam("R20", R20);  nh.getParam("R21", R21);   nh.getParam("R22", R22);
    nh.getParam("t0", t0);  nh.getParam("t1", t1);   nh.getParam("t2", t2);
    Eigen::Matrix3d R_lidar_to_imu;
    R_lidar_to_imu << R00, R01, R02, R10, R11, R12, R20, R21, R22;
    Eigen::Vector3d t_lidar_to_imu(t0, t1, t2);
    std::cout << "R_lidar_to_imu: " << std::endl << R_lidar_to_imu << std::endl;
    std::cout << "t_lidar_to_imu: " << t_lidar_to_imu.transpose() << std::endl;

    layer_num = floor(double(IMG_WID+IMG_HEI)*0.5/500.0f);

    std::string save_directory;
    nh.param<std::string>("SaveDir",save_directory,"");
    int save_for_pr_eval = 1; //true;
    nh.getParam("save_for_pr_eval", save_for_pr_eval);
    int save_for_avgrecall_eval = 0; //true;
    nh.getParam("save_for_avgrecall_eval", save_for_avgrecall_eval);
    if (save_for_pr_eval)
    {
        save_for_avgrecall_eval = 0; // cannot do both eval record at the same time
        std::cout << "save_for_avgrecall_eval: " << save_for_avgrecall_eval << std::endl;
    }

    std::cout << "save_for_pr_eval: " << save_for_pr_eval << std::endl;
    float range_gtlc_search = 50;
    nh.getParam("range_gtlc_search", range_gtlc_search);
    std::cout << "range_gtlc_search: " << range_gtlc_search << std::endl;

    std::fstream debug_file = std::fstream(save_directory + "lc_debug.txt", std::fstream::out);
    std::fstream timelog_file = std::fstream(save_directory + "timelog.txt", std::fstream::out);
    std::fstream gt_doc = std::fstream(save_directory + "lc_gt_doc.txt", std::fstream::out);

    ros::Publisher pubOdomAftMapped =
        nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    ros::Publisher pubRegisterCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
    ros::Publisher pubCurrentBinary =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
    ros::Publisher pubMatchedCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
    ros::Publisher pubMatchedBinary =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
    ros::Publisher pubSTD =
        nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);
    ros::Publisher pub_raw_img = nh.advertise<sensor_msgs::Image>("/raw_in_img",1000);
    ros::Publisher pub_dep_img = nh.advertise<sensor_msgs::Image>("/depth_img",1000);

    ros::Publisher pubCorrectionId = nh.advertise<visualization_msgs::MarkerArray>("/ids_pose", 10);

    std::cout << "bag_file set to " << bag_file << std::endl;
    std::cout << "imu_pose_fname set to " << imu_pose_fname << std::endl;
    std::cout << "cam_pose_fname set to " << cam_pose_fname << std::endl;

    std::map<std::string, imu_lidar_camera> sorter;
    bool is_loaded = load_poses(sorter, cam_pose_fname, imu_pose_fname);
    if (!is_loaded)  return 0;

    ros::Rate loop(5000);

    ConfigSetting config_setting;
    load_config_setting(setting_path, config_setting);
    icp_threshold = config_setting.icp_threshold_;
    icp_threshold = 0.3;
    std::cout << "icp_threshold: " << icp_threshold << std::endl;
    std::cout << "Press any key to start ... " << std::endl;
    std::cin.get();

    auto t_00 = std::chrono::high_resolution_clock::now();
    // save all point clouds of key frame
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_list;

    // save all planes of key frame
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> history_plane_list;

    // save all binary descriptors of key frame
    std::vector<std::vector<BinaryDescriptor>> history_binary_list;

    // hash table, save all descriptor from geo and rgb
    std::unordered_map<STD_LOC, std::vector<STD>> STD_map, STD_map_brf;

    std::vector<std::vector<cv::Point2f>> corner_list;
    pcl::PointCloud<pcl::PointXYZI>::Ptr posi_cloud (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cens (new pcl::PointCloud<pcl::PointXYZI>());
    std::vector<pcl::PointXYZI> id_vec;

    // currently, this code only accept maximum 30 images in each keyframe
    int max_submap_img_num = 30;
    std::vector<cv::Mat> image_list(max_submap_img_num);
    std::vector<cv::Mat> image_gray_list(max_submap_img_num), image_gray_ds_list(max_submap_img_num);
    std::vector<double> image_times(max_submap_img_num);
    std::vector<std::vector<kpt_2d_info>> kpts_from_img_list(max_submap_img_num);
    std::vector<std::vector<std::vector<__m128i>>> brief_arrays_mm128_list(max_submap_img_num);
    std::vector< std::vector<cv::Point2f> > corners_tmp_list(max_submap_img_num);
    std::vector< std::vector<cv::Point2f> > corners_out_list(max_submap_img_num);
    std::vector<std::vector<float>> harrisR_list(max_submap_img_num);
    std::vector<cv::Mat> eigtmp_list(max_submap_img_num);
    std::vector<cv::Mat> eigmat_list, Dx_list, Dy_list, cov_list, descriptors_list, sum_list;
    //std::vector<std::vector<float>> depth_mask_list(max_submap_img_num);
    cv::Mat mask_resize;

    // Create depth mask using basic int array, so that we can use "memset()" to reset mask to -1 quickly
    std::vector<int*> depthf_mask_list(max_submap_img_num);
    int *empty_arr0 = new int[IMG_WID*IMG_HEI];  int *empty_arr1 = new int[IMG_WID*IMG_HEI];
    int *empty_arr2 = new int[IMG_WID*IMG_HEI];  int *empty_arr3 = new int[IMG_WID*IMG_HEI];
    int *empty_arr4 = new int[IMG_WID*IMG_HEI];  int *empty_arr5 = new int[IMG_WID*IMG_HEI];
    int *empty_arr6 = new int[IMG_WID*IMG_HEI];  int *empty_arr7 = new int[IMG_WID*IMG_HEI];
    int *empty_arr8 = new int[IMG_WID*IMG_HEI];  int *empty_arr9 = new int[IMG_WID*IMG_HEI];
    int *empty_arr10 = new int[IMG_WID*IMG_HEI];  int *empty_arr11 = new int[IMG_WID*IMG_HEI];
    int *empty_arr12 = new int[IMG_WID*IMG_HEI];  int *empty_arr13 = new int[IMG_WID*IMG_HEI];
    int *empty_arr14 = new int[IMG_WID*IMG_HEI];  int *empty_arr15 = new int[IMG_WID*IMG_HEI];
    int *empty_arr16 = new int[IMG_WID*IMG_HEI];  int *empty_arr17 = new int[IMG_WID*IMG_HEI];
    int *empty_arr18 = new int[IMG_WID*IMG_HEI];  int *empty_arr19 = new int[IMG_WID*IMG_HEI];
    int *empty_arr20 = new int[IMG_WID*IMG_HEI];  int *empty_arr21 = new int[IMG_WID*IMG_HEI];
    int *empty_arr22 = new int[IMG_WID*IMG_HEI];  int *empty_arr23 = new int[IMG_WID*IMG_HEI];
    int *empty_arr24 = new int[IMG_WID*IMG_HEI];  int *empty_arr25 = new int[IMG_WID*IMG_HEI];
    int *empty_arr26 = new int[IMG_WID*IMG_HEI];  int *empty_arr27 = new int[IMG_WID*IMG_HEI];
    int *empty_arr28 = new int[IMG_WID*IMG_HEI];  int *empty_arr29 = new int[IMG_WID*IMG_HEI];

    for (int si = 0; si < max_submap_img_num; si++)
    {
        //std::vector<float> empty_vec(IMG_WID*IMG_HEI, -1);
        //depth_mask_list[si] = empty_vec;
        if (si == 0)    memset(empty_arr0, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 1)    memset(empty_arr1, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 2)    memset(empty_arr2, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 3)    memset(empty_arr3, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 4)    memset(empty_arr4, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 5)    memset(empty_arr5, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 6)    memset(empty_arr6, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 7)    memset(empty_arr7, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 8)    memset(empty_arr8, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 9)    memset(empty_arr9, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 10)    memset(empty_arr10, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 11)    memset(empty_arr11, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 12)    memset(empty_arr12, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 13)    memset(empty_arr13, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 14)    memset(empty_arr14, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 15)    memset(empty_arr15, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 16)    memset(empty_arr16, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 17)    memset(empty_arr17, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 18)    memset(empty_arr18, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 19)    memset(empty_arr19, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 20)    memset(empty_arr20, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 21)    memset(empty_arr21, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 22)    memset(empty_arr22, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 23)    memset(empty_arr23, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 24)    memset(empty_arr24, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 25)    memset(empty_arr25, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 26)    memset(empty_arr26, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 27)    memset(empty_arr27, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 28)    memset(empty_arr28, -1, sizeof(int)*IMG_WID*IMG_HEI);
        if (si == 29)    memset(empty_arr29, -1, sizeof(int)*IMG_WID*IMG_HEI);

        if (si == 0)    depthf_mask_list[si] = empty_arr0;
        if (si == 1)    depthf_mask_list[si] = empty_arr1;
        if (si == 2)    depthf_mask_list[si] = empty_arr2;
        if (si == 3)    depthf_mask_list[si] = empty_arr3;
        if (si == 4)    depthf_mask_list[si] = empty_arr4;
        if (si == 5)    depthf_mask_list[si] = empty_arr5;
        if (si == 6)    depthf_mask_list[si] = empty_arr6;
        if (si == 7)    depthf_mask_list[si] = empty_arr7;
        if (si == 8)    depthf_mask_list[si] = empty_arr8;
        if (si == 9)    depthf_mask_list[si] = empty_arr9;
        if (si == 10)    depthf_mask_list[si] = empty_arr10;
        if (si == 11)    depthf_mask_list[si] = empty_arr11;
        if (si == 12)    depthf_mask_list[si] = empty_arr12;
        if (si == 13)    depthf_mask_list[si] = empty_arr13;
        if (si == 14)    depthf_mask_list[si] = empty_arr14;
        if (si == 15)    depthf_mask_list[si] = empty_arr15;
        if (si == 16)    depthf_mask_list[si] = empty_arr16;
        if (si == 17)    depthf_mask_list[si] = empty_arr17;
        if (si == 18)    depthf_mask_list[si] = empty_arr18;
        if (si == 19)    depthf_mask_list[si] = empty_arr19;
        if (si == 20)    depthf_mask_list[si] = empty_arr20;
        if (si == 21)    depthf_mask_list[si] = empty_arr21;
        if (si == 22)    depthf_mask_list[si] = empty_arr22;
        if (si == 23)    depthf_mask_list[si] = empty_arr23;
        if (si == 24)    depthf_mask_list[si] = empty_arr24;
        if (si == 25)    depthf_mask_list[si] = empty_arr25;
        if (si == 26)    depthf_mask_list[si] = empty_arr26;
        if (si == 27)    depthf_mask_list[si] = empty_arr27;
        if (si == 28)    depthf_mask_list[si] = empty_arr28;
        if (si == 29)    depthf_mask_list[si] = empty_arr29;
    }

    for (int si = 0; si < max_submap_img_num; si++)
    {
        cv::Mat descriptors(10000, 64, CV_8U);
        descriptors_list.push_back(descriptors);
    }

    std::vector<std::vector<__m128i>> all_briefs;
    all_briefs.reserve(1e6);

    std::vector<std::tuple<int, int, float>> search_row_col;  int search_range = 10;
    for (int r = -search_range; r < search_range; r++)
    {
        for (int c = -search_range; c < search_range; c++)
        {
            if (r == 0 && c == 0) continue;
            float pixeldis = sqrt((r)*(r)+(c)*(c));
            search_row_col.push_back({r, c, pixeldis});
        }
    }
    auto search_uv_sort = [](std::tuple<int, int, float> &a, std::tuple<int, int, float> &b) -> bool
    {
      return std::get<2>(a) < std::get<2>(b);
    };
    std::sort (search_row_col.begin(), search_row_col.end(), search_uv_sort);

    std::vector<std::vector<STD>> STD_list_vec;
    std::vector<STD> STD_list_last;
    auto t_tmp = std::chrono::high_resolution_clock::now();
    debug_file<<"init data structures takes: " << time_inc(t_tmp, t_00) <<std::endl;
    // Calc mean time
    double mean_time = 0;
    float tsum_corext2d = 0.0f; float tsum_depthcal = 0.0f; float tsum_corext3d = 0.0f; float tsum_gen_std = 0.0f;
    float tsum_can_ser = 0.0f; float tsum_ransac = 0.0f; float tsum_geo = 0.0f; float tsum_update_database = 0.0f;
    float tsum_record_result = 0.0f;

    bool is_build_descriptor = false;
    int key_frame_id = 0;

    std::fstream file_;
    file_.open(bag_file, std::ios::in);
    if (!file_) {
        std::cout << "File " << bag_file << " does not exit" << std::endl;
    }
    ROS_INFO("Start to load the rosbag %s", bag_file.c_str());
    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    }
    std::vector<std::string> types;
    types.push_back(std::string("sensor_msgs/PointCloud2"));
    types.push_back(std::string("sensor_msgs/CompressedImage"));
    rosbag::View view(bag, rosbag::TypeQuery(types));
    bool is_init_bag = false;

    Eigen::Vector3d init_translation;
    int count = 0;
    Eigen::Vector3d posi_now;
    cloud_bounding_box cbx;
    std::vector<cloud_bounding_box> cbx_vec;
    std::vector<Eigen::Vector3d> posi_vec;
    double time_start;
    int is_init_starttime = false;
    int img_store_counter = 0;

    BOOST_FOREACH (rosbag::MessageInstance const m, view)
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_ptr != NULL)
        {
            if (count == 0)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
                    new pcl::PointCloud<pcl::PointXYZI>);
                key_cloud_list.push_back(temp_cloud);
            }
            //double laser_time = cloud_ptr->header.stamp.toSec();
            std::string laser_time = std::to_string(cloud_ptr->header.stamp.toSec()).erase(15);
            debug_file << "laser_time: " << laser_time << std::endl;
            pcl::PCLPointCloud2 pcl_pc;
            pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
            pcl_conversions::toPCL(*cloud_ptr, pcl_pc);
            pcl::fromPCLPointCloud2(pcl_pc, pcl_cloud);

            if (sorter.find(laser_time) == sorter.end())
            {
                std::cout << "warn: sorter cannot find this laser cloud time" << std::endl;
                continue;
            }
            Eigen::Vector3d translation = sorter[laser_time].m_lidar_t;
            if (isnan(translation[0]) || isnan(translation[1]) || isnan(translation[2]))  continue;
            if (isinf(translation[0]) || isinf(translation[1]) || isinf(translation[2]))  continue;
            posi_now = translation;
            posi_vec.push_back(posi_now);
            Eigen::Quaterniond q(sorter[laser_time].m_lidar_q);
            nav_msgs::Odometry odom;
            odom.header.frame_id = "camera_init";
            odom.pose.pose.position.x = translation[0];
            odom.pose.pose.position.y = translation[1];
            odom.pose.pose.position.z = translation[2];
            odom.pose.pose.orientation.w = q.w();
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            pubOdomAftMapped.publish(odom);
            loop.sleep();

            if (!is_init_bag)
            {
                init_translation = translation;
                translation << 0, 0, 0;
                is_init_bag = true;
            }
            else
                translation = translation - init_translation;

            pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            for (size_t i = 0; i < pcl_cloud.size(); i++)
            {
                Eigen::Vector3d pv(pcl_cloud.points[i].x, pcl_cloud.points[i].y,
                                   pcl_cloud.points[i].z);
                // pv = lidar_to_base_init_rot * pv + lidar_to_base_init_t;
                pv = q * (R_lidar_to_imu * pv + t_lidar_to_imu) + translation;  // All lidar points are registered in world frame (first imu frame)
                pcl::PointXYZI pi = pcl_cloud.points[i];
                pi.x = pv[0];
                pi.y = pv[1];
                pi.z = pv[2];
                if (isnan(pi.x)||isnan(pi.y)||isnan(pi.z))  continue;
                register_cloud->push_back(pi);
                //raw_cloud->push_back(pi);
            }

            //debug_file << "register_cloud size: " <<  register_cloud->size() << std::endl;
            down_sampling_voxel(*register_cloud, config_setting.ds_size_);

            for (size_t i = 0; i < register_cloud->size(); i++)
                key_cloud_list.back()->points.push_back(register_cloud->points[i]);

            if (count < config_setting.sub_frame_num_ - 1)
                count++;
            else
            {
                count = 0;
                is_build_descriptor = true;
                pcl::PointXYZI a_posi;
                a_posi.x = posi_now[0]; a_posi.y = posi_now[1]; a_posi.z = posi_now[2]; a_posi.intensity = key_frame_id;
                posi_cloud->push_back(a_posi);

                double x_cen = 0, y_cen = 0, z_cen = 0; int cnt = 0;
                for (size_t i = 0; i < key_cloud_list.back()->size(); i++)
                {
                    Eigen::Vector3d pv(key_cloud_list.back()->points[i].x, key_cloud_list.back()->points[i].y,
                                       key_cloud_list.back()->points[i].z);
                    if (i == 0)
                    {
                        cbx.x_lim_[0] = pv[0]; cbx.x_lim_[1] = pv[0];
                        cbx.y_lim_[0] = pv[1]; cbx.y_lim_[1] = pv[1];
                        cbx.z_lim_[0] = pv[2]; cbx.z_lim_[1] = pv[2];
                    }
                    else
                    {
                        cbx.x_lim_[0] = pv[0] < cbx.x_lim_[0] ? pv[0] : cbx.x_lim_[0];
                        cbx.y_lim_[0] = pv[1] < cbx.y_lim_[0] ? pv[1] : cbx.y_lim_[0];
                        cbx.z_lim_[0] = pv[2] < cbx.z_lim_[0] ? pv[2] : cbx.z_lim_[0];
                        cbx.x_lim_[1] = pv[0] > cbx.x_lim_[1] ? pv[0] : cbx.x_lim_[1];
                        cbx.y_lim_[1] = pv[1] > cbx.y_lim_[1] ? pv[1] : cbx.y_lim_[1];
                        cbx.z_lim_[1] = pv[2] > cbx.z_lim_[1] ? pv[2] : cbx.z_lim_[1];
                    }
                    x_cen += pv[0]; y_cen += pv[1]; z_cen += pv[2]; cnt++;
                }
                cbx_vec.push_back(cbx);
                pcl::PointXYZI a_cen;
                x_cen/=double(cnt); y_cen/=double(cnt); z_cen/=double(cnt);
                a_cen.x = x_cen; a_cen.y = y_cen; a_cen.z = z_cen; a_cen.intensity = key_frame_id;
                cloud_cens->push_back(a_cen);
            }
        }

        sensor_msgs::CompressedImage::ConstPtr img_ptr = m.instantiate<sensor_msgs::CompressedImage>();
#ifdef disable_rgb
        if (0)
#else
        if (img_ptr != NULL)
#endif
        {
            if(img_store_counter >= max_submap_img_num)   continue;
            cv::Mat temp_img = extract_image_from_msg(img_ptr, pre_ds_image_level);
            image_list[img_store_counter] = temp_img;
            cv::Mat img_gray;
            cv::cvtColor(temp_img, img_gray, CV_RGB2GRAY);

            image_gray_list[img_store_counter] = img_gray;
            cv::Mat img_resize;
            if (layer_num > 0)
            {
                pyrDown_multiple(img_gray, img_resize, layer_num);   // downsampling image for efficiency
            }
            else
                img_resize = img_gray;
            image_gray_ds_list[img_store_counter] = img_resize;

            image_times[img_store_counter] = img_ptr->header.stamp.toSec();
            img_store_counter ++;
            if (!is_init_starttime)
            {
                time_start = img_ptr->header.stamp.toSec();
                is_init_starttime = true;
                for (int si = 0; si < max_submap_img_num; si++)
                {
                    cv::Mat eig(img_resize.size(), CV_32FC1);
                    eigmat_list.push_back(eig);
                    cv::Mat cov(img_resize.size(), CV_32FC3 );
                    cov_list.push_back(cov);
                    cv::Mat Dx(img_resize.size(), CV_32FC1);
                    Dx_list.push_back(Dx);
                    cv::Mat Dy(img_resize.size(), CV_32FC1);
                    Dy_list.push_back(Dy);
                    cv::Mat Sum(img_gray.rows+1, img_gray.cols+1, CV_32S);
                    sum_list.push_back(Sum);
                }
                if (set_used_region)
                    mask_resize = set_used_region_mask(img_resize, img_resize.rows, img_resize.cols);
            }
        }

        // If enough registered LiDAR scans and images are accumulated
        // do core stuff
        if (is_build_descriptor)
        {
            float t_corext2d = 0.0f; float t_depthcal = 0.0f; float t_corext3d = 0.0f; float t_gen_std = 0.0f;
            float t_can_ser = 0.0f; float t_ransac = 0.0f; float t_geo = 0.0f; float t_update_database = 0.0f;
            float t_record_result = 0.0f;

            int skip_ratio = std::rint(float(img_store_counter) / 10.0f);
            skip_ratio = std::max(1, skip_ratio);
            std::vector<size_t> index;
            for (size_t ii = 0; ii < img_store_counter; )
            {
                index.push_back(ii);
                ii+=skip_ratio;
            }

            std::cout << "Key Frame:" << key_frame_id
                      << ", cloud size:" << key_cloud_list[key_frame_id]->size() << std::endl;
            debug_file << std::endl << "Key frame:" << key_frame_id
                       << ", cloud size:" << key_cloud_list[key_frame_id]->size()
                       << ", image size:" << img_store_counter << std::endl;

            std::vector<std::pair<int, float>> proposed_candidates;
            if (key_cloud_list[key_frame_id]->empty())
            {
                auto t_record_result_start = std::chrono::high_resolution_clock::now();
                if (save_for_pr_eval) gt_doc  << " " << key_frame_id << " " << posi_now.x() << " " << posi_now.y() << " " << posi_now.z()
                                              << " " << -1 << " " << 0 << " " << -1 << " " << 0 << " " << 0 << std::endl;
                if (save_for_avgrecall_eval) record_topN_loopclosure(key_frame_id, posi_vec, proposed_candidates, std::round((4543/config_setting.sub_frame_num_)*0.01), gt_doc); //4543 is total frame num of kitti00
                auto t_record_result_end = std::chrono::high_resolution_clock::now();
                t_record_result += time_inc(t_record_result_end,t_record_result_start);
                key_frame_id++;
                is_build_descriptor = false;
                img_store_counter = 0;
                continue;
            }

            // -------------------------------------------------**Generate 3D key pt from LiDAR scans**------------------------------------------------- //
            auto t_cor_ext_start = std::chrono::high_resolution_clock::now();
            std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;
            init_voxel_map(config_setting, *key_cloud_list[key_frame_id], voxel_map);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr frame_plane_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
            get_plane(voxel_map, frame_plane_cloud);
            history_plane_list.push_back(frame_plane_cloud);

            std::vector<Plane *> proj_plane_list;
            std::vector<Plane *> merge_plane_list;
            get_project_plane(config_setting, voxel_map, proj_plane_list);
            sort(proj_plane_list.begin(), proj_plane_list.end(), plane_greater_sort);
            if (proj_plane_list.empty())
            {
                auto t_record_result_start = std::chrono::high_resolution_clock::now();
                if (save_for_pr_eval) gt_doc  << " " << key_frame_id << " " << posi_now.x() << " " << posi_now.y() << " " << posi_now.z()
                                              << " " << -1 << " " << 0 << " " << -1 << " " << 0 << " " << 0 << std::endl;
                if (save_for_avgrecall_eval) record_topN_loopclosure(key_frame_id, posi_vec, proposed_candidates, std::round((4543/config_setting.sub_frame_num_)*0.01), gt_doc); //4543 is total frame num of kitti00
                auto t_record_result_end = std::chrono::high_resolution_clock::now();
                t_record_result += time_inc(t_record_result_end,t_record_result_start);
                key_frame_id++;
                for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++)
                    delete (iter->second);

                if (!corner_list.empty()) std::vector<std::vector<cv::Point2f>>().swap(corner_list);
                is_build_descriptor = false;
                img_store_counter = 0;
                continue;
            }
            merge_plane(config_setting, proj_plane_list, merge_plane_list);
            sort(merge_plane_list.begin(), merge_plane_list.end(), plane_greater_sort);

            std::vector<BinaryDescriptor> binary_list;
            binary_extractor(config_setting, merge_plane_list,
                             key_cloud_list[key_frame_id], binary_list);

            auto t_cor_ext_end = std::chrono::high_resolution_clock::now();
            t_corext3d = time_inc(t_cor_ext_end, t_cor_ext_start);
            // ------------------------------------------------------------------------------------------------------------------------------- //

#ifndef disable_rgb
            // -------------------------------------------------**Generate 3D key pt from camera images**------------------------------------------------- //
            if (img_store_counter > 0)
#else
            if (0)
#endif
            {
                auto t_tmp = std::chrono::high_resolution_clock::now();
                std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &img_id)
                //for (auto img_id:index)
                {
                    cv::Mat &gray_ds_img  = image_gray_ds_list[img_id];
                    cv::Mat &eig = eigmat_list[img_id];
                    cv::Mat &eigtmp  = eigtmp_list[img_id];
                    cv::Mat &cov = cov_list[img_id];
                    cv::Mat &Dx = Dx_list[img_id];
                    cv::Mat &Dy = Dy_list[img_id];
                    cv::Mat &temp_img = image_list[img_id];
                    cv::Mat &gray_img  = image_gray_list[img_id];
                    cv::Mat &sum = sum_list[img_id];
                    std::string img_time_crop = std::to_string(image_times[img_id]).erase(15);

                    if (set_used_region)
                        extract_2dkeypt_roughly(eig, eigtmp, gray_ds_img, cov, Dx, Dy, mask_resize,
                                                corners_tmp_list[img_id], corners_out_list[img_id], harrisR_list[img_id], layer_num);
                    else
                        extract_2dkeypt_roughly(eig, eigtmp, gray_ds_img, cov, Dx, Dy, cv::Mat(),
                                                corners_tmp_list[img_id], corners_out_list[img_id], harrisR_list[img_id], layer_num);

                    std::string fname = save_directory+std::to_string(key_frame_id)+"_"+std::to_string(img_id)+".png";
                    find_more_accurate_pixel(temp_img, gray_img, corners_tmp_list[img_id], corners_out_list[img_id], layer_num, fname);
                    extract_brief_fromimage(corners_out_list[img_id], gray_img, sum, descriptors_list[img_id],
                                            brief_arrays_mm128_list[img_id]);

                    std::vector<kpt_2d_info> kpts_from_img(corners_out_list[img_id].size());
                    for( int i = 0; i < corners_out_list[img_id].size(); i++ )
                    {
                        kpt_2d_info a_kpt2d;
                        a_kpt2d.pt_2d = corners_out_list[img_id][i];
                        a_kpt2d.brief_array = brief_arrays_mm128_list[img_id][i];
                        a_kpt2d.Rval = harrisR_list[img_id][i];
                        a_kpt2d.c_q_w = sorter[img_time_crop].m_pose_c_2_w_q;
                        a_kpt2d.c_t_w = sorter[img_time_crop].m_pose_c_2_w_t;
                        kpts_from_img[i] = a_kpt2d;
                    }
                    kpts_from_img_list[img_id] = kpts_from_img;
                }
                );

                auto t_cor_ext2 = std::chrono::high_resolution_clock::now();
                t_corext2d = time_inc(t_cor_ext2, t_tmp);
                //debug_file<<"extract 2d keypt and brief descriptor for images takes: " << time_inc(t_cor_ext2, t_tmp) <<std::endl;

                // Visualize 2d keypt with raw image
                if (0)
                {
                    for (auto img_id:index)
                    {
                        cv::Mat temp_img2 = image_list[img_id].clone();
                        for( int i = 0; i < corners_out_list[img_id].size(); i++ )
                        {
                          cv::circle( temp_img2, corners_out_list[img_id][i], 6, cv::Scalar(0,255,0));
                        }

                        cv_bridge::CvImage out_msg;
                        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
                        out_msg.image = temp_img2;
                        pub_raw_img.publish(out_msg);
                    }
                }

                auto t_2dto3d_start = std::chrono::high_resolution_clock::now();
                for (auto img_id:index)
                {
                    //for ( auto &val : depth_mask_list[img_id] )
                        //val = -1;
                    memset(depthf_mask_list[img_id], -1, sizeof(int)*IMG_WID*IMG_HEI);
                }

                find3dpts_for2dcorners(kpts_from_img_list, depthf_mask_list, key_cloud_list[key_frame_id],
                                       image_times, sorter, index, search_row_col,
                                       IMG_HEI, IMG_WID, fx, fy, cx, cy, save_directory);
                auto t_2dto3d = std::chrono::high_resolution_clock::now();

                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cor3d_cloud_ds(new pcl::PointCloud<pcl::PointXYZINormal>);
                std::vector<std::vector<__m128i>> out_brief_list;
                non_maximal_suppression(cor3d_cloud_ds, out_brief_list, kpts_from_img_list, index, config_setting);
                auto t_nonmax = std::chrono::high_resolution_clock::now();
                t_depthcal = time_inc(t_nonmax, t_2dto3d_start);

                //debug_file<<"find3dpts_for2dcorners takes: " << time_inc(t_2dto3d, t_2dto3d_start) <<std::endl;
                //debug_file<<"non_maximal_suppression takes: " << time_inc(t_nonmax, t_2dto3d) <<std::endl;

                // Visualize extracted 2d keypt on image in rviz
                std::vector<Eigen::Vector3d> projected_cor3d;
                std::string img_time = std::to_string(image_times[0]).erase(15);
                project_onimage(projected_cor3d, cor3d_cloud_ds, sorter[img_time].m_pose_c_2_w_q,
                                sorter[img_time].m_pose_c_2_w_t, sorter[img_time].m_pose_w_2_c_t,
                                IMG_HEI, IMG_WID, fx, fy, cx, cy);
                for (int idx = 0; idx < cor3d_cloud_ds->size(); idx++)
                {
                    auto a_cor3d = cor3d_cloud_ds->points[idx];
                    BinaryDescriptor abin;
                    abin.location_ = Eigen::Vector3d(a_cor3d.x, a_cor3d.y, a_cor3d.z);
                    abin.occupy_array_ = std::vector<bool>();
                    abin.summary_ = 0;
                    abin.brf_idx = all_briefs.size();
                    all_briefs.push_back(out_brief_list[idx]);
                    abin.harrisR_ = int(cor3d_cloud_ds->points[idx].intensity);//idx;
                    abin.breif_obs_t = image_times[int(cor3d_cloud_ds->points[idx].normal_x)];
                    binary_list.push_back(abin);
                }
                // ------------------------------------------------------------------------------------------------------------------------------- //

                auto tmp_image = image_list[0];
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_2d (new pcl::PointCloud<pcl::PointXYZINormal>());
                for (auto a_pcor3d:projected_cor3d)
                {
                    pcl::PointXYZINormal pt2d;
                    pt2d.x = a_pcor3d[1];
                    pt2d.y = a_pcor3d[0];
                    pt2d.z = 0;
                    pt2d.intensity = a_pcor3d[2];
                    cloud_2d->push_back(pt2d);
                    cv::circle( tmp_image, cv::Point2f(pt2d.x, pt2d.y), 6, cv::Scalar(0,255,0));
                    cv::putText( tmp_image, std::to_string(int(cor3d_cloud_ds->points[int(pt2d.intensity)].intensity)),
                                 cv::Point(pt2d.x, pt2d.y), cv::FONT_HERSHEY_DUPLEX, 1.5,  CV_RGB(255, 0, 0), 1.5);
                }
                if (0) // Save as png
                {
                    std::string filename = save_directory + std::to_string(key_frame_id) + "_overall.png";
                    cv::imwrite(filename.c_str(), tmp_image);
                }
                cv_bridge::CvImage tmp_msg;
                tmp_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
                tmp_msg.image = tmp_image;
                pub_dep_img.publish(tmp_msg);
                loop.sleep();
                //std::cin.get();
            }

            // -------------------------------------------------**Generate ibtc descripotrs**------------------------------------------------- //
            auto t_gen_std_start = std::chrono::high_resolution_clock::now();
            std::vector<STD> STD_list;
            float max_triangle_side_len = generate_std(config_setting, binary_list, key_frame_id, STD_list);
            //debug_file<<"STD_list size " << STD_list.size() <<std::endl;
#ifndef disable_rgb
            auto STD_list_bkq = STD_list;
            if (max_triangle_side_len < 9 || STD_list.size() < 500)
            {
                debug_file<< "Generated descriptors too small or too few descriptors !!! also use last frame's."  <<std::endl;
                for (auto &var : STD_list_last)  // also use last frame descriptor
                {
                    var.frame_number_ = key_frame_id;
                    STD_list.push_back(var);
                }
            }
            //debug_file<<"STD_list size " << STD_list.size() <<std::endl;
#endif
            history_binary_list.push_back(binary_list);
            auto t_gen_std_end = std::chrono::high_resolution_clock::now();
            t_gen_std = time_inc(t_gen_std_end, t_gen_std_start);

            debug_file << "[Corner] corner size:" << binary_list.size()
                       << "  descriptor size:" << STD_list.size() << std::endl;

            STD_list_vec.push_back(STD_list);
#ifndef disable_rgb
            STD_list_last = STD_list_bkq;
#endif
            // ------------------------------------------------------------------------------------------------------------------------------- //

            // Visualize descriptors in rviz
            pcl::PointCloud<pcl::PointXYZ> key_points_cloud;
            for (auto var : binary_list)
            {
                pcl::PointXYZ pi;
                pi.x = var.location_[0];
                pi.y = var.location_[1];
                pi.z = var.location_[2];
                key_points_cloud.push_back(pi);
            }
            sensor_msgs::PointCloud2 pub_cloud;
            pcl::toROSMsg(key_points_cloud, pub_cloud);
            pub_cloud.header.frame_id = "camera_init";
            pubCurrentBinary.publish(pub_cloud);
            loop.sleep();
            Eigen::Vector3d color1(1, 0, 0);
            publish_binary(binary_list, color1, "current", pubSTD);
            loop.sleep();
            //publish_std_list(STD_list, pubSTD, Eigen::Vector3d(0,1,0));
            //loop.sleep();

            auto t_record_result_start = std::chrono::high_resolution_clock::now();
            if (save_for_pr_eval)
                check_record_gtloopclosure(/*posi_cloud*/cloud_cens,posi_now,key_cloud_list,cbx_vec,key_frame_id,
                                           range_gtlc_search,config_setting,debug_file,gt_doc);
            auto t_record_result_end = std::chrono::high_resolution_clock::now();
            t_record_result += time_inc(t_record_result_end,t_record_result_start);

            sensor_msgs::PointCloud2 laserCloudmsg;
            pcl::toROSMsg(*key_cloud_list[key_frame_id], laserCloudmsg);
            laserCloudmsg.header.stamp = ros::Time::now();
            laserCloudmsg.header.frame_id = "camera_init";
            pubRegisterCloud.publish(laserCloudmsg);

            pub_keyframe_ids(id_vec, pubCorrectionId,  posi_now, key_frame_id, "camera_init");

            // -------------------------------------------------**Loop Detection**------------------------------------------------- //
            // Candidate search
            auto t_can_ser_start = std::chrono::high_resolution_clock::now();
            std::vector<STDMatchList> alternative_match;
#ifdef disable_rgb
            candidate_searcher(config_setting, STD_map, STD_list, alternative_match, debug_file);
#else
            rgb_candidate_searcher(config_setting, STD_map, STD_map_brf, STD_list, alternative_match, all_briefs, debug_file);
#endif
            auto t_can_ser_end = std::chrono::high_resolution_clock::now();
            t_can_ser = time_inc(t_can_ser_end,t_can_ser_start);
            //debug_file << "rgb_candidate_searcher time:" << t_can_ser << std::endl;

            // Ransac + Geometrical verification
            bool triggle_loop = false;
            Eigen::Vector3d best_t;
            Eigen::Matrix3d best_rot;
            Eigen::Vector3d loop_translation;
            Eigen::Matrix3d loop_rotation;
            std::vector<std::pair<STD, STD>> sucess_match_list;
            std::vector<std::pair<STD, STD>> unsucess_match_list;
            std::vector<std::pair<STD, STD>> sucess_match_list_publish;
            std::vector<std::pair<STD, STD>> unsucess_match_list_publish;
            int match_size = 0;
            int rough_size = 0;
            int candidate_id = -1;
            double mean_triangle_dis = 0;
            double mean_binary_similarity = 0;
            double mean_brief_similarity = 0;
            double outlier_mean_triangle_dis = 0;
            double outlier_mean_binary_similarity = 0;
            double outlier_mean_brief_similarity = 0;
            int match_frame = 0;
            double best_score = 0;
            int best_frame = -1;
            bool is_rgb_case_final = false;
            bool btc_match_found = false;

            for (int i = 0; i < alternative_match.size(); i++)
            {
                if (alternative_match[i].match_list_.size() > 0)
                {
                    bool fine_sucess = false;
                    Eigen::Matrix3d std_rot;
                    Eigen::Vector3d std_t;
                    //debug_file << "[Rough match] rough match frame:"
                               //<< alternative_match[i].match_frame_ << " match size:"
                               //<< alternative_match[i].match_list_.size() << std::endl;
                    sucess_match_list.clear();
                    auto t_ransac_start = std::chrono::high_resolution_clock::now();
                    bool is_rgb_case = false;
#ifdef disable_rgb
                    fine_loop_detection_tbb(
                        config_setting, alternative_match[i].match_list_, fine_sucess,
                        std_rot, std_t, sucess_match_list, unsucess_match_list);
#else
                    rgb_fine_loop_detection_tbb(
                        config_setting, alternative_match[i].match_list_, fine_sucess,
                        std_rot, std_t, sucess_match_list, unsucess_match_list, is_rgb_case, alternative_match[i].match_frame_, debug_file);
#endif
                    auto t_ransac_end = std::chrono::high_resolution_clock::now();
                    t_ransac += time_inc(t_ransac_end, t_ransac_start);
                    if (fine_sucess)
                    {

                        double score = geometric_verify(config_setting,
                            frame_plane_cloud,
                            history_plane_list[alternative_match[i].match_frame_], std_rot,
                            std_t);

                        debug_file << "[Geo Ver] Icp score:" << score << ", match frame:" << alternative_match[i].match_frame_ << std::endl;
                        proposed_candidates.push_back({alternative_match[i].match_frame_,score});
                        if (score > best_score)
                        {
                            unsucess_match_list_publish = unsucess_match_list;
                            sucess_match_list_publish = sucess_match_list;
                            best_frame = alternative_match[i].match_frame_;
                            best_score = score;
                            best_rot = std_rot;
                            best_t = std_t;
                            rough_size = alternative_match[i].match_list_.size();
                            match_size = sucess_match_list.size();
                            candidate_id = i;
                            if (score > icp_threshold) is_rgb_case_final = is_rgb_case;
                            if (!is_rgb_case && score > icp_threshold)  btc_match_found = true;
                        }
                    }
                    auto t_geo_end = std::chrono::high_resolution_clock::now();
                    t_geo += time_inc(t_geo_end, t_ransac_end);
                }
            }

            if (best_score > icp_threshold)
            {
                loop_translation = best_t;
                loop_rotation = best_rot;

                match_frame = best_frame;
                triggle_loop = true;
                mean_triangle_dis =
                    calc_triangle_dis(sucess_match_list_publish);
                mean_binary_similarity =
                    calc_binary_similaity(sucess_match_list_publish);
                outlier_mean_triangle_dis =
                    calc_triangle_dis(unsucess_match_list_publish);
                outlier_mean_binary_similarity =
                    calc_binary_similaity(unsucess_match_list_publish);
            }
            else
                triggle_loop = false;

            is_build_descriptor = false;
            //raw_cloud->clear();
            // ------------------------------------------------------------------------------------------------------------------------------- //

            // Record result into text file
            auto t_record_result_start2 = std::chrono::high_resolution_clock::now();
            if (save_for_pr_eval)
                record_proposed_loopclosure(key_cloud_list,cbx_vec,key_frame_id,best_frame,best_score,config_setting,debug_file,gt_doc);
            if (save_for_avgrecall_eval)
                record_topN_loopclosure(key_frame_id, posi_vec, proposed_candidates, std::round((4543/config_setting.sub_frame_num_)*0.01), gt_doc);
                                                                                               //4543 is total frame num of kitti00
            auto t_record_result_end2 = std::chrono::high_resolution_clock::now();
            t_record_result += time_inc(t_record_result_end2,t_record_result_start2);

            if (triggle_loop)
            {
                debug_file << "[Loop Sucess] " << key_frame_id << "--" << match_frame
                           << ", candidate id:" << candidate_id
                           << ", plane-plane overlap:" << best_score << std::endl;
                std::cout  << "[Loop Sucess] " << key_frame_id << "--" << match_frame
                           << ", candidate id:" << candidate_id
                           << ", plane-plane overlap:" << best_score << std::endl;
                debug_file << "[Loop Info] "
                           << "rough size:" << rough_size
                           << ", match size:" << match_size
                           << ", rough triangle dis:" << outlier_mean_triangle_dis
                           << ", fine triangle dis:" << mean_triangle_dis
                           << ", rough binary similarity:" << outlier_mean_binary_similarity
                           << ", fine binary similarity:" << mean_binary_similarity
                           << ", rough brief similarity:" << outlier_mean_brief_similarity
                           << ", fine brief similarity:" << mean_brief_similarity
                           << std::endl;

                pcl::toROSMsg(*key_cloud_list[match_frame], pub_cloud);
                pub_cloud.header.frame_id = "camera_init";
                pubMatchedCloud.publish(pub_cloud);
                loop.sleep();
                pcl::PointCloud<pcl::PointXYZ> matched_key_points_cloud;
                for (auto var : history_binary_list[match_frame])
                {
                    pcl::PointXYZ pi;
                    pi.x = var.location_[0];
                    pi.y = var.location_[1];
                    pi.z = var.location_[2];
                    matched_key_points_cloud.push_back(pi);
                }
                pcl::toROSMsg(matched_key_points_cloud, pub_cloud);
                pub_cloud.header.frame_id = "camera_init";
                pubMatchedBinary.publish(pub_cloud);
                Eigen::Vector3d color2(0, 1, 0);
                if (is_rgb_case_final && !btc_match_found)
                    color2 = Eigen::Vector3d(0, 0, 1);
                publish_binary(history_binary_list[match_frame], color2, "history", pubSTD);
                loop.sleep();

                publish_std(sucess_match_list_publish, pubSTD, color2);
                //debug_file << "sucess_match_list_publish: " << sucess_match_list_publish.size() << std::endl;
                loop.sleep();
            }
            else
            {
                debug_file << "[Loop Fail] " << key_frame_id << " -- " << best_frame << ", plane-plane overlap:" << best_score << std::endl;
                std::cout  << "[Loop Fail] " << key_frame_id << " -- " << best_frame << ", plane-plane overlap:" << best_score << std::endl;
            }

            auto t_update_database_start = std::chrono::high_resolution_clock::now();
            add_STD(STD_map, STD_list);
            add_STD_brf(STD_map_brf, STD_list);
            auto t_update_database_end = std::chrono::high_resolution_clock::now();
            t_update_database = time_inc(t_update_database_end,t_update_database_start);

            tsum_gen_std+=t_gen_std; tsum_corext3d+=t_corext3d;tsum_corext2d+=t_corext2d; tsum_depthcal+=t_depthcal;
            tsum_can_ser+=t_can_ser; tsum_ransac+=t_ransac;    tsum_geo+=t_geo; tsum_update_database+=t_update_database;
            mean_time += (t_gen_std+t_gen_std+t_corext2d+t_depthcal+t_can_ser+t_ransac+t_geo);
            debug_file << "[Time] build descriptor:" << tsum_gen_std/float(key_frame_id)
                       << ", update database:" << tsum_update_database/float(key_frame_id)
                       << ", extract 3d keypt from cloud geometry:" << tsum_corext3d/float(key_frame_id)
                       << ", extract 2d keypt from images:" << tsum_corext2d/float(key_frame_id)
                       << ", depth matrix calculation:" << tsum_depthcal/float(key_frame_id)
                       << ", loop candidate search:" << tsum_can_ser/float(key_frame_id)
                       << ", triangle ransac:" <<tsum_ransac/float(key_frame_id)
                       << ", plane-plane verification:" <<tsum_geo/float(key_frame_id)
                       << ", average total time:" << mean_time / (key_frame_id + 1) << " ms" << std::endl;
            std::cout  << "[Time] build descriptor:" << tsum_gen_std/float(key_frame_id)
                       << ", update database:" << tsum_update_database/float(key_frame_id)
                       << ", extract 3d keypt from cloud geometry:" << tsum_corext3d/float(key_frame_id)
                       << ", extract 2d keypt from images:" << tsum_corext2d/float(key_frame_id)
                       << ", depth matrix calculation:" << tsum_depthcal/float(key_frame_id)
                       << ", loop candidate search:" << tsum_can_ser/float(key_frame_id)
                       << ", triangle ransac:" <<tsum_ransac/float(key_frame_id)
                       << ", plane-plane verification:" <<tsum_geo/float(key_frame_id)
                       << ", average total time:" << mean_time / (key_frame_id + 1) << " ms" << std::endl;
            tsum_record_result += t_record_result;
            debug_file << "[Time] average time for recording result and searching ground truth:" << tsum_record_result/float(key_frame_id) << " ms" << std::endl;
            std::cout  << "[Time] average time for recording result and searching ground truth:" << tsum_record_result/float(key_frame_id) << " ms" << std::endl << std::endl;

            timelog_file << key_frame_id << " " << t_gen_std << " " << t_update_database
                         << " " << t_corext3d << " " << t_corext2d << " " << t_depthcal
                         << " " << t_can_ser << " " << t_ransac << " " << t_geo << std::endl;

            key_frame_id++;
            for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++)
                delete (iter->second);

            img_store_counter = 0;
            if (!corner_list.empty()) std::vector<std::vector<cv::Point2f>>().swap(corner_list);
        }
    }
    depthf_mask_list.clear();
    delete[] empty_arr0, empty_arr1, empty_arr2, empty_arr3, empty_arr4, empty_arr5, empty_arr6, empty_arr7, empty_arr8, empty_arr9;
    delete[] empty_arr10, empty_arr11, empty_arr12, empty_arr13, empty_arr14, empty_arr15, empty_arr16, empty_arr17, empty_arr18, empty_arr19;
    delete[] empty_arr20, empty_arr21, empty_arr22, empty_arr23, empty_arr24, empty_arr25, empty_arr26, empty_arr27, empty_arr28, empty_arr29;
}
