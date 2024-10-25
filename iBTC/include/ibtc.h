#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <cv_bridge/cv_bridge.h>
#include <execution>
#include <fstream>
#include <mutex>
//#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

#include <chrono>

#define HASH_P 116101
#define MAX_N 10000000000

typedef struct cloud_bounding_box {
  Eigen::Vector2f x_lim_;
  Eigen::Vector2f y_lim_;
  Eigen::Vector2f z_lim_;
  int frame_id_;
} cloud_bounding_box;

typedef struct ConfigSetting {
  /* for point cloud pre-preocess*/
  int stop_skip_enable_ = 0;
  float ds_size_ = 0.5;
  int useful_corner_num_ = 30;
  int point_skip_ = 1;

  /* for key points*/
  float plane_merge_normal_thre_;
  float plane_merge_dis_thre_;
  float plane_detection_thre_ = 0.01;
  float voxel_size_ = 1.0;
  int voxel_init_num_ = 10;
  int proj_plane_num_ = 3;
  float proj_image_resolution_ = 0.5;
  float proj_image_high_inc_ = 0.5;
  float proj_dis_min_ = 0.2;
  float proj_dis_max_ = 5;
  float summary_min_thre_ = 10;
  int line_filter_enable_ = 0;
  int touch_filter_enable_ = 0;

  /* for STD */
  float descriptor_near_num_ = 10;
  float descriptor_min_len_ = 1;
  float descriptor_max_len_ = 10;
  float non_max_suppression_radius_ = 3.0;
  float std_side_resolution_ = 0.2;

  /* for place recognition*/
  int skip_near_num_ = 20;
  int candidate_num_ = 50;
  int sub_frame_num_ = 10;
  float rough_dis_threshold_ = 0.03;
  float similarity_threshold_ = 0.7;
  float icp_threshold_ = 0.5;
  int ransac_Rt_thr = 4;
  float normal_threshold_ = 0.1;
  float dis_threshold_ = 0.3;
  /* for data base*/
  int std_add_skip_frame_ = 1;

  int vis_descriptor_near_num_ = 50;
  float vis_sim_threshold_ = 0.7;
  int vis_useful_corner_num_ = 30;

  /* for result record*/
  int is_kitti_ = 1;
  /* extrinsic for lidar to vehicle*/
  Eigen::Matrix3d rot_lidar_to_vehicle_;
  Eigen::Vector3d t_lidar_to_vehicle_;

  /* for gt file style*/
  int gt_file_style_ = 0;

} ConfigSetting;

struct imu_lidar_camera
{
    imu_lidar_camera()
    {
        msg_time = 0.0;
        camera_poseset = false;
        lidar_poseset = false;
//        this->lidar.reset(new PointCloudXYZI());
    };
    double msg_time;
    bool camera_poseset;
    bool lidar_poseset;
    Eigen::Quaterniond m_pose_w_2_c_q, m_pose_c_2_w_q;
    Eigen::Vector3d m_pose_w_2_c_t, m_pose_c_2_w_t;

    Eigen::Quaterniond m_lidar_q;
    Eigen::Vector3d m_lidar_t;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef struct BinaryDescriptor {
    BinaryDescriptor()
    {
        brf_idx = -1;
    };
    std::vector<bool> occupy_array_;
    unsigned char summary_;
    //std::vector<uint8_t> brief_array_;
    unsigned int brf_idx;
    double breif_obs_t;
    float harrisR_;
    Eigen::Vector3d location_;
} BinaryDescriptor;

typedef struct BinaryDescriptorF {
  // std::vector<bool> occupy_array_;
  // bool occupy_array_[49];
  unsigned char summary_;
  Eigen::Vector3f location_;
} BinaryDescriptorF;

// 1kb,12.8
typedef struct STD {
  Eigen::Vector3d triangle_;
  Eigen::Vector3d angle_;
  Eigen::Vector3d center_;
  unsigned short frame_number_;
  // std::vector<unsigned short> score_frame_;
  // std::vector<Eigen::Matrix3d> position_list_;
  BinaryDescriptor binary_A_;
  BinaryDescriptor binary_B_;
  BinaryDescriptor binary_C_;
} STD;

typedef struct STDF {
  Eigen::Vector3f triangle_;
  Eigen::Vector3f angle_;
  // Eigen::Vector3f A_;
  // Eigen::Vector3f B_;
  // Eigen::Vector3f C_;
  // unsigned char count_A_;
  // unsigned char count_B_;
  // unsigned char count_C_;
  Eigen::Vector3f center_;
  // float triangle_scale_;
  unsigned short frame_number_;
  // BinaryDescriptorF binary_A_;
  // BinaryDescriptorF binary_B_;
  // BinaryDescriptorF binary_C_;
} STDF;

typedef struct Plane {
  pcl::PointXYZINormal p_center_;
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Matrix3d covariance_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float d_ = 0;
  int id_ = 0;
  int sub_plane_num_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
} Plane;

typedef struct STDMatchList {
  std::vector<std::pair<STD, STD>> match_list_;
  int match_frame_;
  double mean_dis_;
} STDMatchList;

struct M_POINT {
  float xyz[3];
  float intensity;
  int count = 0;
};

class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// Hash value
namespace std {
template <> struct hash<VOXEL_LOC> {
  int64 operator()(const VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};
} // namespace std

class STD_LOC {
public:
  int64_t x, y, z, a, b, c;

  STD_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0, int64_t va = 0,
          int64_t vb = 0, int64_t vc = 0)
      : x(vx), y(vy), z(vz), a(va), b(vb), c(vc) {}

  bool operator==(const STD_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
    // return (x == other.x && y == other.y && z == other.z && a == other.a &&
    //         b == other.b && c == other.c);
  }
};

namespace std {
template <> struct hash<STD_LOC> {
  int64 operator()(const STD_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
    // return ((((((((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N +
    //              (s.x)) *
    //             HASH_P) %
    //                MAX_N +
    //            s.a) *
    //           HASH_P) %
    //              MAX_N +
    //          s.b) *
    //         HASH_P) %
    //            MAX_N +
    //        s.c;
  }
};
} // namespace std

inline int hamming_dis(const std::vector<bool> &b1, const std::vector<bool> &b2)
{
    int x = std::accumulate(b1.rbegin(), b1.rend(), 0, [](int i, int j) { return (i<<1)+j; });
    int y = std::accumulate(b2.rbegin(), b2.rend(), 0, [](int i, int j) { return (i<<1)+j; });
    int xor_ = x ^ y;
    int distance = 0;
    while (xor_ != 0)
    {
        distance += 1;
        xor_ = xor_ & (xor_ - 1);
    }
    return distance;
}

inline int hamming_dis(int x, int y)
{
    int xor_ = x ^ y;
    int distance = 0;
    while (xor_ != 0)
    {
        distance += 1;
        xor_ = xor_ & (xor_ - 1);
    }
    return distance;
}

inline int hamming_distance_fix_omp(std::vector<bool> &b1, std::vector<bool> &b2, int n) {
    int disti = 0;
#pragma omp parallel for reduction(+:disti)
    for(int i=0; i<n; i++)
    {
       disti += (b1[i] != b2[i]);
    }
    return disti;
}

#include <emmintrin.h>    // SSE2
//inline size_t count_equal(const std::vector<uint8_t> &vec1, const std::vector<uint8_t> &vec2, const int &n)
//{
//    __m128i vcount = _mm_setzero_si128();
//    size_t i = 0;

//    for ( ; i + 16 <= n; i += 16)           // for each vector in block
//    {
//        __m128i v1 = _mm_loadu_si128((__m128i *)&vec1[i]);
//        __m128i v2 = _mm_loadu_si128((__m128i *)&vec2[i]);
//        __m128i vcmp = _mm_cmpeq_epi8(v1, v2);
//        vcount = _mm_sub_epi8(vcount, vcmp);
//    }
//    vcount = _mm_sad_epu8(vcount, _mm_setzero_si128());
//    return _mm_extract_epi16(vcount, 0) + _mm_extract_epi16(vcount, 4);
//}

inline size_t count_equal(const std::vector<__m128i> &vec1, const std::vector<__m128i> &vec2, const int &n)
{
    //const size_t n = vec1.size();
    __m128i vcmp;
    __m128i vcount = _mm_setzero_si128();
    size_t i;

    for ( ; i < n; ++i )           // for each vector in block
    {
        vcmp = _mm_cmpeq_epi8(vec1[i], vec2[i]);
        vcount = _mm_sub_epi8(vcount, vcmp);
    }

    vcount = _mm_sad_epu8(vcount, _mm_setzero_si128());

    return _mm_extract_epi16(vcount, 0) + _mm_extract_epi16(vcount, 4);
}

inline size_t count_equal_tgt(const std::vector<__m128i> &vec1a, const std::vector<__m128i> &vec2a,
                              const std::vector<__m128i> &vec1b, const std::vector<__m128i> &vec2b,
                              const std::vector<__m128i> &vec1c, const std::vector<__m128i> &vec2c, const int &n)
{
    __m128i vcmp;
    __m128i vcount = _mm_setzero_si128();
    size_t i = 0, count = 0;

    for ( ; i < n; ++i )           // for each vector in block
    {
        vcmp = _mm_cmpeq_epi8(vec1a[i], vec2a[i]);
        vcount = _mm_sub_epi8(vcount, vcmp);
    }
    vcount = _mm_sad_epu8(vcount, _mm_setzero_si128());
    count += _mm_extract_epi16(vcount, 0) + _mm_extract_epi16(vcount, 4);

    vcount = _mm_setzero_si128();
    i = 0;

    for ( ; i < n; ++i )           // for each vector in block
    {
        vcmp = _mm_cmpeq_epi8(vec1b[i], vec2b[i]);
        vcount = _mm_sub_epi8(vcount, vcmp);
    }
    vcount = _mm_sad_epu8(vcount, _mm_setzero_si128());
    count += _mm_extract_epi16(vcount, 0) + _mm_extract_epi16(vcount, 4);

    vcount = _mm_setzero_si128();
    i = 0;

    for ( ; i < n; ++i )           // for each vector in block
    {
        vcmp = _mm_cmpeq_epi8(vec1c[i], vec2c[i]);
        vcount = _mm_sub_epi8(vcount, vcmp);
    }
    vcount = _mm_sad_epu8(vcount, _mm_setzero_si128());
    count += _mm_extract_epi16(vcount, 0) + _mm_extract_epi16(vcount, 4);
    return count;
}
class OctoTree {
public:
  ConfigSetting config_setting_;
  std::vector<Eigen::Vector3d> voxel_points_;
  std::vector<float> intensities_;
  int sup_num_ = 0;
  int max_id_ = -1;
  bool is_max = true;
  Plane *plane_ptr_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  int merge_num_ = 0;
  bool is_project_ = false;
  std::vector<Eigen::Vector3d> project_normal;
  bool is_publish_ = false;
  OctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  int surround_ptnum_ = 0;
  float quater_length_;
  bool init_octo_;
  OctoTree(const ConfigSetting &config_setting)
      : config_setting_(config_setting) {
    voxel_points_.clear();
    octo_state_ = 0;
    layer_ = 0;
    init_octo_ = false;
    for (int i = 0; i < 8; i++) {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new Plane;
  }
  void init_plane();
  void init_octo_tree();
};

void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size);

void load_config_setting(std::string &config_file,
                         ConfigSetting &config_setting);

void load_pose_with_time(
    const std::string &pose_file,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &pose_list,
    std::vector<double> &time_list);

void load_cu_pose_with_time(
    const std::string &pose_file,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &pose_list,
    std::vector<double> &time_list);

void load_pose_with_frame(
    const std::string &pose_file,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &pose_list,
    std::vector<int> &frame_number_list);

double binary_similarity(const BinaryDescriptor &b1,
                         const BinaryDescriptor &b2);
double binary_similarity(const BinaryDescriptor &b1,
                         const BinaryDescriptor &b2, int &counter);

double brief_similarity(const std::vector<uint8_t> &b1,
                        const std::vector<uint8_t> &b2, int &counter);

bool binary_greater_sort(BinaryDescriptor a, BinaryDescriptor b);
bool plane_greater_sort(Plane *plane1, Plane *plane2);

void init_voxel_map(const ConfigSetting &config_setting,
                    const pcl::PointCloud<pcl::PointXYZI> &input_cloud,
                    std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map);

void init_voxel_map_cor3d(const ConfigSetting &config_setting,
                          const pcl::PointCloud<pcl::PointXYZI> &input_cloud,
                          std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map);

void get_plane(std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
               pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud);

void get_project_plane(const ConfigSetting &config_setting,
                       std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map,

                       std::vector<Plane *> &project_plane_list);

void merge_plane(const ConfigSetting &config_setting,
                 std::vector<Plane *> &origin_list,
                 std::vector<Plane *> &merge_plane_list);

void non_max_suppression(const ConfigSetting &config_setting,
                         std::vector<BinaryDescriptor> &binary_list);

void binary_extractor(const ConfigSetting &config_setting,
                      const std::vector<Plane *> proj_plane_list,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                      std::vector<BinaryDescriptor> &binary_descriptor_list);

void binary_extractor_rgb(const ConfigSetting &config_setting,
                      const std::vector<Plane *> proj_plane_list,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &cor3d_cloud,
                      std::vector<BinaryDescriptor> &binary_descriptor_list);


void binary_extractor_debug(
    const ConfigSetting &config_setting,
    const std::vector<Plane *> proj_plane_list,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::vector<BinaryDescriptor> &binary_descriptor_list,
    std::vector<BinaryDescriptor> &binary_descriptor_around_list);

void extract_binary(const ConfigSetting &config_setting,
                    const Eigen::Vector3d &project_center,
                    const Eigen::Vector3d &project_normal,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                    std::vector<BinaryDescriptor> &binary_list);

void extract_binary_cor3d(const ConfigSetting &config_setting,
                    const Eigen::Vector3d &project_center,
                    const Eigen::Vector3d &project_normal,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cor3d_cloud,
                    std::vector<BinaryDescriptor> &binary_list,
                    std::vector<BinaryDescriptor> &binary_list_cor3d);


void extract_binary_debug(
    const ConfigSetting &config_setting, const Eigen::Vector3d &project_center,
    const Eigen::Vector3d &project_normal,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::vector<BinaryDescriptor> &binary_list,
    std::vector<BinaryDescriptor> &binary_around_list);

void extract_binary_all(const ConfigSetting &config_setting,
                        const Eigen::Vector3d &project_center,
                        const Eigen::Vector3d &project_normal,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                        std::vector<BinaryDescriptor> &binary_list,
                        cv::Mat &binary_image);

float generate_std(const ConfigSetting &config_setting,
                  const std::vector<BinaryDescriptor> &binary_list,
                  const int &frame_id, std::vector<STD> &std_list);

void candidate_searcher(const ConfigSetting &config_setting,
    std::unordered_map<STD_LOC, std::vector<STD>> &descriptor_map,
    std::vector<STD> &current_STD_list,
    std::vector<STDMatchList> &alternative_match, std::fstream &debug_file);

void rgb_candidate_searcher(
    const ConfigSetting &config_setting,
    std::unordered_map<STD_LOC, std::vector<STD>> &descriptor_map,
    std::unordered_map<STD_LOC, std::vector<STD>> &descriptor_map_brf,
    std::vector<STD> &current_STD_list,
    std::vector<STDMatchList> &alternative_match,
    std::vector<std::vector<__m128i>> &all_briefs,
    std::fstream & debug_file);

void candidate_searcher_old(
    const ConfigSetting &config_setting,
    std::unordered_map<STD_LOC, std::vector<STD>> &descriptor_map,
    std::vector<STD> &current_STD_list,
    std::vector<STDMatchList> &alternative_match);

void triangle_solver(std::pair<STD, STD> &std_pair, Eigen::Matrix3d &std_rot,
                     Eigen::Vector3d &std_t);

void fine_loop_detection(const ConfigSetting &config_setting,
                         std::vector<std::pair<STD, STD>> &match_list,
                         bool &fine_sucess, Eigen::Matrix3d &std_rot,
                         Eigen::Vector3d &std_t,
                         std::vector<std::pair<STD, STD>> &sucess_match_list,
                         std::vector<std::pair<STD, STD>> &unsucess_match_list);

void fine_loop_detection_tbb(const ConfigSetting &config_setting,
    std::vector<std::pair<STD, STD>> &match_list, bool &fine_sucess,
    Eigen::Matrix3d &std_rot, Eigen::Vector3d &std_t,
    std::vector<std::pair<STD, STD>> &sucess_match_list,
    std::vector<std::pair<STD, STD>> &unsucess_match_list);

void rgb_fine_loop_detection_tbb(
    const ConfigSetting &config_setting,
    std::vector<std::pair<STD, STD>> &match_list, bool &fine_sucess,
    Eigen::Matrix3d &std_rot, Eigen::Vector3d &std_t,
    std::vector<std::pair<STD, STD>> &sucess_match_list,
    std::vector<std::pair<STD, STD>> &unsucess_match_list, bool &is_rgb_case,
    const int &match_frame, std::fstream & debug_file);

double
rgb_geometric_verify(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &source_cloud,
                     const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target_cloud,
                     const Eigen::Matrix3d &rot, const Eigen::Vector3d &t);

double
geometric_verify(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
                 const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
                 const Eigen::Matrix3d &rot, const Eigen::Vector3d &t);

double
geometric_verify(const ConfigSetting &config_setting,
                 const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
                 const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
                 const Eigen::Matrix3d &rot, const Eigen::Vector3d &t);

double geometric_icp(const ConfigSetting &config_setting,
                     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud,
                     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
                     Eigen::Matrix3d &rot, Eigen::Vector3d &t);

void add_STD(std::unordered_map<STD_LOC, std::vector<STD>> &descriptor_map,
             std::vector<STD> &STD_list);
void add_STD_brf(std::unordered_map<STD_LOC, std::vector<STD>> &descriptor_map,
             std::vector<STD> &STD_list);

void publish_std(const std::vector<std::pair<STD, STD>> &match_std_list,
                 const ros::Publisher &std_publisher,
                 const Eigen::Vector3d &color);

void publish_std_list(const std::vector<STD> &std_list,
                      const ros::Publisher &std_publisher,
                      const Eigen::Vector3d &color);

void publish_binary(const std::vector<BinaryDescriptor> &binary_list,
                    const Eigen::Vector3d &text_color,
                    const std::string &text_ns,
                    const ros::Publisher &text_publisher);

double
calc_triangle_dis(const std::vector<std::pair<STD, STD>> &match_std_list);
double
calc_binary_similaity(const std::vector<std::pair<STD, STD>> &match_std_list);

//double
//calc_brief_similaity(const std::vector<std::pair<STD, STD>> &match_std_list, std::fstream & debug_file);

double calc_overlap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud1,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud2,
                    double dis_threshold);

double calc_overlap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud1,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud2,
                    double dis_threshold, int skip_num);

double calc_overlap(const pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr &cloud1_kd_tree,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud2,
                    const cloud_bounding_box &cbx1,
                    const cloud_bounding_box &cbx2,
                    int cloud1_size,
                    double dis_threshold, int skip_num);

void check_record_gtloopclosure(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_cens,
                                const Eigen::Vector3d &posi_now,
                                const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &key_cloud_list,
                                const std::vector<cloud_bounding_box> &cbx_vec,
                                const int &key_frame_id,
                                const double &range_gt_search,
                                const ConfigSetting &config_setting,
                                std::fstream &debug_file,
                                std::fstream &gt_doc_file);

void record_proposed_loopclosure(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &key_cloud_list,
                                 const std::vector<cloud_bounding_box> &cbx_vec,
                                 const int &key_frame_id,
                                 const int &match_frame,
                                 const double &best_score,
                                 const ConfigSetting &config_setting,
                                 std::fstream &debug_file,
                                 std::fstream &gt_doc_file);

void record_topN_loopclosure(const int &key_frame_id, std::vector<Eigen::Vector3d> &posi_now,
                             std::vector<std::pair<int, float> > &proposed_candidates, int N,
                             std::fstream &gt_doc_file);

void CalcQuation(const Eigen::Vector3d &vec, const int axis,
                 geometry_msgs::Quaternion &q);

void pubPlane(const ros::Publisher &plane_pub, const std::string plane_ns,
              const int plane_id, const pcl::PointXYZINormal normal_p,
              const float radius, const Eigen::Vector3d rgb);

pcl::PointXYZI vec2point(const Eigen::Vector3d &vec);
Eigen::Vector3d point2vec(const pcl::PointXYZI &pi);

Eigen::Vector3d normal2vec(const pcl::PointXYZINormal &pi);

template <typename T> Eigen::Vector3d point2vec(const T &pi) {
  Eigen::Vector3d vec(pi.x, pi.y, pi.z);
  return vec;
}

double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin);

void load_array(std::vector<bool> &occupy_array_, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_to_load, const int i);
void save_an_array(const BinaryDescriptor& bdes,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_to_store);

void save_descriptor(const std::vector<STD> &current_descriptor,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_to_store);
void load_descriptor(std::unordered_map<STD_LOC, std::vector<STD>>  &feat_map,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_to_load, int &index_max);

template<typename T> struct greaterThanPtr
{
    bool operator()(const T* a, const T* b) const { return *a > *b; }
};

struct kpt_2d_info
{
    kpt_2d_info()
    {
        depth = -1.0f;
        img_time = 0.0f;
    };
    float depth;
    cv::Point2f pt_2d;
    Eigen::Vector3d pt_3d;
    double img_time;
    Eigen::Quaterniond c_q_w;
    Eigen::Vector3d c_t_w;
    std::vector<__m128i> brief_array;
    float Rval;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// ***Adapt from Opencv
void calcHarris_mine( const cv::Mat& _cov, cv::Mat& _dst, double k );

// ***Adapt from Opencv
void goodFeaturesToTrack_opt( const cv::Mat& image, cv::Mat& eig, cv::Mat& tmp, cv::Mat& cov, cv::Mat& Dx, cv::Mat& Dy,
                              cv::OutputArray &_corners,
                              int maxCorners, double qualityLevel, double minDistance,
                              const cv::Mat& mask, int blockSize,
                              bool useHarrisDetector, double harrisK,
                              std::vector<float> &harrisR );
// ***Adapt from Opencv
void cornerEigenValsVecs_mine( const cv::Mat& src, cv::Mat& eigenv, cv::Mat &cov, cv::Mat& Dx, cv::Mat& Dy, int block_size,
                               int aperture_size, double k=0.,
                               int borderType=cv::BORDER_DEFAULT );

void extract_2dkeypt_roughly(cv::Mat& eig, cv::Mat& eig_tmp, const cv::Mat& img_grey_ds, cv::Mat& cov, cv::Mat& Dx, cv::Mat& Dy,
                             const cv::Mat& mask, std::vector<cv::Point2f>& corners,
                             std::vector<cv::Point2f> &corners_out, std::vector<float> &harrisR, const int &layer_num);

std::pair<float, float> calc_grad_xy(const cv::Mat &image, int r, int c);

void find_surrounding_max_R_pixel(const cv::Mat &image, const std::vector<cv::Point2f> &corners,
                                  std::vector<cv::Point2f> &corners_out,
                                  int blockSize, int layer_num);

void find_more_accurate_pixel(const cv::Mat& img_cur, const cv::Mat& img_grey, std::vector<cv::Point2f>& corners,
                              std::vector<cv::Point2f> &corners_out, const int &layer_num, const std::string &filename);

// ***Adapt from Opencv
inline int smoothedSum(const cv::Mat& sum, const cv::KeyPoint& pt, int y, int x, bool use_orientation, cv::Matx21f R)
{
    static const int HALF_KERNEL = /*BriefDescriptorExtractorImpl::KERNEL_SIZE*/9 / 2;

    if ( use_orientation )
    {
      int rx = (int)(((float)x)*R(1,0) - ((float)y)*R(0,0));
      int ry = (int)(((float)x)*R(0,0) + ((float)y)*R(1,0));
      if (rx > 24) rx = 24;
      if (rx < -24) rx = -24;
      if (ry > 24) ry = 24;
      if (ry < -24) ry = -24;
      x = rx; y = ry;
    }
    const int img_y = (int)(pt.pt.y + 0.5) + y;
    const int img_x = (int)(pt.pt.x + 0.5) + x;
    return   sum.at<int>(img_y + HALF_KERNEL + 1, img_x + HALF_KERNEL + 1)
           - sum.at<int>(img_y + HALF_KERNEL + 1, img_x - HALF_KERNEL)
           - sum.at<int>(img_y - HALF_KERNEL, img_x + HALF_KERNEL + 1)
           + sum.at<int>(img_y - HALF_KERNEL, img_x - HALF_KERNEL);
}
// ***Adapt from Opencv
static void pixelTests64(cv::Mat &sum, const std::vector<cv::KeyPoint>& keypoints, cv::Mat &descriptors, bool use_orientation)
{
    cv::Matx21f R;
    //cv::Mat descriptors = _descriptors.getMat();
    std::vector<size_t> index;   size_t i = 0;
    for (; i < keypoints.size(); )
    {
        index.push_back(i);
        i++;
    }
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i)
   // for (size_t i = 0; i < keypoints.size(); ++i)
    {
        uchar* desc = descriptors.ptr(static_cast<int>(i));
        const cv::KeyPoint& pt = keypoints[i];
        if ( use_orientation )
        {
          float angle = pt.angle;
          angle *= (float)(CV_PI/180.f);
          R(0,0) = sin(angle);
          R(1,0) = cos(angle);
        }
#include "generated_64.i"
    }
    );
}

void integral_mine(unsigned char *Src, int *Integral, int Width, int Height, int Stride);

void extract_brief_fromimage(std::vector<cv::Point2f> &corners, cv::Mat &grayImage, cv::Mat &sum,
                             cv::Mat &descriptors, std::vector<std::vector<__m128i>> &brief_arrays);


void generate_depthimg(cv::Mat &depth_map, const pcl::PointCloud<pcl::PointXYZI>::Ptr &ptcloud,
                       const Eigen::Quaterniond c_q_w, const Eigen::Vector3d c_t_w, const Eigen::Vector3d w_t_c,
                       const int &IMG_HEI, const int &IMG_WID, const float &fx, const float &fy, const float &cx, const float &cy);

void generatef_depthmask(int depth_vec[], const pcl::PointCloud<pcl::PointXYZI>::Ptr &ptcloud,
                         const Eigen::Quaterniond c_q_w, const Eigen::Vector3d c_t_w, const Eigen::Vector3d w_t_c,
                         const int &IMG_HEI, const int &IMG_WID, const float &fx, const float &fy, const float &cx, const float &cy);

void project_onimage(std::vector<Eigen::Vector3d> &projected_2dpts, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &ptcloud,
                     Eigen::Quaterniond c_q_w, Eigen::Vector3d c_t_w, Eigen::Vector3d w_t_c,
                     const int &IMG_HEI, const int &IMG_WID, const float &fx, const float &fy, const float &cx, const float &cy);

float findf_surrounding_depth(int r_cen, int c_cen, int depth_vec[], int range, int &r_cen_dismin, int &c_cen_dismin,
                              const std::vector<std::tuple<int, int, float>> &search_row_col,
                              const int &IMG_HEI, const int &IMG_WID);

void find3dpts_for2dcorners(std::vector<std::vector<kpt_2d_info>> &kpts_from_img_list,
                              //std::vector<std::vector<float>> &depth_mask_list,
                              std::vector<int*> &depthf_mask_list,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud,
                              const std::vector<double> &image_times,
                              std::map<std::string, imu_lidar_camera>& sorter,
                              const std::vector<size_t> &index,
                              const std::vector<std::tuple<int, int, float>> &search_row_col,
                              const int &IMG_HEI, const int &IMG_WID, const float &fx, const float &fy, const float &cx, const float &cy,
                              const cv::String &save_directory);

inline float calc_similarity(const std::vector<__m128i> &b1, const  std::vector<__m128i> &b2)
{
    return count_equal(b1, b2, 256/16);
}
inline float calc_brief_hamming_dis(const std::vector<__m128i> &b1, const std::vector<__m128i> &b2)
{
    return b1.size() - calc_similarity(b1,b2);
}

void non_maximal_suppression(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cor3d_cloud_out,
                             std::vector<std::vector<__m128i>> &out_brief_list,
                             std::vector<std::vector<kpt_2d_info>> &kpts_from_img_list,
                             const std::vector<size_t> &index,
                             const ConfigSetting &config_setting);
