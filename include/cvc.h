#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <unordered_map>
#include <algorithm>
#include<pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


using namespace std;
const float PI = 3.1415926;
bool compare_cluster(pair<int,int> a,pair<int,int> b);
class CvcCluster{
private:


    template<typename T> string toString(const T& t) {
        ostringstream oss;
        oss << t;
        return oss.str();
    }
    static int64_t gtm() {
        struct timeval tm;
        gettimeofday(&tm, 0);
        int64_t re = (((int64_t) tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
        return re;
    }

    float Polar_angle_cal(float x, float y){
        float temp_tangle = 0;
        if(x== 0 && y ==0){
            temp_tangle = 0;
        }else if(y>=0){
            temp_tangle = (float)atan2(y,x);
        }else if(y<=0){
            temp_tangle = (float)atan2(y,x)+2*PI;
        }
        return temp_tangle;
    }

    struct PointAPR{
        float azimuth;
        float polar_angle;
        float range;
    };

    struct Voxel{
        bool haspoint = false;
        int cluster = -1;
        vector<int> index;
    };

    float minrange = 3;
    float maxrange = 3;
    float minazimuth = 0;
    float maxazimuth = 0;
    float deltaA = 2;
    float deltaR = 0.35;
    float deltaP = 1.2;
    int length = 0;
    int width = 0;
    int height = 0;


    ros::Subscriber sub_point_cloud_;
    ros::Publisher pub_cvc_cluster_;

public:

    template <typename PointT>
    void calculateAPR(const pcl::PointCloud<PointT>& cloud_IN, vector<PointAPR>& vapr);
    void build_hash_table(const vector<PointAPR>& vapr, unordered_map<int, Voxel> &map_out);
    void find_neighbors(int polar, int range, int azimuth, vector<int>& neighborindex);
    bool most_frequent_value(vector<int> values, vector<int> &cluster_index);
    void mergeClusters(vector<int>& cluster_indices, int idx1, int idx2);
    vector<int>  CVC(unordered_map<int, Voxel> &map_in,const vector<PointAPR>& vapr);
    vector<float> hsv2rgb(vector<float>& hsv);


    void callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud);
    CvcCluster(ros::NodeHandle &nh);
    ~CvcCluster();

};

