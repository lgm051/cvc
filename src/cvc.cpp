#include "cvc.h"


CvcCluster::CvcCluster(ros::NodeHandle &nh)
{
    sub_point_cloud_ = nh.subscribe("/velodyne_points", 10, &CvcCluster::callback, this);

    pub_cvc_cluster_ = nh.advertise<sensor_msgs::PointCloud2>("/cluster_points", 10);

    ros::spin();
}

CvcCluster::~CvcCluster() {}

template <typename PointT>
void CvcCluster::calculateAPR(const pcl::PointCloud<PointT>& cloud_IN, vector<PointAPR>& vapr){
    for (int i =0; i<cloud_IN.points.size(); ++i){
        PointAPR par;
        par.polar_angle = Polar_angle_cal(cloud_IN.points[i].x, cloud_IN.points[i].y);
        par.range = sqrt(cloud_IN.points[i].x*cloud_IN.points[i].x+cloud_IN.points[i].y*cloud_IN.points[i].y);
        par.azimuth =(float) atan2(cloud_IN.points[i].z,par.range);
        if(par.azimuth < minazimuth){
            minazimuth = par.azimuth;
        }
        if(par.azimuth > maxazimuth){
            maxazimuth = par.azimuth;
        }
        if(par.range < minrange){
            minrange = par.range;
        }
        if(par.range > maxrange){
            maxrange = par.range;
        }
        vapr.push_back(par);
    }

    length = round((maxrange - minrange)/deltaR);
    width = 301;
    height = round((maxazimuth - minazimuth)/deltaA);

}

void CvcCluster::build_hash_table(const vector<PointAPR>& vapr, unordered_map<int, Voxel> &map_out){
    vector<int> ri;
    vector<int> pi;
    vector<int> ai;
    for(int i =0; i< vapr.size(); ++i){
        int azimuth_index = round(((vapr[i].azimuth-minazimuth)*180/PI)/deltaA);
        int polar_index = round(vapr[i].polar_angle*180/PI/deltaP);
        int range_index = round((vapr[i].range-minrange)/deltaR);
        int voxel_index = (polar_index*(length+1)+range_index)+azimuth_index*(length+1)*(width+1);
        ri.push_back(range_index);
        pi.push_back(polar_index);
        ai.push_back(azimuth_index);
        unordered_map<int, Voxel>::iterator it_find;
        it_find = map_out.find(voxel_index);
        if (it_find != map_out.end()){
            it_find->second.index.push_back(i);

        }else{
            Voxel vox;
            vox.haspoint =true;
            vox.index.push_back(i);
            vox.index.swap(vox.index);
            map_out.insert(make_pair(voxel_index,vox));
        }

    }
    auto maxPosition = max_element(ai.begin(), ai.end());
    auto maxPosition1 = max_element(ri.begin(), ri.end());
    auto maxPosition2 = max_element(pi.begin(), pi.end());
    cout<<*maxPosition<<" "<<*maxPosition1<<" "<<*maxPosition2<<endl;

}


void CvcCluster::find_neighbors(int polar, int range, int azimuth, vector<int>& neighborindex){
    for (int z = azimuth - 1; z <= azimuth + 1; z++){
        if (z < 0 || z >round((maxazimuth-minazimuth)*180/PI/deltaA)){
            continue;
        }

        for (int y = range - 1; y <= range + 1; y++){
            if (y < 0 || y >round((50-minrange)/deltaR)){
                continue;
            }

            for (int x = polar - 1; x <= polar + 1; x++){
                int px = x;
                if (x < 0 ){
                    px=300;
                }
                if(x>300){
                    px=0;

                }

                neighborindex.push_back((px*(length+1)+y)+z*(length+1)*(width+1));
            }
        }
    }
}

bool compare_cluster(pair<int,int> a,pair<int,int> b){
    return a.second>b.second;
}
bool CvcCluster::most_frequent_value(vector<int> values, vector<int> &cluster_index) {
    unordered_map<int, int> histcounts;
    for (int i = 0; i < values.size(); i++) {
        if (histcounts.find(values[i]) == histcounts.end()) {
            histcounts[values[i]] = 1;
        }
        else {
            histcounts[values[i]] += 1;
        }
    }

    int max = 0, maxi;
    vector<pair<int, int>> tr(histcounts.begin(), histcounts.end());
    sort(tr.begin(),tr.end(),compare_cluster);
    for(int i = 0 ; i< tr.size(); ++i){
        if(tr[i].second>10){
            cluster_index.push_back(tr[i].first);
        }
    }

    return true;
}

void CvcCluster::mergeClusters(vector<int>& cluster_indices, int idx1, int idx2) {
    for (int i = 0; i < cluster_indices.size(); i++) {
        if (cluster_indices[i] == idx1) {
            cluster_indices[i] = idx2;
        }
    }
}

vector<int>  CvcCluster::CVC(unordered_map<int, Voxel> &map_in,const vector<PointAPR>& vapr){
    int current_cluster = 0;
    cout<<"CVC"<<endl;
    vector<int> cluster_indices = vector<int>(vapr.size(), -1);

    for(int i = 0; i< vapr.size(); ++i){

        if (cluster_indices[i] != -1)
            continue;
        int azimuth_index = round((vapr[i].azimuth+fabs(minazimuth))*180/PI/deltaA);
        int polar_index = round(vapr[i].polar_angle*180/PI/deltaP);
        int range_index = round((vapr[i].range-minrange)/deltaR);
        int voxel_index = (polar_index*(length+1)+range_index)+azimuth_index*(length+1)*(width+1);

        unordered_map<int, Voxel>::iterator it_find;
        unordered_map<int, Voxel>::iterator it_find2;

        it_find = map_in.find(voxel_index);
        vector<int> neightbors;

        if (it_find != map_in.end()){

            vector<int> neighborid;
            find_neighbors(polar_index, range_index, azimuth_index, neighborid);
            for (int k =0; k<neighborid.size(); ++k){

                it_find2 = map_in.find(neighborid[k]);

                if (it_find2 != map_in.end()){

                    for(int j =0 ; j<it_find2->second.index.size(); ++j){
                        neightbors.push_back(it_find2->second.index[j]);
                    }
                }
            }
        }

        neightbors.swap(neightbors);

        if(neightbors.size()>0){
            for(int j =0 ; j<neightbors.size(); ++j){
                int oc = cluster_indices[i] ;
                int nc = cluster_indices[neightbors[j]];
                if (oc != -1 && nc != -1) {
                    if (oc != nc)
                        mergeClusters(cluster_indices, oc, nc);
                }
                else {
                    if (nc != -1) {
                        cluster_indices[i] = nc;
                    }
                    else {
                        if (oc != -1) {
                            cluster_indices[neightbors[j]] = oc;
                        }
                    }
                }

            }
        }

        if (cluster_indices[i] == -1) {
            current_cluster++;
            cluster_indices[i] = current_cluster;
            for(int s =0 ; s<neightbors.size(); ++s){
                cluster_indices[neightbors[s]] = current_cluster;
            }
        }


    }
    return cluster_indices;
}

vector<float> CvcCluster::hsv2rgb(vector<float>& hsv){
    vector<float> rgb(3);
    float R,G,B,H,S,V;
    H = hsv[0];
    S = hsv[1];
    V = hsv[2];
    if(S == 0){
        rgb[0]=rgb[1]=rgb[2]=V;
    }else{

        int i = int(H*6);
        float f = (H*6) - i;
        float a = V * ( 1 - S );
        float b = V * ( 1 - S * f );
        float c = V * ( 1 - S * (1 - f ) );
        i = i%6;
        switch(i){
            case 0: {rgb[0] = V; rgb[1]= c; rgb[2] = a; break;}
            case 1: {rgb[0] = b; rgb[1] = V; rgb[2] = a;break;}
            case 2: {rgb[0] = a; rgb[1] = V; rgb[2] = c;break;}
            case 3: {rgb[0] = a; rgb[1] = b; rgb[2] = V;break;}
            case 4: {rgb[0] = c; rgb[1] = a; rgb[2] = V;break;}
            case 5: {rgb[0] = V; rgb[1] = a; rgb[2] = b;break;}
        }
    }

    return rgb;
}

void CvcCluster::callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*in_cloud, *cloud_gr);

    vector<PointAPR> papr;
    calculateAPR(*cloud_gr,papr);
    unordered_map<int, Voxel> hvoxel;
    build_hash_table(papr,hvoxel);
    vector<int> cluster_index = CVC(hvoxel,papr);
    vector<int> cluster_id;
    most_frequent_value(cluster_index, cluster_id);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
    cout<<"front: " << cluster_id.front() <<"!" << endl;
cout<<"data: " << cluster_id.size() <<"!" << endl;
//    for (int i = 0; i < cluster_index.size(); i++)
//    {
/*        for (std::vector<int>::const_iterator pit = cluster_index.begin(); pit != cluster_index.end(); ++pit)
            cluster->points.push_back(cloud_gr->points[*pit]);*/

for (int k = 0; k < cluster_id.size(); ++k) {
            for (int j = 0; j < cluster_index.size(); ++j) {
                if (cluster_index[j] == cluster_id[k]) {

                    cluster->points.push_back(cloud_gr->points[j]);
   		    
                }
            }
        }

    cluster->width = cluster->points.size ();
    cluster->height = 1;
    cluster->is_dense = true;

//    clusters.push_back(cluster);
//    }
    cluster->header = cloud_gr->header;
    sensor_msgs::PointCloud2 pub_point;
    pcl::toROSMsg(*cluster, pub_point);
    pub_cvc_cluster_.publish(pub_point);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cvc_test");

    ros::NodeHandle nh;

    CvcCluster core(nh);

    return 0;
}
