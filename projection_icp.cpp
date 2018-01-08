#include <ros/ros.h>
#include <ros/package.h>


#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <stdlib.h>
#include <sstream>
#include <stdio.h>

// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include <pcl_ros/transforms.h>
#include <tf_conversions/tf_eigen.h>

#include <ar_track_alvar_msgs/AlvarCorners.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <vtkRenderWindow.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <ros/ros.h>
#include <ros/package.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <vtkRenderWindow.h>

#include <stdlib.h>
#include <sstream>
#include <stdio.h>

#include "boost/filesystem.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

using namespace std;
//ros::Rate rate(10);
typedef pcl::PointXYZ PointT;

pcl::PointCloud<PointT>::Ptr createPlaneCornersFromCenters(pcl::PointCloud<PointT>::Ptr centers, pcl::PointCloud<PointT>::Ptr plane);


tf::StampedTransform tf_tag_0_3d;
boost::mutex mut_0_3d;
bool found_tag_0_3d = false;

Eigen::MatrixXf tags_matrix;
cv::Mat projection_matrix(3,3, CV_32F);
cv::Mat distortion_matrix(1,5, CV_32F);

bool cam_info_received = false;
bool find_pose_approx = false;
bool cam_approximation_received = false;

vector<int> tag_ids;
ar_track_alvar_msgs::AlvarMarkers markers;
boost::mutex markers_mutex;

cv::Mat rvec, tvec, null_matrix;

int NUMBER_MARKERS_USED = 18;

pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
boost::mutex cloud_mutex;

geometry_msgs::PoseStamped ref_pose;

boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerX;

tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
    Eigen::Matrix4f mf = trans; //The matrix I want to convert
    Eigen::Matrix4d md(mf.cast<double>());
    Eigen::Affine3d affine(md);
    tf::Transform transform_;
    tf::transformEigenToTF(affine,transform_);
    tf::Quaternion test = transform_.getRotation().normalize();
    transform_.setRotation(test);
    return transform_;
}

void initTagsMatrix(){

    tags_matrix.setZero(NUMBER_MARKERS_USED, 4); // 8 tags in homogeneous coordinates

    // tag 0
    if(NUMBER_MARKERS_USED > 0){
        tags_matrix(0,0) = 0; // x
        tags_matrix(0,1) = 0; // y
        tags_matrix(0,3) = 1;
    }
    // tag 1
    if(NUMBER_MARKERS_USED > 1){
        tags_matrix(1,0) = 0;
        tags_matrix(1,1) = -0.2765;
        tags_matrix(1,3) = 1;
    }
    // tag 2
    if(NUMBER_MARKERS_USED > 2){
        tags_matrix(2,0) = 0;
        tags_matrix(2,1) = -0.5530;
        tags_matrix(2,3) = 1;
    }
    // tag 3
    if(NUMBER_MARKERS_USED > 3){
        tags_matrix(3,0) = -0.2020;
        tags_matrix(3,1) = -0.5530;
        tags_matrix(3,3) = 1;
    }
    // tag 4
    if(NUMBER_MARKERS_USED > 4){
        tags_matrix(4,0) = -0.4040;
        tags_matrix(4,1) = -0.5530;
        tags_matrix(4,3) = 1;
    }
    // tag 5
    if(NUMBER_MARKERS_USED > 5){
        tags_matrix(5,0) = -0.4040;
        tags_matrix(5,1) = -0.2765;
        tags_matrix(5,3) = 1;
    }
    // tag 6
    if(NUMBER_MARKERS_USED > 6){
        tags_matrix(6,0) = -0.4040;
        tags_matrix(6,1) = 0;
        tags_matrix(6,3) = 1;
    }
    // tag 7
    if(NUMBER_MARKERS_USED > 7){
        tags_matrix(7,0) = -0.2020;
        tags_matrix(7,1) = 0;
        tags_matrix(7,3) = 1;
    }
    // tag 8
    if(NUMBER_MARKERS_USED > 8){
        tags_matrix(8,0) = 0; // x
        tags_matrix(8,1) = -0.2765 / 2; // y
        tags_matrix(8,3) = 1;
    }
    // tag 9
    if(NUMBER_MARKERS_USED > 9){
        tags_matrix(9,0) = 0; // x
        tags_matrix(9,1) = -0.2765 - 0.2765 / 2; // y
        tags_matrix(9,3) = 1;
    }
    // tag 10
    if(NUMBER_MARKERS_USED > 10){
        tags_matrix(10,0) = -0.2020 / 2; // x
        tags_matrix(10,1) = -0.5530; // y
        tags_matrix(10,3) = 1;
    }
    // tag 11
    if(NUMBER_MARKERS_USED > 11){
        tags_matrix(11,0) = -0.2020 - 0.2020 / 2; // x
        tags_matrix(11,1) = -0.5530; // y
        tags_matrix(11,3) = 1;
    }
    // tag 12
    if(NUMBER_MARKERS_USED > 12){
        tags_matrix(12,0) = -0.4040; // x
        tags_matrix(12,1) = -0.2765 - 0.2765 / 2; // y
        tags_matrix(12,3) = 1;
    }
    // tag 13
    if(NUMBER_MARKERS_USED > 13){
        tags_matrix(13,0) = -0.4040; // x
        tags_matrix(13,1) = -0.2765 / 2; // y
        tags_matrix(13,3) = 1;
    }
    // tag 14
    if(NUMBER_MARKERS_USED > 14){
        tags_matrix(14,0) = -0.2020 - 0.2020 / 2; // x
        tags_matrix(14,1) = 0; // y
        tags_matrix(14,3) = 1;
    }
    // tag 15
    if(NUMBER_MARKERS_USED > 15){
        tags_matrix(15,0) = -0.2020 / 2; // x
        tags_matrix(15,1) = 0; // y
        tags_matrix(15,3) = 1;
    }

    // tag 16
    if(NUMBER_MARKERS_USED > 16){
        tags_matrix(16,0) = -0.2020 / 2; // x
        tags_matrix(16,1) = -0.2765 - 0.2765 / 2; // y
        tags_matrix(16,3) = 1;
    }

    // tag 17
    if(NUMBER_MARKERS_USED > 17){
        tags_matrix(17,0) = -0.2020 - 0.2020 / 2; // x
        tags_matrix(17,1) =  -0.2765 - 0.2765 / 2; // y
        tags_matrix(17,3) = 1;
    }
    //tags_matrix = tags_matrix.transpose();

    cout << "Tags Matrix : " << endl << tags_matrix << endl;
}

vector<cv::Point3f> getRealWorldCoordinates(vector<int> indices){

    vector<cv::Point3f> pts_vec;

    for(int i = 0 ; i < indices.size(); i++){

        int id = indices.at(i);
        double x = tags_matrix(id, 0);
        double y = tags_matrix(id, 1);
        double z = tags_matrix(id, 2); // 0 (all on the same plane)
        double dist = 0.0445 / 2;

        cv::Point3f p;
        p.z = 0;

        // counter clockwise from top left corners and ending with center
        //        p.x = x + dist;
        //        p.y = y + dist;
        //        pts_vec.push_back(p);

        //        p.x = x - dist;
        //        p.y = y + dist;
        //        pts_vec.push_back(p);

        //        p.x = x - dist;
        //        p.y = y - dist;
        //        pts_vec.push_back(p);

        //        p.x = x + dist;
        //        p.y = y - dist;
        //        pts_vec.push_back(p);

        p.x = x;
        p.y = y;
        pts_vec.push_back(p);
    }


    return pts_vec;

}



void arMarkersCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& msg){

    markers_mutex.lock();
    markers = *msg;
    markers_mutex.unlock();
}

void cloud_callback (const pcl::PCLPointCloud2ConstPtr& input){

    cloud_mutex.lock();
    pcl::fromPCLPointCloud2(*input,*cloud);
    cloud_mutex.unlock();
}

void publisher_thread(){

    ros::Rate r(10);
    static tf::TransformBroadcaster broadcaster;

    while(ros::ok()){

        markers_mutex.lock();
        ar_track_alvar_msgs::AlvarMarkers local_markers = markers;
        markers_mutex.unlock();

        if(local_markers.markers.size() > 0){

            cout << "Got markers! " << endl;

            std::vector<int> id_vector(NUMBER_MARKERS_USED, 0);
            std::vector<int> id_ordered;

            // tags coordinates in camera frame
            vector<cv::Point3f> corners_tags_vec;

            // Check if the tags belong to at least 2 different lines
            bool line1 = false ;
            bool line2 = false ;
            bool line3 = false ;
            bool line4 = false ;

            for(int i = 0; i < local_markers.markers.size(); i++){
                ar_track_alvar_msgs::AlvarMarker tag = local_markers.markers.at(i);
                if(tag.id >= 0 && tag.id < NUMBER_MARKERS_USED){
                    ar_track_alvar_msgs::AlvarMarker tag = local_markers.markers.at(i);

                    geometry_msgs::Point pt = tag.pose.pose.position;
                    corners_tags_vec.push_back(cv::Point3f(pt.x, pt.y, pt.z) );

                    id_ordered.push_back(tag.id);
                    id_vector.at(tag.id) = 1;

                    ref_pose = tag.pose;

                    int id = tag.id;
                    if(id == 1 || id == 8 || id == 9) line1 = true;
                    if(id == 3 || id == 10 || id == 11) line2 = true;
                    if(id == 5 || id == 12 || id == 13) line3 = true;
                    if(id == 7 || id == 14 || id == 15) line4 = true;
                }
            }


            // tag coordinates in reference frame
            vector<cv::Point3f> corners_real_vec = getRealWorldCoordinates(id_ordered);
            int lines = 0;
            if(line1) lines++;
            if(line2) lines++;
            if(line3) lines++;
            if(line4) lines++;

            int size1 = corners_real_vec.size();
            int size2 = corners_tags_vec.size();
            if(size1 > 2 && size2 > 2 && size1 == size2 && lines > 1){

                // Find main plane
                pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
                cloud_mutex.lock();
                *scene = *cloud;
                cloud_mutex.unlock();

                pcl::PassThrough<PointT> pass_filter;
                pass_filter.setFilterFieldName("z");
                pass_filter.setFilterLimits(0.3, 1.5);
                pass_filter.setInputCloud(scene); // To see workspace
                pass_filter.filter(*scene);

                // Filter point cloud
                pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
                pcl::VoxelGrid<PointT> grid;
                float leaf = 0.005;
                grid.setLeafSize (leaf, leaf, leaf);
                grid.setInputCloud (scene);
                grid.filter (*scene);


                // Find main plane coefficients (RANSAC) from a filtered point cloud (faster)
                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
                pcl::SACSegmentation<PointT> segmentation;
                segmentation.setInputCloud(scene);
                segmentation.setModelType(pcl::SACMODEL_PLANE);
                segmentation.setMethodType(pcl::SAC_RANSAC);
                segmentation.setDistanceThreshold(0.01);
                segmentation.setOptimizeCoefficients(true);
                segmentation.segment(*inlierIndices, *coefficients);

                pcl::ExtractIndices<PointT> extract;
                extract.setInputCloud (scene);
                extract.setIndices (inlierIndices);
                extract.setNegative (false);
                extract.filter (*filtered_cloud);


                pcl::PointCloud<PointT>::Ptr detected_tags_pc(new pcl::PointCloud<PointT>);
                pcl::PointCloud<PointT>::Ptr real_pc(new pcl::PointCloud<PointT>);
                pcl::Correspondences correspondences;

                // Fill point correspondences
                for(int i = 0; i < size1; i++){
                    pcl::Correspondence corr;
                    corr.index_match = i;
                    corr.index_query = i;
                    correspondences.push_back(corr);

                    PointT pcl_pt;
                    cv::Point3f cv_pt = corners_tags_vec.at(i);
                    pcl_pt.x = cv_pt.x;
                    pcl_pt.y = cv_pt.y;
                    pcl_pt.z = cv_pt.z;

                    detected_tags_pc->points.push_back(pcl_pt);

                    cv_pt = corners_real_vec.at(i);
                    pcl_pt.x = cv_pt.x;
                    pcl_pt.y = cv_pt.y;
                    pcl_pt.z = cv_pt.z;
                    real_pc->points.push_back(pcl_pt);
                }

                // Show in viewer

                //pclViewer->removeAllPointClouds();

                // pclViewer->updatePointCloud()


                // Filter points not on the main plane of the scene
//                pcl::PointCloud<PointT>::Ptr detected_tags_plane(new pcl::PointCloud<PointT>);
//                pcl::PointCloud<PointT>::Ptr real_plane(new pcl::PointCloud<PointT>);

//                pcl::SampleConsensusModelPlane<PointT>::Ptr dit (new pcl::SampleConsensusModelPlane<PointT> (detected_tags_pc));
//                std::vector<int> inliers;
//                Eigen::Vector4f coeff = Eigen::Vector4f(coefficients->values.at(0),coefficients->values.at(1),coefficients->values.at(2),coefficients->values.at(3));
//                //cout << "coeff : " << coeff << endl;
//                dit -> selectWithinDistance (coeff, 0.03, inliers);
//                pcl::PointIndices::Ptr plane_indices (new pcl::PointIndices);
//                plane_indices->indices = inliers;
//                cout << " # of inliers : " << inliers.size() << endl;

//                // Extract only points on the main plane
//                pcl::ExtractIndices<PointT> ex;
//                ex.setInputCloud (detected_tags_pc);
//                ex.setIndices (plane_indices);
//                ex.setNegative (false);
//                ex.filter (*detected_tags_plane);

//                ex.setInputCloud(real_pc);
//                ex.filter (*real_plane);

//                if(plane_indices->indices.size() > 0){
//                    correspondences.clear();
//                    for(int i = 0; i < size1; i++){
//                        pcl::Correspondence corr;
//                        corr.index_match = i;
//                        corr.index_query = i;
//                        correspondences.push_back(corr);
//                    }
//                }

                // Create the four corners based on detected plane principal components
                //pcl::PointCloud<PointT>::Ptr plane_corners = createPlaneCornersFromCenters(detected_tags_pc, filtered_cloud);

                pclViewer->updatePointCloud (filtered_cloud, ColorHandlerX(filtered_cloud, 255.0, 0.0, 0.0), "plane");
                pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "plane");

                pclViewer->updatePointCloud (detected_tags_pc, ColorHandlerX(detected_tags_pc, 0.0, 255.0, 0.0), "tags");
                pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "tags");

              //  pclViewer->updatePointCloud (plane_corners, ColorHandlerX(plane_corners, 0.0, 0.0, 255.0), "corners");
               // pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "corners");



                // Find a coarse transformation with RANSAC
                // Correspondence Rejection (RANSAC)
                // Select 3 feature pairs randomly
                // Find Transform
                // Calculate number of (feature-feature distance < threshold)
                // After n iterations, keep the best transform
                // Reject the feature pairs for which the point to point distance is bigger than threshold in the aligned clouds
                pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> corr_rejector;
                pcl::Correspondences correspondences_final;
                corr_rejector.setInputSource(detected_tags_pc);
                corr_rejector.setInputTarget(real_pc);
                corr_rejector.setMaximumIterations(1000);
                corr_rejector.setInlierThreshold(0.005);
                corr_rejector.getRemainingCorrespondences(correspondences, correspondences_final);

                Eigen::Matrix4f coarse_transformation  = corr_rejector.getBestTransformation();
                tf::Transform world_tf = tfFromEigen(coarse_transformation);

                broadcaster.sendTransform(tf::StampedTransform(world_tf, ros::Time::now(), "marker_0", "3d_camera_estimation"));


                cout << "Correspondences before : " << correspondences.size() << endl;
                cout << "Correspondences after : " << correspondences_final.size() << endl;

                cout << "coarse_transformation : " << endl << coarse_transformation << endl;

            }

            else{

                ar_track_alvar_msgs::AlvarMarker nearest_tag = local_markers.markers.at(0);

                tf::Transform nearest_tf;
                geometry_msgs::Pose pose = nearest_tag.pose.pose;
                nearest_tf.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z) );
                nearest_tf.setRotation( tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

                tf::Transform ref_frame;
                int id = nearest_tag.id;
                ref_frame.setIdentity();
                ref_frame.setOrigin( tf::Vector3(-tags_matrix(id,0), -tags_matrix(id,1), -tags_matrix(id,2)));

                nearest_tf = nearest_tf * ref_frame;
                nearest_tf = nearest_tf.inverse();

                broadcaster.sendTransform(tf::StampedTransform(nearest_tf, ros::Time::now(), "marker_0", "3d_camera_estimation"));
            }
        }
    }


}



pcl::PointCloud<PointT>::Ptr createPlaneCornersFromCenters(pcl::PointCloud<PointT>::Ptr centers, pcl::PointCloud<PointT>::Ptr plane){

//    using namespace Eigen;

//    Vector4f centroid;
//    Matrix3f covariance;
//    Matrix3f principal_components;
//    Matrix3f principal_components_camera_frame;
//    Matrix4f reference_frame_transform;

//    pcl::PointCloud<PointT>::Ptr centers_new_referential(new pcl::PointCloud<PointT>);
//    pcl::PointCloud<PointT>::Ptr corners_on_the_plane(new pcl::PointCloud<PointT>);

////    // centroid
////    pcl::compute3DCentroid(*plane, centroid);
////    cout << "centroid" << endl;
////    cout << centroid << endl;

////    // covariance
////    pcl::computeCovarianceMatrixNormalized(*plane, centroid, covariance);
////    cout << "covariance" << endl;
////    cout << covariance << endl;

////    // principal components
////    SelfAdjointEigenSolver<Matrix3f> eigen_solver(covariance, ComputeEigenvectors);
////    principal_components = eigen_solver.eigenvectors();
////    principal_components.col(2) = principal_components.col(0).cross(principal_components.col(1));
////    cout << "Principal components Matrix" << endl;
////    cout << principal_components << endl;

////    // reference frame transform
////    reference_frame_transform = Matrix4f::Identity();
////    reference_frame_transform.block<3,3>(0,0) = principal_components.transpose();
////    reference_frame_transform.block<3,1>(0,3) = -1.f * (reference_frame_transform.block<3,3>(0,0) * centroid.head<3>());

////    // Transform center of tags in the plane referential
////    pcl::transformPointCloud(*centers, *centers_new_referential, reference_frame_transform);
////    cout << "ref_frame_transform" << endl;
////    cout << reference_frame_transform << endl;

//    mut_0.lock();
//    tf::StampedTransform stf = tf_tag_0;
//    mut_0.unlock();

//    // Point transform
//    tf::Transform tf_ref;
//   // tf_ref.setOrigin( tf::Vector3(ref_pose.pose.position.x, ref_pose.pose.position.y, ref_pose.pose.position.z ) );
//   // tf_ref.setRotation( tf::Quaternion(ref_pose.pose.orientation.x, ref_pose.pose.orientation.y, ref_pose.pose.orientation.z, ref_pose.pose.orientation.w).normalize());
//    tf_ref.setOrigin( tf_tag_0.getOrigin());
//    tf_ref.setBasis( tf_tag_0.getBasis());

//    Matrix4f point_referential_matrix;
//    pcl_ros::transformAsMatrix(tf_ref, point_referential_matrix);

//    pcl::transformPointCloud(*centers, *centers_new_referential, point_referential_matrix);
//    cout << "ref_frame_transform" << endl;
//    cout << reference_frame_transform << endl;



//    double marker_half_size = 0.0225;

//    // Create a new point cloud for the corners
//    for(int i = 0; i < centers_new_referential->points.size(); i++){

//        PointT p = centers_new_referential->points.at(i);

//        // counter clockwise from top left corners and ending with center

//        PointT p2;
//        p2.x = p.x + marker_half_size;
//        p2.y = p.y + marker_half_size;
//        corners_on_the_plane->points.push_back(p2);

//        p2.x = p.x - marker_half_size;
//        p2.y = p.y + marker_half_size;
//        corners_on_the_plane->points.push_back(p2);

//        p2.x = p.x - marker_half_size;
//        p2.y = p.y - marker_half_size;
//        corners_on_the_plane->points.push_back(p2);

//        p2.x = p.x + marker_half_size;
//        p2.y = p.y - marker_half_size;
//        corners_on_the_plane->points.push_back(p2);


//        corners_on_the_plane->points.push_back(p);
//    }


//    Matrix4f inv_referential_matrix;
//    pcl_ros::transformAsMatrix(tf_ref.inverse(), inv_referential_matrix);
//    pcl::transformPointCloud(*corners_on_the_plane, *corners_on_the_plane, inv_referential_matrix);


//    return corners_on_the_plane;


}




int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "nearest3d_estimation");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    pclViewer->setBackgroundColor (0, 0, 0);
    pclViewer->initCameraParameters ();
    pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
    vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
    renderWindow->SetSize(800,450);
    renderWindow->Render();

    // Filter points not on the main plane of the scene
    pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>);
    pclViewer->addPointCloud (temp, ColorHandlerX(temp, 255.0, 0.0, 0.0), "plane");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "plane");

    pclViewer->addPointCloud (temp, ColorHandlerX(temp, 0.0, 255.0, 0.0), "tags");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "tags");

    pclViewer->addPointCloud (temp, ColorHandlerX(temp, 0.0, 255.0, 0.0), "corners");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "corners");





    // Init tag positions real world
    initTagsMatrix();

    // Camera pose estimation publisher thread
    boost::thread pub(publisher_thread);

    // boost::thread t1(find_tf_thread);

    //cv::namedWindow("projected_image");
    //cv::startWindowThread();

    // camera  subscribers
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_callback);
    ros::Subscriber sub2 = n.subscribe("/3d/ar_pose_marker", 1, arMarkersCallback);

    ros::Rate r(30);
    while (ros::ok()) {
        ros::spinOnce();
        pclViewer->spinOnce (100);
        r.sleep();
    }
    //cv::destroyAllWindows();

    return 0;
}
