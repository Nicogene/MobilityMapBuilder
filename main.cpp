#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/filter.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/geometry.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <pcl/range_image/range_image_planar.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include "MobilityScore.h"
#include "ComputeArea.h"
#include "dot.h"


#include <iostream>
typedef CGAL::Exact_predicates_inexact_constructions_kernel Ker;
using  namespace pcl;
using namespace std;


int
main(int argc, char** argv)
{
    int numfile,i;
    std::cout<<"Insert the number of the first cloud"<<std::endl;
    std::cin>>i;
    std::cout<<"Insert the number of clouds"<<std::endl;
    std::cin>>numfile;
    numfile=i+numfile;
    std::string name;

    while(i<numfile){

        // Objects for storing the point clouds.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr planenormals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr planenormalsproj(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudNoPlane(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePoints(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePointsproj(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr plane_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr plane_with_normalsproj (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudclusters(new pcl::PointCloud<pcl::PointXYZRGB>);
        cv::Mat prova;
//        ofstream trfile1("triangles1.txt");
//        ofstream trfile2("trianglesproj1.txt");
        double r=1,s=0;

        name="inputCloud"+std::to_string(i)+".pcd";


        // Read a PCD file from disk.
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(name, *cloud) != 0)
        {
            return -1;
        }

        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(name, *tmp) != 0)
        {
            return -1;
        }

        //Obtaine the image from the cloud


        int imageSizeX = 640;
        int imageSizeY = 480;
        // Center of projection. here, we choose the middle of the image.
        float centerX = 640.0f / 2.0f;
        float centerY = 480.0f / 2.0f;
        // Focal length. The value seen here has been taken from the original depth images.
        // It is safe to use the same value vertically and horizontally.
        float focalLengthX = 525.0f, focalLengthY = focalLengthX;
        // Sensor pose. Thankfully, the cloud includes the data.
        Eigen::Affine3f sensorPose=(Eigen::Affine3f)Eigen::Translation3f(cloud->sensor_origin_[0], cloud->sensor_origin_[1], cloud->sensor_origin_[2]);
        //Translation3f Traslation(cloud->sensor_origin_[0],cloud->sensor_origin_[1],cloud->sensor_origin_[3]);
        Eigen::Affine3f transformation=(Eigen::Affine3f)Eigen::Translation3f();


        // Noise level. If greater than 0, values of neighboring points will be averaged.
        // This would set the search radius (e.g., 0.03 == 3cm).
        float noiseLevel = 0.0f;
        // Minimum range. If set, any point closer to the sensor than this will be ignored.
        float minimumRange = 0.0f;
        float x,y;
        // Border size. If greater than 0, a border of "unobserved" points will be left
        // in the image when it is cropped.
        // Planar range image object.
        pcl::RangeImagePlanar rangeImagePlanar;

        rangeImagePlanar.createFromPointCloudWithFixedSize(*tmp, imageSizeX, imageSizeY,
                                                           centerX, centerY, focalLengthX, focalLengthX,
                                                           sensorPose, pcl::RangeImage::CAMERA_FRAME,
                                                           noiseLevel, minimumRange);

        if (cloud->isOrganized()) {
            prova = cv::Mat(cloud->height, cloud->width, CV_8UC3);
            cout<<"aperta e organizzata\n";
            if (!cloud->empty()) {

                for (int h=0; h<prova.rows; h++) {
                    for (int w=0; w<prova.cols; w++) {
                        pcl::PointXYZRGB point = cloud->at(w, h);
                        Eigen::Vector3i rgb = point.getRGBVector3i();

                        prova.at<cv::Vec3b>(h,w)[0] = rgb[2];
                        prova.at<cv::Vec3b>(h,w)[1] = rgb[1];
                        prova.at<cv::Vec3b>(h,w)[2] = rgb[0];
                    }
                }
            }
        }

        //cv::imwrite("Map.jpg",prova);

        //removing NAN from the cloud
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(*cloud,*cloud,mapping);
        cout<<"the cloud size is "<<cloud->size()<<endl;

        // Filter out all points with Z values not in the [0-2] range.
        pcl::PassThrough<pcl::PointXYZRGB> filterdepth;
        filterdepth.setInputCloud(cloud);
        filterdepth.setFilterFieldName("z");
        filterdepth.setFilterLimits(0.0, 1.5);
        filterdepth.filter(*cloud);
        //io::savePCDFileASCII("Provaprof.pcd",*cloud);

        //As first step we use a RANSAC technique to extract the main plane of the cloud: the floor.

        // Object for storing the plane model coefficients.
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        // Create the segmentation object.
        pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
        segmentation.setInputCloud(cloud);
        // Configure the object to look for a plane.
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        // Use RANSAC method.
        segmentation.setMethodType(pcl::SAC_RANSAC);
        // Set the maximum allowed distance to the model.
        segmentation.setDistanceThreshold(0.02);
        // Enable model coefficient refinement (optional).
        segmentation.setOptimizeCoefficients(true);

        pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
        segmentation.segment(*inlierIndices, *coefficients);

        if (inlierIndices->indices.size() == 0)
            std::cout << "Could not find any points that fitted the plane model." << std::endl;
        else
        {
            std::cerr << "Model coefficients: " << coefficients->values[0] << " "
            << coefficients->values[1] << " "
            << coefficients->values[2] << " "
            << coefficients->values[3] << std::endl;

            // Copy all inliers of the model to another cloud.
            //pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inlierIndices, *inlierPoints);
            // Also, extract the plane points to visualize them later.

            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inlierIndices);
            //We divide the cloud in 2 clouds, one containing only the floor and one without the floor
            extract.filter(*planePoints);
            extract.setNegative(true);
            extract.filter(*cloudNoPlane);
            cout<<"Cloud size "<<cloud->points.size()<<endl;
            cout<<"CloudNOPlane size "<<cloudNoPlane->points.size()<<endl;
            cout<<"Plane size "<<planePoints->points.size()<<endl;
            //io::savePCDFileASCII("NCNOPlaneX.pcd",*cloudNoPlane);
            //io::savePCDFileASCII("NCPlaneX.pcd",*planePoints);
            //Downsampling and SOR on both the clouds
            pcl::UniformSampling<pcl::PointXYZRGB> filterd;
            PointCloud<int> keypointIndices;
            if(cloudNoPlane->points.size()>=800){
                filterd.setInputCloud(cloudNoPlane);
                // We set the size of every voxel to be 1x1x1cm
                // (only one point per every cubic centimeter will survive).
                filterd.setRadiusSearch(0.01f);
                // We need an additional object to store the indices of surviving points.
                filterd.compute(keypointIndices);
                copyPointCloud(*cloudNoPlane, keypointIndices.points, *cloudNoPlane);
            }

            filterd.setInputCloud(planePoints);
            filterd.setRadiusSearch(0.01f);
            filterd.compute(keypointIndices);
            copyPointCloud(*planePoints, keypointIndices.points, *planePoints);
            //io::savePCDFileASCII("ProvaDow",*cloud);
            cout<<"CloudNOPlane size "<<cloudNoPlane->points.size()<<endl;
            cout<<"Plane size "<<planePoints->points.size()<<endl;
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
            if(cloudNoPlane->points.size()>=800){
                filter.setInputCloud(cloudNoPlane);
                // Set number of neighbors to consider to 50.
                filter.setMeanK(50);
                // Set standard deviation multiplier to 1.
                // Points with a distance larger than 1 standard deviation of the mean distance will be outliers.
                filter.setStddevMulThresh(1.0);
                filter.filter(*cloudNoPlane);
            }



            filter.setInputCloud(planePoints);
            // Set number of neighbors to consider to 50.
            filter.setMeanK(50);
            // Set standard deviation multiplier to 1.
            // Points with a distance larger than 1 standard deviation of the mean distance will be outliers.
            filter.setStddevMulThresh(1.0);
            filter.filter(*planePoints);

            //We define the variables that we will use to extract the fitted plane using LS method

            Eigen::Vector4f PlaneParameter;
            float curv;
            vector<int> indices;
            indices.resize(planePoints->points.size());

            for (int k = 0; k <indices.size() ; ++k) {
                indices[k]=k;
            }

            //Normals computation on both clouds
            NormalEstimation<PointXYZRGB,Normal> ne;
            search::KdTree<PointXYZRGB>::Ptr kdtree(new search::KdTree<PointXYZRGB>);
            if(cloudNoPlane->points.size()>=800){
                ne.setInputCloud(cloudNoPlane);
                ne.setRadiusSearch(0.06);
                //ne.setKSearch(25); //if you use this it compute the normals using k-nearest neighbors
                ne.setSearchMethod(kdtree);
                ne.compute(*normals);
            }
//            cout<<"Ciao1"<<endl;

            ne.setInputCloud(planePoints);
            ne.setRadiusSearch(0.06);
            //ne.setKSearch(25); //if you use this it compute the normals using k-nearest neighbors
            ne.setSearchMethod(kdtree);
            ne.compute(*planenormals);
            ne.computePointNormal(*planePoints,indices,PlaneParameter,curv); //It's the best LS plane

            //cout<<"I parametri del modello del piano LS sono a="<<PlaneParameter[0]<<" b= "<<PlaneParameter[1]<<" c= "<<PlaneParameter[2]<<" d= "<<PlaneParameter[3]<<endl;

            //Now we color the clouds using the RGB-Normals coding
//            cout<<"Ciao2"<<endl;
            vector<float> nx,nxp,ny,nyp,nz,nzp;
            nx.resize(cloudNoPlane->points.size());
            ny.resize(cloudNoPlane->points.size());
            nz.resize(cloudNoPlane->points.size());
            nxp.resize(planePoints->points.size());
            nyp.resize(planePoints->points.size());
            nzp.resize(planePoints->points.size());
//            cout<<nx.size()<<" "<<ny.size()<<" "<<nz.size()<<std::endl;
//            cout<<nxp.size()<<" "<<nyp.size()<<" "<<nzp.size()<<std::endl;
//            cout<<normals->size()<<" "<<planenormals->size()<<std::endl;
            //myfile<<"Number of points: "<<cnormal->points.size()<<endl;
            for (int j = 0; j <normals->size() ; ++j) {
                nx[j] = normals->points[j].normal_x;
                ny[j] = normals->points[j].normal_y;
                nz[j] = normals->points[j].normal_z;
            }
            for (int j = 0; j <planenormals->size() ; ++j) {
                nxp[j] = planenormals->points[j].normal_x;
                nyp[j] =planenormals->points[j].normal_y;
                nzp[j] =planenormals->points[j].normal_z;
            }
                float minx,miny,minz,maxx,maxy,maxz;
                float minxp,minyp,minzp,maxxp,maxyp,maxzp;
            //calcolo minimi e massimi dei valori di xyz per normalizzarli tra 0 e 255 per usarli in rgb
            maxxp=*max_element(nxp.begin(),nxp.end());
            maxyp=*max_element(nyp.begin(),nyp.end());
            maxzp=*max_element(nzp.begin(),nzp.end());
            minxp=*min_element(nxp.begin(),nxp.end());
            minyp=*min_element(nyp.begin(),nyp.end());
            minzp=*min_element(nzp.begin(),nzp.end());
//            cout<<nx.size()<<" "<<ny.size()<<" "<<nz.size()<<std::endl;
            if(normals->size()>=600){//perche' di alcuni non lo calcola
                maxx=*max_element(nx.begin(),nx.end());
                maxy=*max_element(ny.begin(),ny.end());
                maxz=*max_element(nz.begin(),nz.end());
                maxx=max(maxx,maxxp);
                maxy=max(maxy,maxyp);
                maxz=max(maxz,maxzp);
                minx=*min_element(nx.begin(),nx.end());
                miny=*min_element(ny.begin(),ny.end());
                minz=*min_element(nz.begin(),nz.end());
                minx=min(minx,minxp);
                miny=min(miny,minyp);
                minz=min(minz,minzp);
            }
            else{
                maxx=maxxp;
                maxy=maxyp;
                maxz=maxzp;
                minx=minxp;
                miny=minyp;
                minz=minzp;

            }
            //cout<<maxx<<" "<<maxy<<" "<<maxz<<endl;




            //cout<<minx<<" "<<miny<<" "<<minz<<endl;

            if(normals->size()>=600){
                for(int i=0;i<cloudNoPlane->points.size();i++)
                {
                    cloudNoPlane->points[i].r=(255/(maxx-minx))*(normals->points[i].normal_x-minx);
                    cloudNoPlane->points[i].g=(255/(maxy-miny))*(normals->points[i].normal_y-miny);
                    cloudNoPlane->points[i].b=(255/(maxz-minz))*(normals->points[i].normal_z-minz);

                }
            }

            for(int i=0;i<planePoints->points.size();i++)
            {
                planePoints->points[i].r=(255/(maxx-minx))*(planenormals->points[i].normal_x-minx);
                planePoints->points[i].g=(255/(maxy-miny))*(planenormals->points[i].normal_y-miny);
                planePoints->points[i].b=(255/(maxz-minz))*(planenormals->points[i].normal_z-minz);

            }


            //Color-based segmentation on the cloud without the floor colored with the NC coding.
            pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
            std::vector<pcl::PointIndices> clusters;

            if(cloudNoPlane->points.size()>= 800) {
                // Color-based region growing clustering object.

                clustering.setInputCloud(cloudNoPlane);
                clustering.setSearchMethod(kdtree);
                // Here, the minimum cluster size affects also the postprocessing step:
                // clusters smaller than this will be merged with their neighbors.
                clustering.setMinClusterSize(600);//800
                clustering.setMaxClusterSize(1000000);
                // Set the distance threshold, to know which points will be considered neighbors.
                clustering.setDistanceThreshold(10);//10
                // Color threshold for comparing the RGB color of two points.
                clustering.setPointColorThreshold(2);//10
                // Region color threshold for the postprocessing step: clusters with colors
                // within the threshold will be merged in one.
                clustering.setRegionColorThreshold(2);
                clustering.extract(clusters);


                pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = clustering.getColoredCloud();




                *cloudclusters = (*colored_cloud) + (*planePoints);

                //io::savePCDFileASCII("Colored.pcd", *cloudclusters);
    //			pcl::visualization::CloudViewer viewer("Cluster viewer");
    //			viewer.showCloud(colored_cloud);
    //			while (!viewer.wasStopped()) {
    //				boost::this_thread::sleep(boost::posix_time::microseconds(100));
    //			}

                //Slope computation for the floor cloud
            }
            else
                cout<<"C'è solo un piano!\n";

//            cout<<"Ciao3"<<endl;

            float pslope;
            std::vector<float> accx,accy,accz, pmeanNorm, normfit;
            pmeanNorm.resize(3); normfit.resize(3);
            for (int j=0; j < planenormals->points.size();j++){

                if(std::isnan(planenormals->points[j].normal_x)||std::isnan(planenormals->points[j].normal_y)||std::isnan(planenormals->points[j].normal_z))

                    cout<<"nan\n";
                else{

                    accx.push_back(planenormals->points[j].normal_x);
                    accy.push_back(planenormals->points[j].normal_y);
                    accz.push_back(planenormals->points[j].normal_z);}
            }

            /*maxx=*max_element(accx.begin(),accx.end());
            maxy=*max_element(accy.begin(),accy.end());
            maxz=*max_element(accz.begin(),accz.end());

            minx=*min_element(accx.begin(),accx.end());
            miny=*min_element(accy.begin(),accy.end());
            minz=*min_element(accz.begin(),accz.end());*/

            pmeanNorm[0]=accumulate(accx.begin(),accx.end(),0.0)/accx.size();
            pmeanNorm[1]=accumulate(accy.begin(),accy.end(),0.0)/accy.size();
            pmeanNorm[2]=accumulate(accz.begin(),accz.end(),0.0)/accz.size();

            normfit[0]=coefficients->values[0];
            normfit[1]=coefficients->values[1];
            normfit[2]=coefficients->values[2];

            /*cout<<"NORMALS COMPARISON\n";
            cout<<normfit[0]<<" "<<normfit[1]<<" "<<normfit[2]<<" "<<endl;

            cout<<pmeanNorm[0]<<" "<<pmeanNorm[1]<<" "<<pmeanNorm[2]<<" "<<endl;CHECKED SONO UGUALI*/

            //pslope= acos(dot(pmeanNorm,pmeanNorm)/(sqrt(pow(pmeanNorm[0],2) + pow(pmeanNorm[1],2) + pow(pmeanNorm[2],2))*sqrt(pow(pmeanNorm[0],2) + pow(pmeanNorm[1],2) + pow(pmeanNorm[2],2)))) * 180.0 / M_PI; //in gradi
            float ip,ipfit,normplane=sqrt(pow(pmeanNorm[0],2)+pow(pmeanNorm[1],2)+pow(pmeanNorm[2],2));
            float normplanefit=sqrt(pow(normfit[0],2)+pow(normfit[1],2)+pow(normfit[2],2));

            ip=dot(pmeanNorm,pmeanNorm);
            ipfit=dot(normfit,normfit);  //normfit è un vettore, normplanefit è la norma
            float a=ip/(normplane*normplane),b=ipfit/(normplanefit*normplanefit);

            //Roughness computation as distance of points to the fitted plane, obtained using LS and RANSAC methods
            //vector<double> distp,distls;
            //distp.resize(planePoints->points.size());
            //distls.resize(planePoints->points.size());


            //float b=1.0;
            //cout<<pmeanNorm[0]<<endl<<pmeanNorm[1]<<endl<<pmeanNorm[2]<<endl;
            //cout<<normplane<<endl;
            //cout<<"IP è "<<ip<<endl;
            //printf("a=%X b=%X\n",a, b);
            if(a>1)
                a=1;

            float slopefit;

            pslope=acos(a)*180/M_PI;
            slopefit=acos(b)*180/M_PI;
            if(std::isnan(slopefit))
                slopefit=0;
            //cout<<"La slope del cluster n 1 è "<<pslope<<"°"<<endl; SONO UGUALI
            s=slopefit;
            cout<<"La slope del cluster n 1 è "<< slopefit<<"°"<<endl;


//            ofstream myfilep("slopes1.txt");
            //ofstream myfiler("roughness1.txt");
            //ofstream myfileLS("roughnessLS1.txt");

//            for (int j = 0; j < accy.size() ; ++j) {
//                vector<float> norm;
//                norm.resize(3);
//                norm[0]=accx[j];
//                norm[1]=accy[j];
//                norm[2]=accz[j];
//                if(std::isnan(acos(dot(norm,pmeanNorm)/(sqrt(pow(pmeanNorm[0],2) + pow(pmeanNorm[1],2) + pow(pmeanNorm[2],2))*sqrt(pow(norm[0],2) + pow(norm[1],2) + pow(norm[2],2))))* 180.0 / M_PI))
//                    norm.clear();
//                else{

//                myfilep << acos(dot(norm,pmeanNorm)/(sqrt(pow(pmeanNorm[0],2) + pow(pmeanNorm[1],2) + pow(pmeanNorm[2],2))*sqrt(pow(norm[0],2) + pow(norm[1],2) + pow(norm[2],2))))* 180.0 / M_PI<<endl;
//                norm.clear();}
//            }
//            ofstream myfileptxt("cluster1.txt");

//            for (int j = 0; j < planePoints->points.size() ; ++j) {
//                //Calcolo la roughness come distanza dei punti dal piano trovato
//                //RANSAC PLANE
//                //distp[j]= (coefficients->values[0] * planePoints->points[j].x + coefficients->values[1] * planePoints->points[j].y + coefficients->values[2] * planePoints->points[j].z + coefficients->values[3])/(sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2)));
//                //myfiler<<distp[j]<<endl;
//                //LS PLANE
//                //distls[j]= (PlaneParameter[0] * planePoints->points[j].x + PlaneParameter[1] * planePoints->points[j].y + PlaneParameter[2] * planePoints->points[j].z + PlaneParameter[3])/(sqrt(pow(PlaneParameter[0],2)+pow(PlaneParameter[1],2)+pow(PlaneParameter[2],2)));
//                //myfileLS<<distls[j]<<endl;
//                //File for normals valuation.
//                myfileptxt << planePoints->points[j].x << "\t" << planePoints->points[j].y << "\t" << planePoints->points[j].z << "\t" <<
//                planenormals->points[j].normal_x << "\t" << planenormals->points[j].normal_y << "\t" << planenormals->points[j].normal_z <<
//                endl;

//            }

            //Altro metodo per calcolare la ROUGHNESS Ai/Ao

            double totAi=0.0,totA0=0.0;


            //Per la fast greedy projectio triangulation abbiamo biosgno di una point normal.

            pcl::concatenateFields(*planePoints, *planenormals, *plane_with_normals);
            //* cloud_with_normals = cloud + normals

            // Create search tree*
            pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
            tree2->setInputCloud (plane_with_normals);

            // Initialize objects
            pcl::GreedyProjectionTriangulation<PointXYZRGBNormal> gp3;
            pcl::PolygonMesh triangles;

            // Set the maximum distance between connected points (maximum edge length)
            gp3.setSearchRadius (0.025);

            // Set typical values for the parameters
            gp3.setMu (2.5);
            gp3.setMaximumNearestNeighbors (100);
            gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
            gp3.setMinimumAngle(M_PI/18); // 10 degrees
            gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
            gp3.setNormalConsistency(false);

            // Get result
            gp3.setInputCloud (plane_with_normals);
            gp3.setSearchMethod (tree2);
            gp3.reconstruct (triangles);

            CGAL::Delaunay_triangulation_3<Ker>::Finite_cells_iterator cit;

            std::list< CGAL::Point_3 <Ker> > points;

            double totAiDel=0,totA0Del=0;

            for (int i = 0; i <planePoints->points.size() ; ++i) {
                points.push_front(CGAL::Point_3<Ker> (planePoints->points[i].x,planePoints->points[i].y,planePoints->points[i].z));
            }
            CGAL::Delaunay_triangulation_3<Ker> dt(points.begin(),points.end());
            for (cit=dt.finite_cells_begin(); cit != dt.finite_cells_end() ; ++cit) {
                pcl::PointXYZRGB v1,v2,v3;
                CGAL::Point_3<Ker> p1(cit->vertex(0)->point()), p2(cit->vertex(1)->point()),p3(cit->vertex(2)->point());
                v1.x=p1.x(); v2.x=p2.x(); v3.x=p3.x();
                v1.y=p1.y(); v2.y=p2.y(); v3.y=p3.y();
                v1.z=p1.z(); v2.x=p2.z(); v3.x=p3.z();
//                trfile1<<v1.x<<" "<<v1.y<<" "<<v1.z<<"\n";
//                trfile1<<v2.x<<" "<<v2.y<<" "<<v2.z<<"\n";
//                trfile1<<v3.x<<" "<<v3.y<<" "<<v3.z<<"\n";

                totAiDel=totAiDel + ComputeArea(v1,v2,v3);
            }
            points.clear();

            cout<<"TotAiDel= "<<totAiDel<<endl;




            for (int l = 0; l <triangles.polygons.size() ; ++l) {

                pcl::PointXYZRGB v1,v2,v3;
                v1=planePoints->points[triangles.polygons[l].vertices[0]];
                v2=planePoints->points[triangles.polygons[l].vertices[1]];
                v3=planePoints->points[triangles.polygons[l].vertices[2]];
                //cout<<ComputeArea(v1,v2,v3)<<endl;
                //cout<<"ITERATION\n";
                //cout<<v1<<endl<<v2<<endl<<v3<<endl;

                totAi=totAi + ComputeArea(v1,v2,v3);
                //cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
            }
            //cout<<"TOTAi= "<<totAi<<endl;
            //Dopo aver calcolato l'area dei triangoli originali proietto i punti sul modello e calcolo l area di questi.
            pcl::ProjectInliers<pcl::PointXYZRGB> proj;
            proj.setModelType (pcl::SACMODEL_PLANE);
            proj.setInputCloud (planePoints);
            proj.setModelCoefficients (coefficients);
            proj.filter (*planePointsproj);


            ne.setSearchMethod(kdtree);
            ne.setInputCloud(planePointsproj);
            ne.setRadiusSearch(0.06);
            //ne.setKSearch(25); //if you use this it compute the normals using k-nearest neighbors
            ne.setSearchMethod(kdtree);
            ne.compute(*planenormalsproj);



            pcl::concatenateFields(*planePointsproj, *planenormalsproj, *plane_with_normalsproj);
            //* cloud_with_normals = cloud + normals

            // Create search tree*
            pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree3 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
            tree3->setInputCloud (plane_with_normalsproj);

            // Initialize objects
            pcl::GreedyProjectionTriangulation<PointXYZRGBNormal> gp4;
            pcl::PolygonMesh triangles2;

            // Set the maximum distance between connected points (maximum edge length)
            gp4.setSearchRadius (0.025);

            // Set typical values for the parameters
            gp4.setMu (2.5);
            gp4.setMaximumNearestNeighbors (100);
            gp4.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
            gp4.setMinimumAngle(M_PI/18); // 10 degrees
            gp4.setMaximumAngle(2*M_PI/3); // 120 degrees
            gp4.setNormalConsistency(false);

            // Get result
            gp4.setInputCloud (plane_with_normalsproj);
            gp4.setSearchMethod (tree3);
            gp4.reconstruct (triangles2);


            float totA0new=0;
            Eigen::Vector3f normtemp;
            normtemp[0]=coefficients->values[0];
            normtemp[1]=coefficients->values[1];
            normtemp[2]=coefficients->values[2];

            for (int i = 0; i <planePointsproj->points.size() ; ++i) {
                points.push_front(CGAL::Point_3<Ker> (planePointsproj->points[i].x,planePointsproj->points[i].y,planePointsproj->points[i].z));
            }
            CGAL::Delaunay_triangulation_3<Ker> dtprj(points.begin(),points.end());
            for (cit=dtprj.finite_cells_begin(); cit != dtprj.finite_cells_end() ; ++cit) {
                pcl::PointXYZRGB v1,v2,v3;
                CGAL::Point_3<Ker> p1(cit->vertex(0)->point()), p2(cit->vertex(1)->point()),p3(cit->vertex(2)->point());
                v1.x=p1.x(); v2.x=p2.x(); v3.x=p3.x();
                v1.y=p1.y(); v2.y=p2.y(); v3.y=p3.y();
                v1.z=p1.z(); v2.x=p2.z(); v3.x=p3.z();

//                trfile2<<v1.x<<" "<<v1.y<<" "<<v1.z<<"\n";
//                trfile2<<v2.x<<" "<<v2.y<<" "<<v2.z<<"\n";
//                trfile2<<v3.x<<" "<<v3.y<<" "<<v3.z<<"\n";

                totA0Del=totA0Del + ComputeArea(v1,v2,v3);
            }
            cout<<"totA0Del= "<<totA0Del<<endl;


            for (int l = 0; l <triangles2.polygons.size() ; ++l) {

                pcl::PointXYZRGB v1,v2,v3;


                //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudtmp(new pcl::PointCloud<pcl::PointXYZRGB>);
                v1=planePointsproj->points[triangles2.polygons[l].vertices[0]];
                v2=planePointsproj->points[triangles2.polygons[l].vertices[1]];
                v3=planePointsproj->points[triangles2.polygons[l].vertices[2]];


                /*pcl::geometry::project(v1.getVector3fMap(),v2.getVector3fMap(),normtemp,v1v);
                pcl::geometry::project(v2.getVector3fMap(),v1.getVector3fMap(),normtemp,v2v);
                pcl::geometry::project(v3.getVector3fMap(),v2.getVector3fMap(),normtemp,v3v);
                v1n.x=v1v[0];
                v2n.y=v2v[1];
                v3n.z=v3v[2];
                //cout<<ComputeArea(v1,v2,v3)<<endl;
                //cout<<"ITERATION\n";
                //cout<<v1<<endl<<v2<<endl<<v3<<endl;


                totA0new=totA0new + ComputeArea(v1n, v2n, v3n);

                cout<<"--------------------------\n";
                cout<<ComputeArea(v1,v2,v3)<<endl;
                cout<<ComputeArea(v1n, v2n, v3n)<<endl;*/
                totA0=totA0 + ComputeArea(v1,v2,v3);


                //cout<<"SEE HERE "<<totA0<<endl;
                //cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
            }


            //cout<<"TOTA0= "<<totA0<<endl;

            //cout<<"TOTAi= "<<totAi<<endl;

            //cout<<"!!!!!!!!!!!!!!!!!!!ROUGHNESS = "<<totAi/totA0<<endl;
            cout<<"!!!!!!!!!!!!!!!!!!!ROUGHNESS DELAUNAY = "<<totAiDel/totA0Del<<endl;

            r=totAiDel/totA0Del;
            double ms;

            ms=MobilityScore(r,s);

            cout<<"Il mobilty score è "<<ms<<endl;

            //cout<<"La roughness del cluster 1 è "<<fabs(accumulate(distp.begin(),distp.end(),0.0)/distp.size())<<endl;
            //cout<<"La roughnessLS del cluster 1 è "<<fabs(accumulate(distls.begin(),distls.end(),0.0)/distls.size())<<endl;
            cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n";

            //pcl::ConcaveHull<pcl::PointXYZRGB> hull;
            //hull.setInputCloud(planePoints);
            // Set alpha, which is the maximum length from a vertex to the center of the voronoi cell
            // (the smaller, the greater the resolution of the hull).
            //hull.setAlpha(0.1);
            //hull.reconstruct(*planePoints);

            // Projection of points on the image.

            for (int i = 0; i <planePoints->points.size(); ++i) {
                planePoints->points[i].r=0;
                planePoints->points[i].g=255;
                planePoints->points[i].b=0;
                //cout<<i<<endl;
                rangeImagePlanar.getImagePoint(planePoints->points[i].getVector3fMap(),y,x);
                if (!std::isnan(x) && !std::isnan(y)) {
                    //cout<<" \n"<<endl;
                    prova.at<cv::Vec3b>(x, y)[2] = 0;
                    prova.at<cv::Vec3b>(x, y)[1] = 255;
                    prova.at<cv::Vec3b>(x, y)[0] = 0;
                    prova.at<cv::Vec3b>(x+1, y)[2] = 0;
                    prova.at<cv::Vec3b>(x+1, y)[1] = 255;
                    prova.at<cv::Vec3b>(x+1, y)[0] = 0;
                    prova.at<cv::Vec3b>(x, y+1)[2] = 0;
                    prova.at<cv::Vec3b>(x, y+1)[1] = 255;
                    prova.at<cv::Vec3b>(x, y+1)[0] = 0;
                    prova.at<cv::Vec3b>(x+1, y+1)[2] = 0;
                    prova.at<cv::Vec3b>(x+1, y+1)[1] = 255;
                    prova.at<cv::Vec3b>(x+1, y+1)[0] = 0;
                    prova.at<cv::Vec3b>(x-1, y)[2] = 0;
                    prova.at<cv::Vec3b>(x-1, y)[1] = 255;
                    prova.at<cv::Vec3b>(x-1, y)[0] = 0;
                    prova.at<cv::Vec3b>(x, y-1)[2] = 0;
                    prova.at<cv::Vec3b>(x, y-1)[1] = 255;
                    prova.at<cv::Vec3b>(x, y-1)[0] = 0;
                    prova.at<cv::Vec3b>(x-1, y-1)[2] = 0;
                    prova.at<cv::Vec3b>(x-1, y-1)[1] = 255;
                    prova.at<cv::Vec3b>(x-1, y-1)[0] = 0;

                    prova.at<cv::Vec3b>(x+2, y)[2] = 0;
                    prova.at<cv::Vec3b>(x+2, y)[1] = 255;
                    prova.at<cv::Vec3b>(x+2, y)[0] = 0;
                    prova.at<cv::Vec3b>(x, y+2)[2] = 0;
                    prova.at<cv::Vec3b>(x, y+2)[1] = 255;
                    prova.at<cv::Vec3b>(x, y+2)[0] = 0;
                    prova.at<cv::Vec3b>(x+2, y+2)[2] = 0;
                    prova.at<cv::Vec3b>(x+2, y+2)[1] = 255;
                    prova.at<cv::Vec3b>(x+2, y+2)[0] = 0;
                    prova.at<cv::Vec3b>(x-2, y)[2] = 0;
                    prova.at<cv::Vec3b>(x-2, y)[1] = 255;
                    prova.at<cv::Vec3b>(x-2, y)[0] = 0;
                    prova.at<cv::Vec3b>(x, y-2)[2] = 0;
                    prova.at<cv::Vec3b>(x, y-2)[1] = 255;
                    prova.at<cv::Vec3b>(x, y-2)[0] = 0;
                    prova.at<cv::Vec3b>(x-2, y-2)[2] = 0;
                    prova.at<cv::Vec3b>(x-2, y-2)[1] = 255;
                    prova.at<cv::Vec3b>(x-2, y-2)[0] = 0;

                    prova.at<cv::Vec3b>(x+3, y)[2] = 0;
                    prova.at<cv::Vec3b>(x+3, y)[1] = 255;
                    prova.at<cv::Vec3b>(x+3, y)[0] = 0;
                    prova.at<cv::Vec3b>(x, y+3)[2] = 0;
                    prova.at<cv::Vec3b>(x, y+3)[1] = 255;
                    prova.at<cv::Vec3b>(x, y+3)[0] = 0;
                    prova.at<cv::Vec3b>(x+3, y+3)[2] = 0;
                    prova.at<cv::Vec3b>(x+3, y+3)[1] = 255;
                    prova.at<cv::Vec3b>(x+3, y+3)[0] = 0;
                    prova.at<cv::Vec3b>(x-3, y)[2] = 0;
                    prova.at<cv::Vec3b>(x-3, y)[1] = 255;
                    prova.at<cv::Vec3b>(x-3, y)[0] = 0;
                    prova.at<cv::Vec3b>(x, y-3)[2] = 0;
                    prova.at<cv::Vec3b>(x, y-3)[1] = 255;
                    prova.at<cv::Vec3b>(x, y-3)[0] = 0;
                    prova.at<cv::Vec3b>(x-3, y-3)[2] = 0;
                    prova.at<cv::Vec3b>(x-3, y-3)[1] = 255;
                    prova.at<cv::Vec3b>(x-3, y-3)[0] = 0;

                }



            }

            //cv::imwrite("Map.jpg",prova);


//            pcl::io::savePCDFileASCII("cluster1.pcd",*planePoints);

            *cloudMap=*planePoints;
            int currentClusterNum = 2;
            for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
            {
                // ...add all its points to a new cloud...
                //For each cluster obtained through Color-based region growing techinque we do the same steps done for the floor cloud,
                // we compute slope and roughness
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeclus(new PointCloud<PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterproj(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::Normal>::Ptr ncluster(new pcl::PointCloud<pcl::Normal>);
                pcl::PointCloud<pcl::Normal>::Ptr nclusterproj(new pcl::PointCloud<pcl::Normal>);
                pcl::PointCloud<PointXYZRGBNormal>::Ptr cluster_with_normals( new PointCloud<PointXYZRGBNormal>);
                pcl::PointCloud<PointXYZRGBNormal>::Ptr cluster_with_normalsproj( new PointCloud<PointXYZRGBNormal>);
//                std::string fileName4 = "triangles" + std::to_string(currentClusterNum) + ".txt";
//                std::string fileName5 = "trianglesproj" + std::to_string(currentClusterNum) + ".txt";
//                ofstream myfile4(fileName4.c_str());
//                ofstream myfile5(fileName5.c_str());
                double rc=1,sc=0,msc;
                vector<int> indicesclus;
                float curvclus;
                //Eigen::Vector4f PlaneParamClus;
                for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++){
                    cluster->points.push_back(cloudNoPlane->points[*point]);
                    planeclus->points.push_back(cloudNoPlane->points[*point]);
                    ncluster->points.push_back(normals->points[*point]);} //Sono tutti i normals della cloud no plane.

                cluster->width = cluster->points.size();
                cluster->height = 1;
                cluster->is_dense = true;
                ncluster->width = cluster->points.size();
                //ncluster->height = 1;
                //ncluster->is_dense = true;

                indicesclus.resize(cluster->points.size());
                for (int k = 0; k <indicesclus.size() ; ++k) {
                    indicesclus[k]=k;

                }

                // Object for storing the plane model coefficients.
                pcl::ModelCoefficients::Ptr coefficientsclus(new pcl::ModelCoefficients);
                // Create the segmentation object.
                pcl::SACSegmentation<pcl::PointXYZRGB> segmentationclus;
                segmentationclus.setInputCloud(cluster);
                // Configure the object to look for a plane.
                segmentationclus.setModelType(pcl::SACMODEL_PLANE);
                // Use RANSAC method.
                segmentationclus.setMethodType(pcl::SAC_RANSAC);
                // Set the maximum allowed distance to the model.
                segmentationclus.setDistanceThreshold(0.01);//0,01
                // Enable model coefficient refinement (optional).
                segmentationclus.setOptimizeCoefficients(true);

                pcl::PointIndices::Ptr inlierIndicesclus(new pcl::PointIndices);
                segmentationclus.segment(*inlierIndicesclus, *coefficientsclus);
                //cout<<inlierIndicesclus->indices.size()<<endl;

                if (inlierIndices->indices.size() == 0)
                    std::cout << "Could not find any points that fitted the plane model." << std::endl;
                else
                {
                    /*std::cerr << "Model coefficients: " << coefficientsclus->values[0] << " "
                    << coefficientsclus->values[1] << " "
                    << coefficientsclus->values[2] << " "
                    << coefficientsclus->values[3] << std::endl;*/

                    //Altro metodo per calcolare la ROUGHNESS Ai/Ao

                    double totAic=0.0,totA0c=0.0,totAiDelc=0.0,totA0Delc=0.0;


                    //Per la fast greedy projection triangulation abbiamo biosgno di una point normal.



                    // Object for retrieving the convex hull.
                    pcl::ConcaveHull<pcl::PointXYZRGB> hullc;
                    hullc.setInputCloud(planeclus);
                    hullc.setAlpha(0.01);
                    hullc.reconstruct(*planeclus);

                    pcl::concatenateFields(*cluster, *ncluster, *cluster_with_normals);
                    //* cloud_with_normals = cloud + normals

                    // Create search tree*
                    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
                    tree4->setInputCloud (cluster_with_normals);

                    // Initialize objects
                    pcl::GreedyProjectionTriangulation<PointXYZRGBNormal> gp5;
                    pcl::PolygonMesh triangles3;

                    // Set the maximum distance between connected points (maximum edge length)
                    gp5.setSearchRadius (0.025);//0.025

                    // Set typical values for the parameters
                    gp5.setMu (2.5); //5
                    gp5.setMaximumNearestNeighbors (100); //100
                    gp5.setMaximumSurfaceAngle(M_PI); // 45 degrees
                    gp5.setMinimumAngle(M_PI/18); // 10 degrees
                    gp5.setMaximumAngle(2*M_PI/3); // 120 degrees
                    gp5.setNormalConsistency(false);

                    // Get result
                    gp5.setInputCloud (cluster_with_normals);
                    gp5.setSearchMethod (tree4);
                    gp5.reconstruct (triangles3);

                    CGAL::Delaunay_triangulation_3<Ker>::Finite_cells_iterator citclus;

                    std::list< CGAL::Point_3 <Ker> > pointsclus;


                    for (int i = 0; i <cluster->points.size() ; ++i) {
                        pointsclus.push_front(CGAL::Point_3<Ker> (cluster->points[i].x,cluster->points[i].y,cluster->points[i].z));
                    }
                    CGAL::Delaunay_triangulation_3<Ker> dtclus(pointsclus.begin(),pointsclus.end());
                    for (cit=dtclus.finite_cells_begin(); cit != dtclus.finite_cells_end() ; ++cit) {
                        pcl::PointXYZRGB v1,v2,v3;
                        CGAL::Point_3<Ker> p1(cit->vertex(0)->point()), p2(cit->vertex(1)->point()),p3(cit->vertex(2)->point());
                        v1.x=p1.x(); v2.x=p2.x(); v3.x=p3.x();
                        v1.y=p1.y(); v2.y=p2.y(); v3.y=p3.y();
                        v1.z=p1.z(); v2.x=p2.z(); v3.x=p3.z();

//                        myfile4<<v1.x<<" "<<v1.y<<" "<<v1.z<<"\n";
//                        myfile4<<v2.x<<" "<<v2.y<<" "<<v2.z<<"\n";
//                        myfile4<<v3.x<<" "<<v3.y<<" "<<v3.z<<"\n";

                        totAiDelc=totAiDelc + ComputeArea(v1,v2,v3);
                    }
                    pointsclus.clear();

                    cout<<"TotAiDel= "<<totAiDelc<<endl;



                    for (int l = 0; l <triangles3.polygons.size() ; ++l) {

                        pcl::PointXYZRGB v1,v2,v3;
                        v1=cluster->points[triangles3.polygons[l].vertices[0]];
                        v2=cluster->points[triangles3.polygons[l].vertices[1]];
                        v3=cluster->points[triangles3.polygons[l].vertices[2]];

                        totAic=totAic + ComputeArea(v1,v2,v3);
                    }

                    //Dopo aver calcolato l'area dei triangoli originali proietto i punti sul modello e calcolo l area di questi.
                    pcl::ProjectInliers<pcl::PointXYZRGB> proj1;
                    proj1.setModelType (pcl::SACMODEL_PLANE);
                    proj1.setInputCloud (cluster);
                    proj1.setModelCoefficients (coefficientsclus);
                    proj1.filter (*clusterproj);

                    ne.setSearchMethod(kdtree);
                    ne.setInputCloud(clusterproj);
                    ne.setRadiusSearch(0.06);
                    //ne.setKSearch(25); //if you use this it compute the normals using k-nearest neighbors
                    ne.setSearchMethod(kdtree);
                    ne.compute(*nclusterproj);



                    pcl::concatenateFields(*clusterproj, *nclusterproj, *cluster_with_normalsproj);
                    //* cloud_with_normals = cloud + normals

                    // Create search tree*
                    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree6 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
                    tree6->setInputCloud (cluster_with_normalsproj);

                    // Initialize objects
                    pcl::GreedyProjectionTriangulation<PointXYZRGBNormal> gp6;
                    pcl::PolygonMesh triangles4;

                    // Set the maximum distance between connected points (maximum edge length)
                    gp6.setSearchRadius (0.025);//1

                    // Set typical values for the parameters
                    gp6.setMu (2.5); //2.5
                    gp6.setMaximumNearestNeighbors (250); //250
                    gp6.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
                    gp6.setMinimumAngle(M_PI/18); // 10 degrees
                    gp6.setMaximumAngle(2*M_PI/3); // 120 degrees
                    gp6.setNormalConsistency(false);

                    // Get result
                    gp6.setInputCloud (cluster_with_normalsproj);
                    gp6.setSearchMethod (tree6);
                    gp6.reconstruct (triangles4);

                    for (int i = 0; i <clusterproj->points.size() ; ++i) {
                        pointsclus.push_front(CGAL::Point_3<Ker> (clusterproj->points[i].x,clusterproj->points[i].y,clusterproj->points[i].z));
                    }
                    CGAL::Delaunay_triangulation_3<Ker> dtclusprj(pointsclus.begin(),pointsclus.end());
                    for (cit=dtclusprj.finite_cells_begin(); cit != dtclusprj.finite_cells_end() ; ++cit) {
                        pcl::PointXYZRGB v1,v2,v3;
                        CGAL::Point_3<Ker> p1(cit->vertex(0)->point()), p2(cit->vertex(1)->point()),p3(cit->vertex(2)->point());
                        v1.x=p1.x(); v2.x=p2.x(); v3.x=p3.x();
                        v1.y=p1.y(); v2.y=p2.y(); v3.y=p3.y();
                        v1.z=p1.z(); v2.x=p2.z(); v3.x=p3.z();

//                        myfile5<<v1.x<<" "<<v1.y<<" "<<v1.z<<"\n";
//                        myfile5<<v2.x<<" "<<v2.y<<" "<<v2.z<<"\n";
//                        myfile5<<v3.x<<" "<<v3.y<<" "<<v3.z<<"\n";

                        totA0Delc=totA0Delc + ComputeArea(v1,v2,v3);
                    }

                    pointsclus.clear();

                    for (int i = 0; i <planeclus->points.size() ; ++i) {
                        pointsclus.push_front(CGAL::Point_3<Ker> (planeclus->points[i].x,planeclus->points[i].y,planeclus->points[i].z));
                    }


                    cout<<"TotA0Del= "<<totA0Delc<<endl;


                    for (int l = 0; l <triangles4.polygons.size() ; ++l) {

                        pcl::PointXYZRGB v1,v2,v3;
                        v1=clusterproj->points[triangles4.polygons[l].vertices[0]];
                        v2=clusterproj->points[triangles4.polygons[l].vertices[1]];
                        v3=clusterproj->points[triangles4.polygons[l].vertices[2]];


                        totA0c=totA0c + ComputeArea(v1,v2,v3);

                    }

                    //cout<<"TOTA0= "<<totA0c<<endl;
                    //cout<<"TOTA0PCL"<<totA0PCL<<endl;
                    //cout<<"TOTAi= "<<totAic<<endl;
                    //cout<<"TOTAiPCL"<<totAiPCL<<endl;

                    //cout<<"!!!!!!!!!!!!!!!!!!!ROUGHNESS = "<<totAic/totA0c<<endl;
                    cout<<"!!!!!!!!!!!!!!!!!!!ROUGHNESS DELAUNAY = "<<totAiDelc/totA0Delc<<endl;
                    rc=totAiDelc/totA0Delc;


                }

                float slope;



                std::vector<float> accx,accy,accz,normfitclus,meanNorm;
                normfitclus.resize(3);meanNorm.resize(3);
                for (int j=0; j < ncluster->points.size();j++){

                    if(std::isnan(ncluster->points[j].normal_x)||std::isnan(ncluster->points[j].normal_y)||std::isnan(ncluster->points[j].normal_z))

                        cout<<"nan\n";
                    else{

                        accx.push_back(ncluster->points[j].normal_x);
                        accy.push_back(ncluster->points[j].normal_y);
                        accz.push_back(ncluster->points[j].normal_z);} }

                /*maxx=*max_element(accx.begin(),accx.end());
                maxy=*max_element(accy.begin(),accy.end());
                maxz=*max_element(accz.begin(),accz.end());

                minx=*min_element(accx.begin(),accx.end());
                miny=*min_element(accy.begin(),accy.end());
                minz=*min_element(accz.begin(),accz.end());*/

                meanNorm[0]=accumulate(accx.begin(),accx.end(),0.0)/accx.size();
                meanNorm[1]=accumulate(accy.begin(),accy.end(),0.0)/accy.size();
                meanNorm[2]=accumulate(accz.begin(),accz.end(),0.0)/accz.size();


                normfitclus[0]=coefficientsclus->values[0];
                normfitclus[1]=coefficientsclus->values[1];
                normfitclus[2]=coefficientsclus->values[2];

                //cout<<"NORMALS COMPARISON\n";
                //cout<<normfitclus[0]<<" "<<normfitclus[1]<<" "<<normfitclus[2]<<" "<<endl;

                //cout<<meanNorm[0]<<" "<<meanNorm[1]<<" "<<meanNorm[2]<<" "<<endl;


                //cout<< meanNorm[0]<<" "<<meanNorm[1]<<" "<<meanNorm[2]<< " "<<endl;
                //cout<<sqrt(pow(meanNorm[0],2) + pow(meanNorm[1],2) + pow(meanNorm[2],2))<<" "<<sqrt(pow(pmeanNorm[0],2) + pow(pmeanNorm[1],2) + pow(pmeanNorm[2],2))<<endl;
                //cout<< sqrt(pow(0.0781231,2) + pow(-0.51531,2) + pow(-0.852569,2))<<endl;
                //cout<< dot(meanNorm,pmeanNorm)<<endl;
                //cout<<dot(meanNorm,pmeanNorm)/(sqrt(pow(meanNorm[0],2) + pow(meanNorm[1],2) + pow(pmeanNorm[2],2))*sqrt(pow(pmeanNorm[0],2) + pow(pmeanNorm[1],2) + pow(pmeanNorm[2],2)))<<endl;
                slope= acos(fabs(dot(normfitclus,normfit))/(sqrt(pow(normfitclus[0],2) + pow(normfitclus[1],2) + pow(normfitclus[2],2))*sqrt(pow(normfit[0],2) + pow(normfit[1],2) + pow(normfit[2],2)))) * 180.0 / M_PI; //in gradi


                cout<<"La slope del cluster n "<< currentClusterNum <<" è "<<slope<<"°"<<endl;
                sc=slope;

                msc=MobilityScore(rc,sc);
                cout<<"Il mobilty score è "<<msc<<endl;
                double red,green,blue;
                if(msc==1){
                    red=0;green=255;blue=0;}
                else if(msc==0.75){
                    red=255;green=255;blue=0;}
                else if(msc==0.5){
                    red=255;green=180;blue=0;}
                else if(msc==0.25){
                    red=255;green=150;blue=0;}
                else if(msc==0){
                    red=255;green=0;blue=0;}


                for (int i = 0; i <cluster->points.size(); ++i) {
                    //cout<<i<<endl;
                    cluster->points[i].r=red;
                    cluster->points[i].g=green;
                    cluster->points[i].b=0;
                    rangeImagePlanar.getImagePoint(cluster->points[i].getVector3fMap(),y,x);
                    if (!std::isnan(x) && !std::isnan(y)) {
                        //cout<<" \n"<<endl;
                        //E' BGR
                        prova.at<cv::Vec3b>(x, y)[2] = red;
                        prova.at<cv::Vec3b>(x, y)[1] = green;
                        prova.at<cv::Vec3b>(x, y)[0] = 0;
                        prova.at<cv::Vec3b>(x+1, y)[2] = red;
                        prova.at<cv::Vec3b>(x+1, y)[1] = green;
                        prova.at<cv::Vec3b>(x+1, y)[0] = 0;
                        prova.at<cv::Vec3b>(x, y+1)[2] = red;
                        prova.at<cv::Vec3b>(x, y+1)[1] = green;
                        prova.at<cv::Vec3b>(x, y+1)[0] = 0;
                        prova.at<cv::Vec3b>(x+1, y+1)[2] = red;
                        prova.at<cv::Vec3b>(x+1, y+1)[1] = green;
                        prova.at<cv::Vec3b>(x+1, y+1)[0] = 0;
                        prova.at<cv::Vec3b>(x-1, y)[2] = red;
                        prova.at<cv::Vec3b>(x-1, y)[1] = green;
                        prova.at<cv::Vec3b>(x-1, y)[0] = 0;
                        prova.at<cv::Vec3b>(x, y-1)[2] = red;
                        prova.at<cv::Vec3b>(x, y-1)[1] = green;
                        prova.at<cv::Vec3b>(x, y-1)[0] = 0;
                        prova.at<cv::Vec3b>(x-1, y-1)[2] = red;
                        prova.at<cv::Vec3b>(x-1, y-1)[1] = green;
                        prova.at<cv::Vec3b>(x-1, y-1)[0] = 0;

                        prova.at<cv::Vec3b>(x+2, y)[2] = red;
                        prova.at<cv::Vec3b>(x+2, y)[1] = green;
                        prova.at<cv::Vec3b>(x+2, y)[0] = 0;
                        prova.at<cv::Vec3b>(x, y+2)[2] = red;
                        prova.at<cv::Vec3b>(x, y+2)[1] = green;
                        prova.at<cv::Vec3b>(x, y+2)[0] = 0;
                        prova.at<cv::Vec3b>(x+2, y+2)[2] = red;
                        prova.at<cv::Vec3b>(x+2, y+2)[1] = green;
                        prova.at<cv::Vec3b>(x+2, y+2)[0] = 0;
                        prova.at<cv::Vec3b>(x-2, y)[2] = red;
                        prova.at<cv::Vec3b>(x-2, y)[1] = green;
                        prova.at<cv::Vec3b>(x-2, y)[0] = 0;
                        prova.at<cv::Vec3b>(x, y-2)[2] = red;
                        prova.at<cv::Vec3b>(x, y-2)[1] = green;
                        prova.at<cv::Vec3b>(x, y-2)[0] = 0;
                        prova.at<cv::Vec3b>(x-2, y-2)[2] = red;
                        prova.at<cv::Vec3b>(x-2, y-2)[1] = green;
                        prova.at<cv::Vec3b>(x-2, y-2)[0] = 0;

                        prova.at<cv::Vec3b>(x+3, y)[2] = red;
                        prova.at<cv::Vec3b>(x+3, y)[1] = green;
                        prova.at<cv::Vec3b>(x+3, y)[0] = 0;
                        prova.at<cv::Vec3b>(x, y+3)[2] = red;
                        prova.at<cv::Vec3b>(x, y+3)[1] = green;
                        prova.at<cv::Vec3b>(x, y+3)[0] = 0;
                        prova.at<cv::Vec3b>(x+3, y+3)[2] = red;
                        prova.at<cv::Vec3b>(x+3, y+3)[1] = green;
                        prova.at<cv::Vec3b>(x+3, y+3)[0] = 0;
                        prova.at<cv::Vec3b>(x-3, y)[2] = red;
                        prova.at<cv::Vec3b>(x-3, y)[1] = green;
                        prova.at<cv::Vec3b>(x-3, y)[0] = 0;
                        prova.at<cv::Vec3b>(x, y-3)[2] = red;
                        prova.at<cv::Vec3b>(x, y-3)[1] = green;
                        prova.at<cv::Vec3b>(x, y-3)[0] = 0;
                        prova.at<cv::Vec3b>(x-3, y-3)[2] = red;
                        prova.at<cv::Vec3b>(x-3, y-3)[1] = green;
                        prova.at<cv::Vec3b>(x-3, y-3)[0] = 0;
                    }



                }

                *cloudMap=*cloudMap+*cluster;








                // ...and save it to disk.
                if (cluster->points.size() <= 0)
                    break;
                std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
                cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n";
                std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";

//                std::string fileName3 = "cluster" + boost::to_string(currentClusterNum) + ".txt";
//                std::string fileName2 = "slopes" + boost::to_string(currentClusterNum) + ".txt";


//                ofstream myfile(fileName3.c_str());
//                ofstream myfile2(fileName2.c_str());

//                for (int j = 0; j < accy.size() ; ++j) {
//                    vector<float> norm;
//                    norm.resize(3);
//                    norm[0]=accx[j];
//                    norm[1]=accy[j];
//                    norm[2]=accz[j];
//                    if(std::isnan(acos(dot(norm,normfit)/(sqrt(pow(normfit[0],2) + pow(normfit[1],2) + pow(normfit[2],2))*sqrt(pow(norm[0],2) + pow(norm[1],2) + pow(norm[2],2))))* 180.0 / M_PI))
//                        norm.clear();
//                    else{

//                        myfile2 << acos(fabs(dot(norm,normfit))/(sqrt(pow(normfit[0],2) + pow(normfit[1],2) + pow(normfit[2],2))*sqrt(pow(norm[0],2) + pow(norm[1],2) + pow(norm[2],2))))* 180.0 / M_PI<<endl;
//                        norm.clear();}
//                }

//                for (int j = 0; j < cluster->points.size() ; ++j) {
//                    myfile << cluster->points[j].x << "\t" << cluster->points[j].y << "\t" << cluster->points[j].z << "\t" <<

//                    ncluster->points[j].normal_x << "\t" << ncluster->points[j].normal_y << "\t" << ncluster->points[j].normal_z <<
//                    endl;
//                }


                //pcl::io::savePCDFileASCII(fileName, *cluster);

                accx.clear();accy.clear();accz.clear();normfitclus.clear(); //Libero la memoria
                cluster->clear();
                planeclus->clear();
                clusterproj->clear();
                ncluster->clear();
                nclusterproj->clear();
                cluster_with_normals->clear();
                cluster_with_normalsproj->clear();

                currentClusterNum++;
            }

            std::string fileNameim = name;
            std::string namecloud=name;
            fileNameim=/*"ob"+*/fileNameim+"BIG.png";
            pcl::io::savePCDFileASCII("Map"+namecloud, *cloudMap);

            //cv::imwrite(fileNameim,prova);




        }
        cloud->clear();
        tmp->clear();
        normals->clear();
        planenormals->clear();
        planenormalsproj->clear();
        cloudNoPlane->clear();
        planePoints->clear();
        planePointsproj->clear();
        plane_with_normals->clear();
        plane_with_normalsproj->clear();
        cloudMap->clear();
        cloudclusters->clear();



        i++;
    }
}
