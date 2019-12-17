#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h>


int main( int argc, char** argv )
{
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         // 相机位姿
    cv::Mat map_image(300,600,CV_8UC3,cv::Scalar(0,0,0)); //假设cost map分辨率为2cm，地图面积为6m*12m
    ifstream fin("./pose.txt");
    if (!fin)
    {
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
        return 1;
    }
    
    for ( int i=0; i<5; i++ )
    {
        boost::format fmt( "./%s/%d.%s" ); //图像文件格式
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 使用-1读取原始图像
        
        double data[7] = {0};
        for ( auto& d:data )
            fin>>d;
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d T(q);
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        poses.push_back( T );
    }
     
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
        
    for (int i=0; i<5; i++)
    {
        cout<< "转换图像中: " << i+1 <<endl;  
        cv::Mat depth = depthImgs[i];
        cv::Mat color = colorImgs[i]; 
        Eigen::Isometry3d T = poses[i];
        for ( int v=0; v<color.rows; v++ ){
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u];  
                if ( d==0 ) continue;  
                Eigen::Vector3d point; 
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 
                Eigen::Vector3d pointWorld = T*point;
              
                if(pointWorld[0] > -3.0 && pointWorld[0] < 3.0 && pointWorld[2] > -6.0 && pointWorld[2] < 6.0 && pointWorld[1] > 0.1) {
                    // std::cout << "cout:"<< pointWorld[0] <<" "<< pointWorld[2] <<" "<< pointWorld[2] << std::endl;
                    map_image.data[ (uchar)(pointWorld[0]/0.02 + 150) * map_image.step+(uchar)(pointWorld[2]/0.02 + 300)*color.channels() ] = 255;
                    map_image.data[ (uchar)(pointWorld[0]/0.02 + 150) * map_image.step+(uchar)(pointWorld[2]/0.02 + 300)*color.channels()+1 ] = 255;
                    map_image.data[ (uchar)(pointWorld[0]/0.02 + 150) * map_image.step+(uchar)(pointWorld[2]/0.02 + 300)*color.channels()+2 ] = 255;
                }   
            }
        }
        
        cv::namedWindow("1");
        cv::imshow("1",map_image);
        cvWaitKey(0);

        for ( int v=0; v<map_image.rows; v++ ){
            for ( int u=0; u<map_image.cols; u++ )
            {
                map_image.data[v*map_image.step+u*map_image.channels()] = 0;
                map_image.data[v*map_image.step+u*map_image.channels() + 1] = 0;
                map_image.data[v*map_image.step+u*map_image.channels() + 2] = 0;
            }   
        }

    }
     
    return 0;
}
