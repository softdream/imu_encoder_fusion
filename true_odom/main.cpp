#include <opencv2/opencv.hpp>
#include <iostream>

#include <fstream>
#include <string.h>
#include <sstream>
#include <iomanip>

#include <vector>
#include <algorithm>

#include <Eigen/Dense>

int main()
{
	std::ifstream simu_file;
	simu_file.open( "odom_data.txt", std::ifstream::in );

        if( !simu_file.is_open() ){
                std::cerr<<"Failed to open the simulation file !"<<std::endl;
                return false;
        }

	cv::Mat image = cv::Mat::zeros(900, 900, CV_8UC3);

        cv::circle( image, cv::Point( 450, 450 ), 3, cv::Scalar(0, 0, 255), -1 );
	int count = 0;
	std::vector<float> pose_pre(3, 0);
	std::vector<float> pose(3, 0);

	cv::imshow( "trajectory", image );
	cv::waitKey( 0 );

	std::vector<Eigen::Vector2f> poses;
	
	while( count < 10732 ){
		std::string line;
		std::getline( simu_file, line );
		std::istringstream iss( line );
		std::string num;

		iss >> num;
	//	iss >> num;
		for( int i = 0; i < 2; i ++ ){
			iss >> num;
			//std::cout<<num<<" ";
			pose[i] = std::stof( num );
		}


		std::cout<<"frame: "<<count<<", x = " <<pose[0] <<", y = "<<pose[1]<<std::endl;
		
		//cv::line( image, cv::Point2f( 450 - pose_pre[0] * 15, 450 - pose_pre[1] * 15 ), cv::Point2d( 450 - pose[0] * 15, 450 - pose[1] * 15 ), cv::Scalar( 0, 0, 255 ), 1 );
	
		cv::circle( image, cv::Point2d( 800 - pose[0] * 15, 450 -pose[1] * 15 ), 1, cv::Scalar( 0, 255, 0 ), -1 );
		
		cv::imshow( "trajectory", image );
		cv::waitKey( 10 );
	}

	

	cv::waitKey(0);

	return 0;
}

