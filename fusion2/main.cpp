#include "ekf_fusion.h"
#include "file_read.h"

#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <thread>

int main()
{
	std::cout<<"--------------------- EKF FUSION--------------------"<<std::endl;

	// for displaying
	cv::Mat image = cv::Mat::zeros(900, 900, CV_8UC3);

	ekf::EKF<double> ekf;
	
	ekf::DataRead data_read;
	data_read.openFile( "imu_encoder_file.txt" );
	
	Eigen::Matrix<double, 6, 1> encoder_data;
	Eigen::Matrix<double, 4, 1> imu_data;
	int count = 0;
	while( !data_read.endOfFile() ){
		std::cout<<"--------------------------frame count : "<<count<<"------------------------" <<std::endl;
		int ret = data_read.readFrameData( encoder_data, imu_data );
		if( ret == 0 ){ // encoder data
			
			Eigen::Vector3d z;
			z(1) = encoder_data(1); // v
			z(2) = encoder_data(2); // s
			z(0) = encoder_data(3); // theta
			
			ekf.update( z );
			
			Eigen::Matrix<double, 4, 1> x_now = ekf.getStateX();
			std::cout<<"X = "<<std::endl<<x_now<<std::endl;
			
			cv::circle( image, cv::Point2d( 450 - x_now[0] * 10, 450 - x_now[1] * 10 ), 3, cv::Scalar( 0, 0, 255 ), -1 );
                        cv::imshow( "trajectory", image );
                        cv::waitKey(10);
		}
		else if( ret == 1 ){ // imu data
			double time_now = imu_data(0) / 1000;
			double w_now = imu_data(3);			
	
			std::cout<<"timestamp = "<<time_now<<std::endl;
		
			ekf.predict( w_now, time_now );
		
		}
		count ++;

		//std::this_thread::sleep_for(std::chrono::milliseconds( 20 ));
	}
	data_read.closeFile();

	

	return 0;
}
