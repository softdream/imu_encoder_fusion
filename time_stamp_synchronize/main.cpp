#include "ekf_fusion.h"
#include "file_read.h"

#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <thread>

#include "time_synchronize.h"

int main()
{
	std::cout<<"--------------------- EKF FUSION--------------------"<<std::endl;

	// for displaying
	cv::Mat image = cv::Mat::zeros(900, 900, CV_8UC3);

	ekf::EKF<double> ekf;
	
	time_stamp::Synchronize sync;

	ekf::DataRead data_read;
	data_read.openFile( "imu_encoder_file.txt" );
	
	double pre_time = 0;
	bool is_init = false;
	double pre_gz = 0;
	Eigen::Matrix<double, 6, 1> encoder_data;
	Eigen::Matrix<double, 4, 1> imu_data;
	double theta = 0;
	
	int count = 0;
	while( !data_read.endOfFile() ){
		std::cout<<"--------------------------frame count : "<<count<<"------------------------" <<std::endl;
		int ret = data_read.readFrameData( encoder_data, imu_data );
		if( ret == 0 ){ // encoder data
		
			// find nearest time stamp data
			Eigen::Matrix<double, 4, 1> imu_nearest = sync.findAData( encoder_data(0) );

			//std::cout<<"encoder data: "<<std::endl<<encoder_data.transpose()<<std::endl;	
			std::cout<<"imu data : "<<std::endl<<imu_nearest.transpose()<<std::endl;
			std::cout<<"time stamp of imu : "<<imu_nearest(0)<<std::endl;			
			std::cout<<"time stamp of encoder : "<<encoder_data(0)<<std::endl;
                
			double now_time = encoder_data(0);
		        if( is_init == false ) {
                                pre_time = now_time;
				pre_gz = imu_nearest(3);
				is_init = true;
                                continue;
                        }

			Eigen::Matrix<double, 2, 1> u( encoder_data(2), encoder_data(3) );
			ekf.predict( u );
			
			double delta_t = ( now_time - pre_time ) / 1000;
			std::cout<<"delta_t = "<<delta_t<<std::endl;
			theta = delta_t * ( pre_gz + imu_nearest(3) ) * 0.5;
			std::cout<<"theta = "<<theta<<std::endl;
			ekf.update( theta );
			
			pre_gz = imu_nearest(3);
			pre_time = now_time;
			

			Eigen::Matrix<double, 3, 1> pose = ekf.getStateX();
			cv::circle( image, cv::Point2d( 800 - pose[0] * 20, 450 - pose[1] * 20 ), 3, cv::Scalar( 0, 0, 255 ), -1 );
                        cv::imshow( "trajectory", image );
                        cv::waitKey(5);

		}
		else if( ret == 1 ){ // imu data
			sync.addAData( imu_data );
		}
		count ++;

		//std::this_thread::sleep_for(std::chrono::milliseconds( 20 ));
	}

	data_read.closeFile();	

	return 0;
}
