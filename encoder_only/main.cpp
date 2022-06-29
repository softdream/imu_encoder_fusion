#include <opencv2/opencv.hpp>
#include <iostream>

#include <Eigen/Dense>

#include  "file_read.h"

int main()
{
	ekf::DataRead data_read;
	data_read.openFile( "imu_encoder_file.txt" );

	Eigen::Matrix<double, 6, 1> encoder_data;
        Eigen::Matrix<double, 4, 1> imu_data;
        int count = 0;

	cv::Mat image = cv::Mat::zeros(900, 900, CV_8UC3);
		
	Eigen::Vector3d pose( 0.0, 0.0, 0.0 );
	Eigen::Vector3d pose_pre  = pose;
	while( !data_read.endOfFile() ){
                std::cout<<"--------------------------frame count : "<<count<<"------------------------" <<std::endl;
		int ret = data_read.readFrameData( encoder_data, imu_data );
                if( ret == 0 ){ // encoder data
			double delta_s = encoder_data(2);
			double delta_theta = encoder_data(3);
			pose(0) = pose_pre(0) + delta_s * ::cos( pose(2) + 0.5 * delta_theta );
			pose(1) = pose_pre(1) + delta_s * ::sin( pose(2) + 0.5 * delta_theta );
		   	pose(2) = pose_pre(2) + delta_theta;

			pose_pre = pose;
			cv::circle( image, cv::Point2d( 800 - pose[0] * 20, 450 - pose[1] * 20 ), 3, cv::Scalar( 0, 0, 255 ), -1 );
			cv::imshow( "trajectory", image );
			cv::waitKey(10);
                }
                else if( ret == 1 ){ // imu data

                }
                count ++;

	}
	

	data_read.closeFile();

	return 0;
}
