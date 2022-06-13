#include "ekf_fusion.h"
#include "file_read.h"

#include <vector>
#include <iostream>

int main()
{
	std::cout<<"--------------------- EKF --------------------"<<std::endl;

	ekf::EKF<double> ekf;
	
	ekf::DataRead data_read;
	data_read.openFile( "test_data1.txt" );
	
	Eigen::Matrix<double, 6, 1> encoder_data;
	Eigen::Matrix<double, 4, 1> imu_data;
	int count = 0;
	while( !data_read.endOfFile() ){
		std::cout<<"frame count : "<<count <<std::endl;
		int ret = data_read.readFrameData( encoder_data, imu_data );
		if( ret == 0 ){ // encoder data
			
			Eigen::Vector3d z;
			z(0) = encoder_data(1);
			z(1) = encoder_data(2);
			z(2) = encoder_data(3);
			
			ekf.update( z );
			
			Eigen::Matrix<double, 5, 1> x_now = ekf.getStateX();
			std::cout<<"X = "<<std::endl<<x_now<<std::endl;
		}
		else if( ret == 1 ){ // imu data
			Eigen::Vector3d u_now;
			u_now(0) = imu_data(1) - (-0.0078120);
			u_now(1) = imu_data(2) - (0.027588);
			u_now(2) = imu_data(3) - (-4.549618);
			double time_now = imu_data(0) / 1000;
	
			ekf.predict( u_now, time_now );
		
		}
		count ++;
	}
	data_read.closeFile();

	

	return 0;
}
