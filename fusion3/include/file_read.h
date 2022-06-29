#ifndef __FILE_READ_H
#define __FILE_READ_H

#include <iostream>
#include <vector>
#include <fstream>
#include <string.h>
#include <sstream>
#include <iomanip>

#include <stdlib.h>
#include <stdint.h>

#include <Eigen/Dense>

namespace ekf
{

class DataRead
{
public:
	DataRead()
	{

	}

	~DataRead()
	{

	}

	bool openFile( const std::string &file_name )
	{
		file.open( file_name.c_str(), std::ifstream::in );
	
		if( !file.is_open() ){
			std::cout<<"Faile to Open the File !"<<std::endl;
			return false;
		}

		std::cout<<"Open the file "<<std::endl;
	}

	bool readImuFrame( Eigen::Matrix<double, 4, 1> &data )
        {
                data.setZero();
                std::string line;

                std::getline( file, line );

                std::istringstream iss( line );

		std::string tag;
		iss >> tag;
		if( tag.compare("imu") ==  0 ){
                	std::string num;
	                for( size_t i = 0; i < 4; i ++ ){
        	                iss >> num;
                	        data(i) = std::stod( num );
                	}
                	count ++;
		}
		else {
			return false;
		}

                return true;
        }

	bool readEncoderFrame( Eigen::Matrix<double, 6, 1> &data )
        {
                data.setZero();
                std::string line;

                std::getline( file, line );

                std::istringstream iss( line );

                std::string tag;
                iss >> tag;
                if( tag.compare("encoder") ==  0 ){
                        std::string num;
                        for( size_t i = 0; i < 6; i ++ ){
                                iss >> num;
                                data(i) = std::stod( num );
                        }
                        count ++;
                }
                else {
                        return false;
                }

                return true;
        }

	const int readFrameData( Eigen::Matrix<double, 6, 1> &encoder, 
			    Eigen::Matrix<double, 4, 1> &imu )
	{
		encoder.setZero();
		imu.setZero();

		std::string line;
		std::getline( file, line );
		std::istringstream iss( line );

                std::string tag;
                iss >> tag;
                if( tag.compare("encoder") ==  0 ){
                        std::string num;
                        for( size_t i = 0; i < 6; i ++ ){
                                iss >> num;
                                encoder(i) = std::stod( num );
                        }
			count ++;
			return 0;
                }
		else if( tag.compare("imu") == 0 ){
			std::string num;
			for( size_t i = 0; i < 4; i ++ ){
				iss >> num;
				imu(i) = std::stod( num );
			}
			count ++;
			return 1;
		}
	
		return -1;
	}

	void closeFile()
	{
		return file.close();
	}

	inline const int filePointPose() 
        {
                return file.tellg();
        }

        inline const int endOfFile() const 
        {
                return file.eof();
        }

        inline const long getFrameCount() const
        {
                return count;
        }

private:
	std::ifstream file;
	long count =  0;
};

}

#endif
