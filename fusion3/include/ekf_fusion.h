#ifndef __EKF_FUSION_H
#define __EKF_FUISION_H

#include <iostream>
#include <Eigen/Dense>
#include <cmath>

namespace ekf
{

template<typename T>
class EKF
{
public:
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<T, 2, 1>;
	using Vector3 = typename Eigen::Matrix<T, 3, 1>;
	using Matrix2 = typename Eigen::Matrix<T, 2, 2>;
	using Matrix3 = typename Eigen::Matrix<T, 3, 3>;
	using Matrix1x3 = typename Eigen::Matrix<T, 1, 3>;
	using Matrix3x1 = typename Eigen::Matrix<T, 3, 1>;

	EKF()
	{

	}

	~EKF()
	{

	}

	void predict( const Vector2 &u_now )  // input control vector: ( delta_s, delta_theta )
	{
		if( is_init == false ){
			is_init = true;
			
			return;
		}
		std::cout<<" start predict "<<std::endl;

		// 1. state prediction
		x_now(0) = x_pre(0) + u_now(0) * ::cos( x_pre(2) + 0.5 * u_now(1) ); // x
		x_now(1) = x_pre(1) + u_now(0) * ::sin( x_pre(2) + 0.5 * u_now(1) ); // y
		x_now(2) = x_pre(2) + u_now(1); // theta

		// 2. caculate state Jacobian matrix	
		F = Matrix3::Identity();
		F(0, 2) = -u_now(0) * ::sin( x_pre(2) + 0.5 * u_now(1) );
		F(1, 2) =  u_now(0) * ::cos( x_pre(2) + 0.5 * u_now(1) );
	
		// 3. state covarince prediction
		P_now = F * P_pre * F.transpose() + Q;
	
		std::cout<<"delta_theta = "<<x_now(2) - x_pre(2)<<std::endl;
		// 4. update the old value
		//x_pre = x_now;
		//P_pre = P_now;
		//std::cout<<"x estimated : "<<std::endl<<x_now<<std::endl;
		std::cout<<" prediction end "<<std::endl;
	}	

	void update( const DataType z ) // measurement value : delta_theta
	{
		if( is_init == false ){
                        return;
                }
                std::cout<<" update "<<std::endl;

		// 1. measurement estimate
		H << 0, 0, 1;
		DataType h = H * x_now - x_pre(2);
		std::cout<<"h = "<<h<<std::endl;
		std::cout<<"z = "<<z<<std::endl;		
		
		// 2. measurement error
		//DataType error = z - h;
		DataType error = z - h;
		std::cout<<"error = "<<error<<std::endl;
	
		// 3. Kalman Gain
		DataType K_tmp = H * P_now * H.transpose() + R;
		Matrix3x1 K = P_now * H.transpose() * ( 1 / K_tmp );

		std::cout<<"K * error = "<<std::endl<<K * error<<std::endl;
		// 4. state update
		x_now += K * error;
		
		// 5. state covarince matrix update
		P_now = ( Matrix3::Identity() - K * H ) * P_now;
		
		std::cout<<" update end "<<std::endl;
	
		// 6. update the old value
                x_pre = x_now;
                P_pre = P_now;
	}

	const Vector3& getStateX() const
	{
		return x_now;
	}

private:
	// state vector at k-1 moment, ( x, y, theta )
	Vector3 x_pre = Vector3::Zero();
	
	// state vector at k moment, ( x, y, theta )
	Vector3 x_now = Vector3::Zero();

	// state covarince matrix
	Matrix3 P_pre = Matrix3::Identity();
	Matrix3 P_now = Matrix3::Identity();

	// state Jacobian matrix
	Matrix3 F = Matrix3::Identity();

	// measurement update matrix
	Matrix1x3 H = Matrix1x3::Zero();


	// state Gaussian Noise
	Matrix3 Q = Matrix3::Identity() * 10;

	// measurement Gaussian Noise
	DataType R = 10000;
	
	// previous time moment
	DataType time_pre = 0;
	// 
	bool is_init = false;
};

}

#endif
