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
	using Vector4 = typename Eigen::Matrix<T, 4, 1>;
	using Matrix2 = typename Eigen::Matrix<T, 2, 2>;
	using Matrix3 = typename Eigen::Matrix<T, 3, 3>;
	using Matrix4 = typename Eigen::Matrix<T, 4, 4>;
	
	using Matrix4x3 = typename Eigen::Matrix<T, 4, 3>;
	using Matrix3x4 = typename Eigen::Matrix<T, 3, 4>;

	EKF()
	{

	}

	~EKF()
	{

	}

	void predict( const DataType w_now, const DataType time_now )
	{
		if( is_init == false ){
			time_pre = time_now;
			w_pre = w_now;
			
			is_init = true;
		
			return;
		}

		std::cout<<"start predict "<<std::endl;
	
		// 1. state prediction
		DataType delta_t = time_now - time_pre;
                std::cout<<"delta time = "<<delta_t<<std::endl;
		
		x_now(0) = x_pre(0) + x_pre(3) * delta_t * ::cos( x_pre(2) ); // x
		x_now(1) = x_pre(1) + x_pre(3) * delta_t * ::sin( x_pre(2) ); // y
		x_now(2) = x_pre(2) + w_now * delta_t; // q
		x_now(3) = x_pre(3); // v

		// 2. state covarince prediction
		F = Matrix4::Identity();
		F(0, 2) = -x_now(3) * delta_t * ::sin( x_now(2) );
		F(0, 3) = delta_t * ::cos( x_now(2) );
		F(1, 2) = x_now(3) * delta_t * ::cos( x_now(2));
		F(1, 3) = delta_t * ::sin( x_now(2) );

		/*F(0, 2) = -x_pre(3) * delta_t * ::sin( x_pre(2) );
                F(0, 3) = delta_t * ::cos( x_pre(2) );
                F(1, 2) = x_pre(3) * delta_t * ::cos( x_pre(2));
                F(1, 3) = delta_t * ::sin( x_pre(2) );
		*/

		P_now = F * P_pre * F.transpose() + Q;

		// 3. update the values
		x_pre = x_now;
		w_pre = w_now;
		time_pre = time_now;
	
		P_pre = P_now;
	
		std::cout<<"prediction end "<<std::endl;
	}

	void update( const Vector3 &z ) // delta_theta, z_v, delta_s
	{
		if( is_init == false ){
                        return;
                }
                std::cout<<"update"<<std::endl;
                std::cout<<"input measurement z: "<<std::endl<<z<<std::endl;

		// 1. measurement estimate
		Vector3 h( 0, 0, 0 );
		h(0) = x_now(2) - x_pre(2); // delta_theta
		h(1) = x_now(3); // z_v
		DataType cos_tmp = ::cos( x_pre(2) + 0.5 * h(0) );
		DataType sin_tmp = ::sin( x_pre(2) + 0.5 * h(0) );
		DataType tmp = cos_tmp + sin_tmp;
		h(2) = ( x_now(0) - x_pre(0) + x_now(1) - x_pre(1) ) / ( tmp ); // delta_s
	
		// 2. measurement error
		Vector3 error = z - h;
		std::cout<<"error = "<<std::endl<<error<<std::endl;
		
		// measurement Jacobian Matrix
		Matrix3x4 H;
		DataType h_0_2 = ( sin_tmp - cos_tmp ) * ( x_now(0) - x_pre(0) + x_now(1) - x_pre(1) ) * 0.5;

		H << 0,       0,       1,                   0,
		     0,       0,       0,                   1, 
		     1 / tmp, 1 / tmp, h_0_2 / (tmp * tmp), 0;
	
		std::cout<<"H = "<<std::endl<<H<<std::endl;
	
		// Kalman gain
		Matrix3 K_tmp = H * P_now * H.transpose() + R;
		Matrix4x3 K = P_now * H.transpose() * K_tmp.inverse();

		// state update
		x_now += K * error;
		
		// state covarince update
		P_now = ( Matrix4::Identity() - K * H ) * P_now;
		     
		std::cout<<"update end "<<std::endl;
		
		// 3. update the values
                x_pre = x_now;
   //             w_pre = w_now;
 //               time_pre = time_now;

                P_pre = P_now;

	}

	const Vector4& getStateX() const
	{
		return x_now;
	}
	
private:

	// state vector at k-1 moment
	Vector4 x_pre = Vector4::Zero();
	
	// state vector at k moment, ( x, y, q, v )
	Vector4 x_now = Vector4::Zero();
	
	// control vector at k-1 moment
	DataType w_pre = 0;
	

	// state covarince matrix
        Matrix4 P_pre = Matrix4::Identity();
        Matrix4 P_now = Matrix4::Identity();

	// state Jacobian matrix
        Matrix4 F = Matrix4::Identity();
	
	// measurement Gaussian Noise
        Matrix3 R = Matrix3::Identity();

	// state Gaussian Noise
        Matrix4 Q = Matrix4::Identity();


	// previous time moment
	DataType time_pre = 0;
	
	// 
	bool is_init = false;
};

}

#endif
