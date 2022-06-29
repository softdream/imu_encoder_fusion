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
	using Vector5 = typename Eigen::Matrix<T, 5, 1>;
	using Matrix2 = typename Eigen::Matrix<T, 2, 2>;
	using Matrix3 = typename Eigen::Matrix<T, 3, 3>;
	using Matrix5 = typename Eigen::Matrix<T, 5, 5>;
	using Matrix5x3 = typename Eigen::Matrix<T, 5, 3>;
	using Matrix3x5 = typename Eigen::Matrix<T, 3, 5>;

	EKF()
	{

	}

	~EKF()
	{

	}

	void predict( const Vector3 &u_now, const DataType time_now )
	{
		if( is_init == false ){
			time_pre = time_now;
			u_pre = u_now;
			
			is_init = true;
			
			std::cout<<"initialize !"<<std::endl;			

			return;
		}
		std::cout<<"predict "<<std::endl;		

		// 1. state prediction
		Matrix2 q_rotation_pre;
		q_rotation_pre << ::cos(x_pre[0]), -::sin(x_pre[0]),
				  ::sin(x_pre[0]),  ::cos(x_pre[0]);
	
		DataType w_m = 0.5 * ( u_pre[2] + u_now[2] );

		DataType delta_t = time_now - time_pre;
		std::cout<<"delta time = "<<delta_t<<std::endl;	

		x_now[0] = x_pre[0] + w_m * delta_t; // q

		Matrix2 q_rotation_now;
		q_rotation_now << ::cos(x_now[0]), -::sin(x_now[0]),
				  ::sin(x_now[0]),  ::cos(x_now[0]);
	
		Vector2 a_m = 0.5 * ( q_rotation_pre * Vector2( u_pre[0], u_pre[1] ) 
				    + q_rotation_now * Vector2( u_now[0], u_now[1] ) );
	
		x_now[1] = x_pre[1] + a_m[0] * delta_t; // v_x
		x_now[2] = x_pre[2] + a_m[1] * delta_t; // v_y
		
		x_now[3] = x_pre[3] + x_pre[1] * delta_t + 0.5 * a_m[0] * delta_t * delta_t;
		x_now[4] = x_pre[4] + x_pre[2] * delta_t + 0.5 * a_m[1] * delta_t * delta_t;
		std::cout <<"predicted x = "<<std::endl<<x_now<<std::endl;
	
		// ---------- update the value
		x_pre = x_now;
		u_pre = u_now;
		time_pre = time_now;

		// 2. state covarince prediction
		F(1, 0) =  0.5 * ( -::sin( x_pre[0] ) * u_pre[0] - ::cos( x_pre[0] ) * u_pre[1] ) * delta_t;
		F(2, 0) =  0.5 * (  ::cos( x_pre[0] ) * u_pre[0] - ::sin( x_pre[0] ) * u_pre[1] ) * delta_t;
		F(3, 0) = 0.25 * ( -::sin( x_pre[0] ) * u_pre[0] - ::cos( x_pre[0] ) * u_pre[1] ) * delta_t * delta_t;
		F(4, 0) = 0.25 * (  ::cos( x_pre[0] ) * u_pre[0] - ::sin( x_pre[0] ) * u_pre[1] ) * delta_t * delta_t;
		F(3, 1) = delta_t;
		F(4, 2) = delta_t;

		P_now = F * P_pre * F.transpose() + Q;
		
		// ---------- update the value 
		P_pre = P_now;
		
		std::cout<<"predict end"<<std::endl;
	}

	void update( const Vector3 &z ) // theta, s, v
	{
		if( is_init == false ){
			return;
		}
		std::cout<<"update"<<std::endl;
		std::cout<<"input measurement z: "<<std::endl<<z<<std::endl;

		// measurement function
		Vector3 h_measurement;
		h_measurement[0] = x_now[0] - x_pre[0]; // theta

		DataType cos_tmp = ::cos( x_pre[0] + 0.5 * h_measurement[0] );
		DataType sin_tmp = ::sin( x_pre[0] + 0.5 * h_measurement[0] ) ;
		DataType tmp = cos_tmp + sin_tmp;
		h_measurement[1] = ( x_now[3] - x_pre[3] + x_now[4] - x_pre[4] ) / ( tmp ); // s
		h_measurement[2] = ::sqrt( x_now[1] * x_now[1] + x_now[2] * x_now[2] ); // v
	
		// error
		Vector3 error = z - h_measurement;
		std::cout<<"error = "<<std::endl<<error<<std::endl;		

		// measurement Jacobian Matrix
		Matrix3x5 H;
		DataType h_1_0 = ( sin_tmp - cos_tmp ) * ( x_now[3] - x_pre[3] + x_now[4] - x_pre[4] ) * 0.5;
		H << 1,                 0,                    0,                    0,       0,
		     h_1_0 / tmp * tmp, 0,                    0,                    1 / tmp, 1 / tmp,
		     0,         (1 / h_measurement[2]) * x_now[1], (1 / h_measurement[2]) * x_now[2], 0,       0;

		std::cout<<"H = "<<std::endl<<H<<std::endl;	
	
		// Kalman gain
		Matrix3 K_tmp = H * P_now * H.transpose() + R;
		Matrix5x3 K = P_now * H.transpose() * K_tmp.inverse();

		// state update
		x_now += K * error;
		// state covarince update
		P_now = ( Matrix5::Identity() - K * H ) * P_now;
	
		std::cout<<"update end"<<std::endl;
	}

	const Vector5& getStateX() const
	{
		return x_now;
	}
	
	void setStateNoiseQ( const Matrix5 &Q_ ) 
	{
		Q = Q_;
	}	
	
	void setMeasurementNoise( const Matrix3 &R_ )
	{
		R = R_;
	}

private:
	// control vector at k-1 moment, ( a_x, a_y, w )
	Vector3 u_pre = Vector3::Zero();
	// state vector at k-1 moment, ( q, v_x, v_y, p_x, p_y )
	Vector5 x_pre = Vector5::Zero();
	// state vector at k moment
	Vector5 x_now = Vector5::Zero();
	// state covarince matrix
	Matrix5 P_pre = Matrix5::Identity();
	Matrix5 P_now = Matrix5::Identity();

	// state Jacobian matrix
	Matrix5 F = Matrix5::Identity();
	
	// state Gaussian Noise
	Matrix5 Q = Matrix5::Identity();
	// measurement Gaussian Noise
	Matrix3 R = Matrix3::Identity();

	// previous time moment
	DataType time_pre = 0;
	
	// 
	bool is_init = false;
};

}

#endif
