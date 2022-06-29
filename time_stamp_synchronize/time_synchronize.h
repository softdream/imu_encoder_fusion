#ifndef __TIME_SYNCHRONIZE_H
#define __TIME_SYNCHRONIZE_H

#include <vector>

#include <Eigen/Dense>

namespace time_stamp
{
	
template<typename T, int Size = 20>
class Queue
{
public:
	Queue()
	{
		buffer = new T[Size];
	}

	~Queue()
	{
		delete[] buffer;
	}

	bool isEmpty() const
	{
		return front == rear;
	}
	
	bool isFull() const
	{
		return ( ( rear + 1 ) % Size ) == front;
	}

	bool push( const T &elem )
	{
		if( isFull() ){
			std::cerr << "Queue is full !" << std::endl;
                        return false;
		}

		buffer[rear] = elem;
		rear = ( rear + 1 ) % Size;

		return true;
	}

	bool pop()
	{
		if (isEmpty()) {
                        std::cerr << "Queue is empty !" << std::endl;
                        return false;
                }

                front = (front + 1) % Size;

                return true;
	}

	const T find( const double time_stamp )
	{
		double dist = std::numeric_limits<double>::max();
		T ret;
		for( int i = front; i < rear; i ++ ){
		//	std::cout<<"buffer["<<i<<"] : "<<std::endl<<buffer[i]<<std::endl;
			if( std::abs( time_stamp - buffer[i](0) ) <= dist){
				dist = std::abs( time_stamp - buffer[i](0) );
				ret = buffer[i];
			
		//		std::cout<<"dist ["<<i<<"] = "<<dist<<std::endl;
			}
		}	

		return ret;
	}

private:	
	T* buffer = nullptr;
	int front = 0;
	int rear = 0;
};

class Synchronize
{
public:
	using Vector4 = typename Eigen::Matrix<double, 4, 1>;


	Synchronize()
	{

	}

	~Synchronize()
	{

	}
	
	

	void addAData( const Vector4& elem )
	{
		if( !que.isFull() ){
			que.push( elem );
		}
		else {
			que.pop();
			que.push( elem );
		}
	}

	const Vector4 findAData( const double time_stamp )
	{
		return que.find( time_stamp );
	}

private:
	Queue<Vector4> que;

};

}

#endif
