#pragma once
#include <math.h>

class vec3f {
public:
	float x;
	float y;
	float z;

	vec3f( float x=0.0f, float y=0.0f, float z=0.0f ) {
		this->x = x;
		this->y = y;
		this->z = z;
	}
	inline float lengthSquared() { return x*x + y*y + z*z; }
	inline float length() { return sqrt(lengthSquared()); }
};

class DeltaManipulator
{
public:
	DeltaManipulator();
	DeltaManipulator( double e, double f, double re, double rf );
	~DeltaManipulator();

	void Confogure( double e, double f, double re, double rf );

	int Forward( double theta1, double theta2, double theta3, vec3f& pos ) const;
	bool Inverse( const vec3f& pos, double &theta1, double &theta2, double &theta3 ) const;
protected:
	bool calcAngleYZ( vec3f& pos, double &theta ) const;
private:

	double e;     // end effector
	double f;     // base
	double re;
	double rf;
};