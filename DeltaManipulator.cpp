#include "DeltaManipulator.h"
#include <math.h>

 
 // trigonometric constants
 const double sqrt3 = sqrt(3.0);
 const double pi = 3.141592653;    // PI
 const double sin120 = sqrt3/2.0;   
 const double cos120 = -0.5;        
 const double tan60 = sqrt3;
 const double sin30 = 0.5;
 const double tan30 = 1/sqrt3;


 
DeltaManipulator::DeltaManipulator()
{
	this->e = 115.0;
	this->f = 457.3;
	this->re = 232.0;
	this->rf = 112.0;
}

DeltaManipulator::DeltaManipulator( double e, double f, double re, double rf )
{
	this->e = e;
	this->f = f;
	this->re = re;
	this->rf = rf;
}

void DeltaManipulator::Confogure( double e, double f, double re, double rf ) {
	this->e = e;
	this->f = f;
	this->re = re;
	this->rf = rf;
}
	
DeltaManipulator::~DeltaManipulator()
{}

// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
int DeltaManipulator::Forward(double theta1, double theta2, double theta3, vec3f& pos ) const
{
	double t = (f-e)*tan30/2;
	double dtr = pi/(double)180.0;
 
	theta1 *= dtr;
	theta2 *= dtr;
	theta3 *= dtr;
 
	double y1 = -(t + rf*cos(theta1));
	double z1 = -rf*sin(theta1);
 
	double y2 = (t + rf*cos(theta2))*sin30;
	double x2 = y2*tan60;
	double z2 = -rf*sin(theta2);
 
	double y3 = (t + rf*cos(theta3))*sin30;
	double x3 = -y3*tan60;
	double z3 = -rf*sin(theta3);
 
	double dnm = (y2-y1)*x3-(y3-y1)*x2;
 
	double w1 = y1*y1 + z1*z1;
	double w2 = x2*x2 + y2*y2 + z2*z2;
	double w3 = x3*x3 + y3*y3 + z3*z3;
     
	// x = (a1*z + b1)/dnm
	double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
	double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
	// y = (a2*z + b2)/dnm;
	double a2 = -(z2-z1)*x3+(z3-z1)*x2;
	double b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
	// a*z^2 + b*z + c = 0
	double a = a1*a1 + a2*a2 + dnm*dnm;
	double b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
	double c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
	// discriminant
	double d = b*b - (double)4.0*a*c;
	if (d < 0) return -1; // non-existing point
 
	pos.z = -(float)0.5*(b+sqrt(d))/a;
	pos.x = (a1*pos.z + b1)/dnm;
	pos.y = (a2*pos.z + b2)/dnm;
	return 0;
}
 
// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
bool DeltaManipulator::calcAngleYZ( vec3f& pos, double &theta) const {
	float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
	pos.y -= 0.5 * 0.57735    * e;    // shift center to edge
	// z = a + b*y
	float a = (pos.x*pos.x + pos.y*pos.y + pos.z*pos.z +rf*rf - re*re - y1*y1)/(2*pos.z);
	float b = (y1-pos.y)/pos.z;
	// discriminant
	float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
	if (d < 0)
		return false; // non-existing point
	float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
	float zj = a + b*yj;
	theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
	return true;
}
 
 // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
 // returned status: 0=OK, -1=non-existing position
bool DeltaManipulator::Inverse( const vec3f& pos, double &theta1, double &theta2, double &theta3 ) const
{
	theta1 = theta2 = theta3 = 0;
	vec3f pos1 = pos;
	vec3f pos2( pos.x*cos120 + pos.y*sin120, pos.y*cos120 - pos.x*sin120, pos.z );
	vec3f pos3( pos.x*cos120 - pos.y*sin120, pos.y*cos120 + pos.x*sin120, pos.z );

	if ( !calcAngleYZ( pos1, theta1 ) ) {
		return false;
	}
	if ( !calcAngleYZ( pos2, theta2 ) ) {
		return false;
	}

	return calcAngleYZ( pos3, theta3 );
 }