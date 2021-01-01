#pragma once
#define M_PI 3.14159265358979323846

struct Point {
	double x;
	double y;
	double z;
};
enum MathVector_Status {
    MathVector_OK = 0,			// success
    MathVector_ERROR	// some error occures
};

class MathVector {
public:
    //Members
    double x;
    double y;
    double lenght;

    //Constructors
    MathVector(Point, Point);
    MathVector();
    
    //Overloaded operations
    MathVector operator + (MathVector);
    double operator * (MathVector);
    Point operator + (Point);
    MathVector operator * (double scale);
    MathVector operator / (double scale);
    
    //Methods
    double GetAngleBetweenVectors(MathVector);
    double GetAngleToOrtoi();
    MathVector normilizeVec();
    double getAngle(MathVector vec);
};