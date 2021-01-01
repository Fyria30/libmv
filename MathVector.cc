#include "MathVector.h"
#include <cmath>
#include <cstring>


MathVector::MathVector() {
    x = 0;
    y = 0;
    lenght = 0;
}

MathVector::MathVector(Point a, Point b) {
    x = b.x - a.x;
    y = b.y - a.y;
    lenght = sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

MathVector MathVector::operator + (MathVector vector) {
    vector.x += this->x;
    vector.y += this->y;
    return vector;
}
Point MathVector::operator + (Point point) {
    point.x += this->x;
    point.y += this->y;
    return point;
}

double MathVector::operator * (MathVector v){
return this->x * v.x + this->y * v.y;
}

//TODO change to template
MathVector MathVector::operator * (double scale) {
    MathVector result;
    result.x = this->x * scale;
    result.y = this->y * scale;
    return result;
}

//TODO change to template
MathVector MathVector::operator / (double scale) {
    MathVector result;
    result.x = this->x / scale;
    result.y = this->y / scale;
    return result;
}

double MathVector::GetAngleToOrtoi() {
    MathVector orti{ Point{0,0}, Point{1,0} };
    return this->GetAngleBetweenVectors(orti);
}

double  MathVector::GetAngleBetweenVectors(MathVector vec)
{
    if (vec.lenght == 0 || this->lenght == 0)
        return 0;

    double t = (x * vec.x + y * vec.y) / (sqrt((double)pow(x, 2) + pow(y, 2)) * sqrt((double)pow(vec.x, 2) + pow(vec.y, 2)));

    if (t < -1) t = -1;
    else if (t > 1) t = 1;

    double r = acos(t) * 180 / M_PI;
    if (y < vec.y)
        return r;
    else
        return 360 - r;
}

double  MathVector::getAngle(MathVector vec){
    return acos (*this * vec /(this->lenght* vec.lenght) );
}

MathVector MathVector::normilizeVec() {

    double norm = this->lenght;
    if (norm == 0) {
        //cout << "The norm of the vector is 0" << endl;
        return MathVector();
    }
    MathVector result = *this / norm;
    return result;
}