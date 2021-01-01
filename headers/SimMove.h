#pragma once
#include <iostream>
#include <cmath> 
#include <ctime>
#include <random>
#include <fstream>
#include "MathVector.h"
#include <iterator>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filereadstream.h"
#include <rapidjson/istreamwrapper.h>
#include "geodetic-converter.h"
#include "../rr-v2x-mv-lib/include/libv2xmv.h"
#include <limits>

using namespace std;
#define DOUBLE_NULL numeric_limits<double>::min()
enum SimMove_Status {
    SimMove_ERROR = -1,    // some error occures
    SimMove_OK = 0         // success
};
class SimMove {

public:
//----- Members -----
vector<Point> map;
vector<double> distanceMap;
int m_mapSize;
vector <Point> ghostCar;//for test
vector <Point> llaCoordinates;
//MV_Spatial checkPoint;

//----- Methods -----
Point GhostPoint(Point, Point, double, double);
Point GetProection(Point, Point, Point);
bool checkPoints(Point, Point, Point);
SimMove_Status findProection(Point, Point&, int& );
SimMove_Status devideMapIntoSegments();
void saveIntoFileGhostCar(const char* fileName);
SimMove_Status findNearestTop(Point point, Point & a_top, int &a_pointNumber);
SimMove_Status calculateGhostPoint(Point&, Point&, double, double, double&, int64_t&);

//----- API -----
SimMove_Status getGhostPoint(MV_ObjectPositioning&, double, int64_t&);
SimMove_Status loadMapFromFileENU(const char* fileName);
SimMove_Status enuParserLoad(const char* fileName);
SimMove_Status llaParserLoad(const char* fileName);
SimMove_Status saveRawIntoFile(const char* fileName);
SimMove_Status mapGen();
SimMove_Status init(const char* fileName);
};