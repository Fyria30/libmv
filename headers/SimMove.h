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
vector <Point>  m_map;
vector <Point>  m_llaCoordinates;
vector <Point>  m_ghostCar;//for test
vector <double> m_distanceMap;

//----- Methods -----
Point GetProectionToLine(Point start_segment, Point end_segment, Point out_point);
bool isPointInSegment(Point start_segment, Point end_segment, Point out_point);
SimMove_Status findProection(Point out_point, Point& ref_proection, int& ref_segment );
SimMove_Status devideMapIntoSegments();
void saveIntoFileGhostCar(const char* fileName);
SimMove_Status findNearestTop(Point point , Point& ref_top, int& ref_segment_where_the_point_is);
SimMove_Status calculateGhostPoint(Point& ref_point , Point& ref_orientation, int64_t& ref_segment, Point& ref_pointOnline, double speed, double time, double& ref_lenght );

//----- API -----
SimMove_Status getGhostPoint(MV_ObjectPositioning& data_object, int64_t& ref_segment, double time );
SimMove_Status loadMapFromFileENU(const char* fileName);
SimMove_Status enuParserLoad(const char* fileName);
SimMove_Status llaParserLoad(const char* fileName);
SimMove_Status saveRawIntoFile(const char* fileName);
SimMove_Status mapGen();
SimMove_Status init(const char* fileName);
};
