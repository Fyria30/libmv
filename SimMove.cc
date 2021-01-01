#include "SimMove.h"


using namespace rapidjson;

//Point refPoint{52.2370937,-0.4684315,88.9243}; //TODO find out where the reference point is and if it's right
//Point refPoint{55.5647425,37.9911456,0.0}; //LV
Point refPoint;
geodetic_converter::GeodeticConverter geodeticConverter;

Point SimMove::GhostPoint(Point a, Point b, double lenght, double wight)
{
    Point result{ 1,1 };
    MathVector v{ a,b };
    cout << "Angle:" << v.GetAngleToOrtoi() << endl;
    return result;
}

Point SimMove::GetProection(Point p1, Point p2, Point p3)
{
    Point result;
    double lenght_x, lenght_y;
    double t;

    lenght_x = p2.x - p1.x;
    lenght_y = p2.y - p1.y;
    t = (lenght_x * p3.x + lenght_y * p3.y - lenght_x * p1.x - lenght_y * p1.y) / (pow(lenght_x, 2) + pow(lenght_y, 2));
    result.x = lenght_x * t + p1.x;
    result.y = lenght_y * t + p1.y;
    return result;
}

bool SimMove::checkPoints(Point a, Point b, Point check) {
    double ab_lenght = MathVector{ a, b }.lenght;
    double ac_lenght = MathVector{ a, check }.lenght;
    double bc_lenght = MathVector{ b, check }.lenght;

    if ((pow(ac_lenght, 2) > (pow(bc_lenght, 2) + pow(ab_lenght, 2))) //+ 0.1
        || (pow(bc_lenght, 2) > (pow(ac_lenght, 2) + pow(ab_lenght, 2)))) {
        return false;
    }
    return true;
}
//TO DO refactoring
SimMove_Status SimMove::findProection(Point check, Point &a_proection, int &a_segment) {
    // searching for all possible elements
    size_t i = 0;
    vector<int> segments;
    Point result;
    int segment_count;




    for (; i < map.size() - 1; i++) {
        if (checkPoints(map[i], map[i + 1], check)) {
            segments.push_back(i);
        }
    }
    //check the last segment(last and start point), which mathimatically is not in the map.
    if (checkPoints(map[map.size()-1], map[0], check)) {
            segments.push_back(i);
            // add segment 
        }

    if (segments.size() == 0) {
        cout << "The segment is not found, the nearest top is takken" << endl;
        findNearestTop(check,result, segment_count);       
        a_segment = segment_count;
        a_proection = result;
        return SimMove_OK;
    }


    
    double lenght =-1;
    if (segments.size() == 1) {
       segment_count = segments[0];
        //TODO: add here the last segment
       result = GetProection(map[segments[0]], map[segments[0] + 1], check);
       lenght = MathVector{check, result}.lenght;
        //return SimMove_OK;
    }

    // if not only one -> compare lenght; TODO refactoring
    Point temp;
    for (vector<int>::iterator it = segments.begin(); it != segments.end(); it++) {
        if((size_t)*it!=map.size()-1)
        temp = GetProection(map[*it], map[*it + 1], check);
        else 
        temp = GetProection(map[*it],map[0], check);

        if (it == segments.begin()) {
            result = temp;
            segment_count = segments[0];
            lenght = MathVector{ check,temp }.lenght;
        }
        //cout<<"Possible proection point "<< temp.x <<" ; "<< temp.y << endl;
        if (MathVector{ check, result }.lenght > MathVector{ check,temp }.lenght) {
            result = temp;
            segment_count = *it;
            lenght = MathVector{ check,temp }.lenght;
        }

    }
   // cout << "Total proection point: (" << result.x << " ; " << result.y << ") in " << segment_count << " segment " << endl;

//Big lenght means there is no segments in the near -> so we have to take the nearest top
    if(lenght > 5){
        findNearestTop(check, result, segment_count); //TODO: we do do not take into account last point. If there is last point -> the last segment is last point -1
        cout<<"The proection is too far, the nearest point is takken Segment:"<< segment_count <<endl;
        }

    a_segment = segment_count;
    a_proection = result;


    return SimMove_OK;

}

SimMove_Status SimMove::findNearestTop(Point point, Point & a_top, int &a_pointNumber){
    MathVector vec;
    Point temp;
    double lenght = numeric_limits<double>::max();
    int pointNum;
    for(size_t i =0;i<map.size();i++){
        vec  = MathVector(point, map[i]);
        if (vec.lenght < lenght){
        lenght = vec.lenght;
        temp = map[i];
        pointNum = i;

        }
    }
    a_top = temp;
    a_pointNumber = pointNum+1;
    return SimMove_OK;
}
SimMove_Status SimMove::getGhostPoint(MV_ObjectPositioning& object, double time, int64_t& a_segment){
    SimMove_Status status;
    Point enuPoint;
	if (!geodeticConverter.isInitialised()) {
          return SimMove_ERROR;
     }

    //if the first time (a lgorithm flag)-> calculate
    if (!object.positioning.algorithm.flagAlgorithm){
        	geodeticConverter.geodetic2Enu((double)(object.positioning.position[1]/(10000000.0)),
                                   (double)(object.positioning.position[0]/(10000000.0)),
                                   (double)(object.positioning.position[2]/(100.0)),
                                   &enuPoint.x,
                                   &enuPoint.y,
                                   &enuPoint.z);
            object.positioning.algorithm.lenght = DOUBLE_NULL;
            object.positioning.algorithm.flagAlgorithm = true;
    }else
    {
        enuPoint.x = object.positioning.algorithm.enu[0];
        enuPoint.y = object.positioning.algorithm.enu[1];
        enuPoint.z = object.positioning.algorithm.enu[2];
    }
    

    double speed;	   
    int64_t segment;
    Point orientationPoint; 
    orientationPoint.x = object.positioning.orientation[0];
    orientationPoint.y = object.positioning.orientation[1];
    orientationPoint.z = object.positioning.orientation[2];
    speed=sqrt(pow(object.positioning.velocity[0], 2) + pow(object.positioning.velocity[1], 2));

    if (speed == 0.0){
        return SimMove_OK;
    }

    cout<< "Speed:" << speed<<endl;
    status = calculateGhostPoint(enuPoint, orientationPoint,  speed, time, object.positioning.algorithm.lenght, segment); // Put references sign
    
    if(status!=SimMove_OK){
        return status;
    }
    
    // if the point has crossed the final line ->  do not change the data
   // if (object.positioning.finishSegment>segment)
    //      object.positioning.finishSegment=-1;

    //convert back to LLA
    Point llaPoint;
    geodeticConverter.enu2Geodetic(enuPoint.x, enuPoint.y, enuPoint.z ,&llaPoint.y,&llaPoint.x,&llaPoint.z);

    
    //geodeticConverter.enu2Geodetic(map[i].x, map[i].y, map[i].z ,&llaPoint.y,&llaPoint.x,&llaPoint.z);
  /*//test
    Point BackEnu;
    Point test;
    test.x = llaPoint.x * 10000000;
    test.y = llaPoint.y * 10000000;
    test.z = llaPoint.z * 100;
    
    	geodeticConverter.geodetic2Enu((double)(test.y/(10000000.0)),
                                      (double)(test.x/(10000000.0)),
                                      (double)(test.z/(100.0)),
                                   &BackEnu.x,&BackEnu.y,&BackEnu.z);
    cout<<"Before "<< " x: "<<enuPoint.x<< " y: " << enuPoint.y<<" z: " << enuPoint.z<< endl;
    cout<<"Back "<< " x: "<< BackEnu.x<< " y: " << BackEnu.y<<" z: " << BackEnu.z << endl;
    //cout<< " dx: "<<enuPoint.x - oldEnu.x<< " y: " << enuPoint.y - oldEnu.y<<" z: " << enuPoint.z - oldEnu.z << endl;
  */

    a_segment = segment;
    // write it back
    object.positioning.algorithm.enu[0] = enuPoint.x;
    object.positioning.algorithm.enu[1] = enuPoint.y;
    object.positioning.algorithm.enu[2] = enuPoint.z;

    object.positioning.orientation[0] = orientationPoint.x; 
    object.positioning.orientation[2] = orientationPoint.y;
    object.positioning.orientation[3] = orientationPoint.z;

    object.positioning.position[0] = llaPoint.x * 10000000;
    object.positioning.position[1] = llaPoint.y * 10000000;
    object.positioning.position[2] = llaPoint.z * 100;

    return SimMove_OK;
}


SimMove_Status SimMove::calculateGhostPoint(Point& a_check, Point& a_orintation, double speed, double time, double& lenghtFromCenterLine, int64_t& a_segment) {
    Point check = a_check;
    SimMove_Status status;

    // due to accuracy the first iteration will be changed. so we need to return if speed is zero explicitly;
    if(speed ==0)
    return SimMove_OK;

   
    //Find a point proection on the center line. Find a segment where the point is
    Point proectionPoint{0,0,0};
    int segment_count=-1;
    
    status = findProection(check, proectionPoint,segment_count);
    //cout << "The point: "<< check.x <<" "<< check.y <<" " << check.z << endl;
    //cout<< "First vector: "<< map[segment_count].x <<";" <<map[segment_count].y<<"  " << map[segment_count+1].x <<";" <<map[segment_count+1].y<<endl;
    //cout << "First proection point: "<< proectionPoint.x <<" "<< proectionPoint.y <<" " << proectionPoint.z << endl;
    //cout << "First segment: " << segment_count << endl;
    if (status!= SimMove_OK)
        return status;

    //Move the point on needed distance (according time and speed) along the center line
    //auto detecting the distance and use the same distance for this iteration
    if (lenghtFromCenterLine == DOUBLE_NULL){
    //find out the sign
         MathVector ab {map[segment_count],map[segment_count+1]};
         MathVector ac {map[segment_count],check};
         lenghtFromCenterLine = MathVector{check, proectionPoint}.lenght;

         if(ab.GetAngleBetweenVectors(ac) < 180.0){
             lenghtFromCenterLine = -lenghtFromCenterLine;
             }      
        }
    

    double lenghtToStartSegment = MathVector{ proectionPoint, map[segment_count]}.lenght;
    double nowDistance = distanceMap[segment_count] + lenghtToStartSegment;
    double newDistance = nowDistance + speed * time;
    double totalDistance = *(distanceMap.end() - 1);

    //check map. Go to a new lap if the map is over
    while (newDistance > totalDistance) {
        newDistance -= totalDistance;
    }
     cout << "Now distance: " << nowDistance << "   New distance: " << newDistance << endl;

     //find a segment
     //TODO: easy to speed up
    size_t seg = 0;
    for (; seg < distanceMap.size(); seg++) {
        if (distanceMap[seg] <= newDistance && distanceMap[seg + 1] >= newDistance)
            break;
    }  
    double achivedDistance = distanceMap[seg];
    double restDistance = newDistance - achivedDistance;
    //cout << "Distance to the final point on the line: " << restDistance << endl;

    // Get a new point on the line;
    MathVector vec = MathVector(map[seg], map[seg + 1]);
    vec = vec.normilizeVec();
    Point res = vec * restDistance + map[seg]; // here the point which we were finding  
    //cout << "Second segment: " << seg <<endl;  
    //cout << "Second vector: " << map[seg].x <<";" <<map[seg].y<<"  " << map[seg+1].x <<";" <<map[seg+1].y<<endl;
    //cout << "Proection on line: " << res.x << " " << res.y << " " << endl;

    //Make a perpendicular of the point to get a point perpendicular to the center line (left or right)
    MathVector foundSegment { map[seg] ,map[seg + 1]};
    MathVector vec2 = MathVector(map[seg], map[seg + 1]);
    vec2 = vec2.normilizeVec();

    if (lenghtFromCenterLine < 0) {
        double temp = vec2.y;
        vec2.y = vec2.x;
        vec2.x = -temp;
    }
    if (lenghtFromCenterLine > 0) {
        double temp = vec2.x;
        vec2.x = vec2.y;
        vec2.y = -temp;
    }

    Point ghostPoint = vec2 * abs(lenghtFromCenterLine) + res; // here the point which we were finding
    cout << "FouneSegmentVector: " << foundSegment.x << " " << foundSegment.y << endl;
    cout << "Ghost point: (" << ghostPoint.x << " ; " << ghostPoint.y << " ; "<< ghostPoint.z<<")" << " Segment:" << seg << endl;
    a_check = ghostPoint;
    a_segment = seg;
    //a_orintation.x = foundSegment.GetAngleToOrtoi()-180;              // TODO make the last segment workable 

MathVector orti{Point{0,0},Point{0,1}};

double dot = foundSegment.x*orti.x + foundSegment.y*orti.y ;    
double det = foundSegment.x*orti.y - foundSegment.y*orti.x ;     
double angle = atan2(det, dot) * 180/M_PI;

    a_orintation.x =  angle;//foundSegment.GetAngleBetweenVectors(MathVector{Point{0,0},Point{1,0}}) -180; // 
    cout << "Angle: "<< angle << endl;
    //a_orintation.y = 
    //a_orintation.z = 


    ghostCar.push_back(ghostPoint); // for check
    //a_orintation.x =  foundSegment.GetAngleToOrtoi();
    cout<<endl<<endl;
    return SimMove_OK;
}

void SimMove::saveIntoFileGhostCar(const char* fileName)
{
    ofstream outf(fileName);
    if (!outf)
        exit(1);
  
    outf.precision(5);
    for (vector<Point>::iterator it = ghostCar.begin(); it != ghostCar.end(); ++it)
        outf << it->x << ";" << it->y << ";" << endl;
}

SimMove_Status SimMove::init(const char* fileName) {

//init converter it will be needed into parser and map creating
SimMove_Status status;
//Parse the map into map vector
status = llaParserLoad(fileName);
if (status != SimMove_OK)
return status;
//creating segments into vector

status = devideMapIntoSegments();
if (status != SimMove_OK)
return status;



return SimMove_OK;    
}

SimMove_Status SimMove::loadMapFromFileENU(const char* fileName) {
    ifstream file(fileName);
    if (!file)
    {
        cout << "Can not load " <<fileName<< endl;
        return SimMove_ERROR;
    }

    vector<double> values;
    copy(istream_iterator<double>(file), istream_iterator<double>(), back_inserter(values));
    file.close();
    map.clear();
    for (size_t i = 0; i < values.size(); i = i + 2){        
        Point a;
        a.x = values[i];
        a.y = values[i + 1];
        map.push_back(a);
    }
    //cout <<"The last element" << *(distanceMap.end() - 1);
    return SimMove_OK;
    
}

SimMove_Status SimMove::saveRawIntoFile(const char* fileName)
{
	ofstream outf(fileName);
 
	if (!outf){
		cout << "Can not open file" << endl;
		return SimMove_ERROR;
	}
    outf.precision(5);
    for (vector<Point>::iterator it = map.begin() ; it!=map.end() ; ++it){
        outf<< it->x<<";"<< it->y<<";"<<endl;
    }
        return SimMove_OK;
}

SimMove_Status SimMove::devideMapIntoSegments(){
    distanceMap.clear();
    distanceMap.push_back(0.0);
    double tempDistance = 0;
    for (size_t i = 1; i < map.size(); i++)
    {
        tempDistance += MathVector{ map[i - 1], map[i] }.lenght;
        distanceMap.push_back(tempDistance);
    }
    m_mapSize = map.size();
    //cout <<"The last element" << *(distanceMap.end() - 1);
    return SimMove_OK;

}
SimMove_Status SimMove::enuParserLoad(const char* fileName){
    Document document;   
    ifstream ifs(fileName);
    IStreamWrapper isw(ifs); 

    if (document.ParseStream(isw).HasParseError() || !document.IsObject()) {
    cout<<"JSON parse error!";// TODO
				}
    //cout<< "Is array:" << document["result"]["extras"]["rrMapSource"].IsArray() << endl;
    //cout<< document.HasMember("result");

    const char *mapString = document["result"]["extras"]["rrMapSource"].GetString();

    Document mapParser;
    mapParser.Parse(mapString);

    const rapidjson::Value& b = mapParser["Centre"];
    Point mapPoint;
    map.clear();
    for (rapidjson::SizeType i = 0; i < b.Size(); i++)
    {
       const rapidjson::Value& a = b[i];
      //cout<<"!!!!!!!!!!!!!!!!! "<< a.Size()<<endl;

      mapPoint.x= a[rapidjson::SizeType(0)].GetDouble();
      mapPoint.y= a[rapidjson::SizeType(1)].GetDouble();
      map.push_back(mapPoint);
    }

    return SimMove_OK;
}

SimMove_Status SimMove::llaParserLoad(const char* fileName){
//Parse the lla into array
Document document;   
ifstream ifs(fileName);
IStreamWrapper isw(ifs); 

if(!ifs){
cout<< "Can not open the file: "<< fileName << endl;
return SimMove_ERROR;
}

if (document.ParseStream(isw).HasParseError() || !document.IsObject()) {
cout<<"Json parse error. Can not parse the file: " << fileName;
return SimMove_ERROR;
}

Value& w = document["result"]["features"][2]["geometry"]["coordinates"];

if(w.Size()==0 || !w.IsArray())
{
cout<< "Nothing to parse in "<< fileName << endl;
return SimMove_ERROR;
}

for(size_t i=0;i<w.Size();i++){
        Point temp;
        temp.x = w[i][1].GetDouble();
        temp.y = w[i][0].GetDouble();
        temp.z = w[i][2].GetDouble();
        llaCoordinates.push_back(temp);
        //cout<<"Number: " << w[i][j].IsNumber() << endl;
    }

//TODO: be carefull with points. x=y, 
refPoint.x = llaCoordinates[0].x;
refPoint.y = llaCoordinates[0].y;
refPoint.z = llaCoordinates[0].z;
geodeticConverter.initialiseReference(refPoint.x,refPoint.y ,refPoint.z );
//Convert to enu and write the map
   // ofstream outf("llatest.txt");

 // outf.precision(9);
//outf<<"[";
for (size_t i =0 ; i<llaCoordinates.size();i++){
    
		if (geodeticConverter.isInitialised()) {
                Point enuPoint;
				geodeticConverter.geodetic2Enu(llaCoordinates[i].x, llaCoordinates[i].y,llaCoordinates[i].z, &enuPoint.x,&enuPoint.y,&enuPoint.z);
               // outf<<"["<< llaCoordinates[i].y << "," << llaCoordinates[i].x << "," << llaCoordinates[i].z <<"],"<< endl;
                map.push_back(enuPoint);
            }
            
    }
    //outf<<"]";
//Write the enu array

//Convert back enu-lla, do not need here, just for test
llaCoordinates.clear();
Point llaPoint;
for (size_t i =0; i<map.size();i++){
		if (geodeticConverter.isInitialised()) {
				geodeticConverter.enu2Geodetic(map[i].x, map[i].y, map[i].z ,&llaPoint.y,&llaPoint.x,&llaPoint.z);
                llaCoordinates.push_back(llaPoint);
            }
    }

    cout<< "Map Size: "<< map.size()<< endl ;
    return SimMove_OK;
}
SimMove_Status SimMove::mapGen() {
    /*
    Point a{ 0,0,0 };
    Point b{ 4,0,0 };
    Point c{ 5,1,0 };
    Point d{ 4,4,0 };
    Point e{ 4,5,0 };
    Point f{ 1,5,0 };
    Point g{ 4,0,0 };
    Point h{ 0,0,0 };
    */
    //genetare the map
   /* Point a{ 0,0,0 };
    Point b{ 5,0,0 };
    Point c{ 5,5,0 };
    Point d{ 0,5,0 };
    Point e{ 0,0,0 };
   */
    Point a{ 97.435,10.305,0 };
    Point b{ 96.96,10.15,0 };
    Point c{ 96.009,9.99,0 };
    map.push_back(a);
    map.push_back(b);
    map.push_back(c);
    //map.push_back(d);
   /// map.push_back(e);
   // map.push_back(f);
   // map.push_back(g);
   // map.push_back(h);

    // split the track lenght into segments
    distanceMap.push_back(0.0);
    double tempDistance = 0;
    for (size_t i = 1; i < map.size(); i++)
    {
        tempDistance += MathVector{ map[i - 1],map[i] }.lenght;
        distanceMap.push_back(tempDistance);
    }
    cout << "Simulation lib initialization is completed"<< endl;
    return SimMove_OK;
}

