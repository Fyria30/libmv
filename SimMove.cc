#include "SimMove.h"


using namespace rapidjson;

Point refPoint;
geodetic_converter::GeodeticConverter geodeticConverter;

Point SimMove::GetProectionToLine(Point start_point, Point end_point, Point out_point) // get the proection to the line
{
    Point result;
    double lenght_x, lenght_y;
    double t;

    lenght_x = end_point.x - start_point.x;
    lenght_y = end_point.y - start_point.y;
    t = (lenght_x * out_point.x + lenght_y * out_point.y - lenght_x * start_point.x - lenght_y * start_point.y) / (pow(lenght_x, 2) + pow(lenght_y, 2));
    result.x = lenght_x * t + start_point.x;
    result.y = lenght_y * t + start_point.y;
    return result;
}

bool SimMove::isPointInSegment(Point start_point, Point end_point, Point out_point) { //Check if the point in the segment's range
    double ab_lenght = MathVector{ start_point, end_point }.lenght;
    double ac_lenght = MathVector{ start_point, out_point }.lenght;
    double bc_lenght = MathVector{ end_point, out_point }.lenght;

    if ((pow(ac_lenght, 2) > (pow(bc_lenght, 2) + pow(ab_lenght, 2))) //+ 0.1
        || (pow(bc_lenght, 2) > (pow(ac_lenght, 2) + pow(ab_lenght, 2)))) {
        return false;
    }

    return true;
}

//TO DO refactoring
// Devide into two functions. Find Proection. Find segment
SimMove_Status SimMove::findProection(Point check, Point &ref_proection, int &ref_segment) {
    // searching for all possible elements
    size_t i = 0;
    vector<int> segments;
    Point result;
    int segment_count;

    for (; i < m_map.size() - 1; i++) {
        if (isPointInSegment(m_map[i], m_map[i + 1], check)) {
            segments.push_back(i);
        }
    }
    //check the last segment(last and start point), which mathimatically is not in the map.
    if (isPointInSegment(m_map[m_map.size()-1], m_map[0], check)) {
            segments.push_back(i);
        }

    if (segments.size() == 0) {
        cout << "The segment is not found, the nearest top is takken" << endl;
        findNearestTop(check,result, segment_count);       
        ref_segment = segment_count;
        ref_proection = result;
        return SimMove_OK;
    }

    double lenght =-1;

    if (segments.size() == 1) {
            segment_count = segments[0];
            //TODO: add here the last segment
            result = GetProectionToLine(m_map[segments[0]], m_map[segments[0] + 1], check);
            lenght = MathVector{check, result}.lenght;
    }

    // if not only one -> compare lenght; TODO refactoring
    Point temp;
    for (vector<int>::iterator it = segments.begin(); it != segments.end(); it++) {
        if((size_t)*it != m_map.size()-1){
            temp = GetProectionToLine(m_map[*it], m_map[*it + 1], check);}
        else{
            temp = GetProectionToLine(m_map[*it],m_map[0], check);}

        if (it == segments.begin()){
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

    ref_segment = segment_count;
    ref_proection = result;

    return SimMove_OK;

}

SimMove_Status SimMove::findNearestTop(Point point, Point& ref_top, int& ref_pointNumber){
    MathVector vec;
    Point temp;
    double lenght = numeric_limits<double>::max();
    int pointNum;

    for(size_t i =0;i<m_map.size();i++){
        vec  = MathVector(point, m_map[i]);
        if (vec.lenght < lenght){
            lenght = vec.lenght;
            temp = m_map[i];
            pointNum = i;
        }
    }

    ref_top = temp;
    ref_pointNumber = pointNum+1;
    return SimMove_OK;
}
SimMove_Status SimMove::getGhostPoint(MV_ObjectPositioning& ref_object, int64_t& ref_segment, double time){
    SimMove_Status status;
    Point enuPoint;
    Point enuOnLine;
	if (!geodeticConverter.isInitialised()) {
          return SimMove_ERROR;
     }

    //if the first time (a lgorithm flag)-> calculate
    if (!ref_object.positioning.algorithm.flagAlgorithm){
        geodeticConverter.geodetic2Enu((double)(ref_object.positioning.position[1]/(10000000.0)),
                                       (double)(ref_object.positioning.position[0]/(10000000.0)),
                                       (double)(ref_object.positioning.position[2]/(100.0)),
                                       &enuPoint.x,
                                       &enuPoint.y,
                                       &enuPoint.z);
        ref_object.positioning.algorithm.lenght = DOUBLE_NULL;
        ref_object.positioning.algorithm.flagAlgorithm = true;}
    else{
        enuPoint.x = ref_object.positioning.algorithm.enu[0];
        enuPoint.y = ref_object.positioning.algorithm.enu[1];
        enuPoint.z = ref_object.positioning.algorithm.enu[2];

        enuOnLine.x = ref_object.positioning.algorithm.enuPointOnLine[0];
        enuOnLine.y = ref_object.positioning.algorithm.enuPointOnLine[1];
        enuOnLine.z = ref_object.positioning.algorithm.enuPointOnLine[2];
    }

       
    
    Point orientationPoint; 
    orientationPoint.x = ref_object.positioning.orientation[0];
    orientationPoint.y = ref_object.positioning.orientation[1];
    orientationPoint.z = ref_object.positioning.orientation[2];

    


    double speed;	
    speed=sqrt(pow(ref_object.positioning.velocity[0], 2) + pow(ref_object.positioning.velocity[1], 2));

    if (speed == 0.0){
        return SimMove_OK;
    }

    status = calculateGhostPoint(enuPoint, orientationPoint, ref_object.positioning.algorithm.segment, enuOnLine, speed, time, ref_object.positioning.algorithm.lenght); // Put references sign
    
    if(status!=SimMove_OK){
        return status;
    }
    
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
    // write it back
    ref_segment = ref_object.positioning.algorithm.segment;

    ref_object.positioning.algorithm.enu[0] = enuPoint.x;
    ref_object.positioning.algorithm.enu[1] = enuPoint.y;
    ref_object.positioning.algorithm.enu[2] = enuPoint.z;

    ref_object.positioning.orientation[0] = orientationPoint.x; 
    ref_object.positioning.orientation[2] = orientationPoint.y;
    ref_object.positioning.orientation[3] = orientationPoint.z;

    ref_object.positioning.position[0] = llaPoint.x * 10000000;
    ref_object.positioning.position[1] = llaPoint.y * 10000000;
    ref_object.positioning.position[2] = llaPoint.z * 100;

    ref_object.positioning.algorithm.enuPointOnLine[0] = enuOnLine.x;
    ref_object.positioning.algorithm.enuPointOnLine[1] = enuOnLine.y;
    ref_object.positioning.algorithm.enuPointOnLine[2] = enuOnLine.z;

    return SimMove_OK;
}


SimMove_Status SimMove::calculateGhostPoint(Point& ref_check, Point& ref_orintation, int64_t& ref_segment, Point& ref_pointOnLine, double speed, double time, double& lenghtFromCenterLine) {
    Point check = ref_check;
    Point proectionPoint = ref_pointOnLine ;
    int segment_count = ref_segment;
    SimMove_Status status;

    // due to accuracy the first iteration will be changed. so we need to return if speed is zero explicitly;
    if(speed ==0)
        return SimMove_OK;

    //Find a point proection on the center line. Find a segment where the point is    
    //cout << "The point: "<< check.x <<" "<< check.y <<" " << check.z << endl;
    //cout<< "First vector: "<< m_map[segment_count].x <<";" <<m_map[segment_count].y<<"  " << m_map[segment_count+1].x <<";" <<map[segment_count+1].y<<endl;
    //cout << "First proection point: "<< proectionPoint.x <<" "<< proectionPoint.y <<" " << proectionPoint.z << endl;
    //cout << "First segment: " << segment_count << endl;
      
    //Move the point on needed distance (according time and speed) along the center line
    //auto detecting the distance and use the same distance for this iteration
    if (lenghtFromCenterLine == DOUBLE_NULL){ // if that is first iteration
         status = findProection(check, proectionPoint, segment_count);

         if (status!= SimMove_OK)
             return status;

         MathVector ab {m_map[segment_count],m_map[segment_count+1]};
         MathVector ac {m_map[segment_count],check};
         lenghtFromCenterLine = MathVector{check, proectionPoint}.lenght;
         //find out the sign
         if(ab.GetAngleBetweenVectors(ac) < 180.0){
             lenghtFromCenterLine = -lenghtFromCenterLine;
         }      
    }
    

    double lenghtToStartSegment = MathVector{ proectionPoint, m_map[segment_count]}.lenght;
    double nowDistance = m_distanceMap[segment_count] + lenghtToStartSegment;
    double newDistance = nowDistance + speed * time;
    double totalDistance = *(m_distanceMap.end() - 1);

    //check map. Go to a new lap if the map is over
    while (newDistance > totalDistance) {
        newDistance -= totalDistance;
    }
    // cout << "Now distance: " << nowDistance << "   New distance: " << newDistance << endl;

     //find a segment
     //TODO: easy to speed up
    size_t seg = 0;
    for (; seg < m_distanceMap.size(); seg++) {
        if (m_distanceMap[seg] <= newDistance && m_distanceMap[seg + 1] >= newDistance)
            break;
    }  

    double achivedDistance = m_distanceMap[seg];
    double restDistance = newDistance - achivedDistance;
    //cout << "Distance to the final point on the line: " << restDistance << endl;

    // Get a new point on the line;
     MathVector vec = MathVector(m_map[seg], m_map[seg + 1]);
    vec = vec.normilizeVec();
    Point pointOnLine = vec * restDistance + m_map[seg];

    //cout << "Second segment: " << seg <<endl;  
    //cout << "Second vector: " << m_map[seg].x <<";" <<m_map[seg].y<<"  " << m_map[seg+1].x <<";" <<m_map[seg+1].y<<endl;
    //cout << "Proection on line: " << res.x << " " << res.y << " " << endl;

    //Make a perpendicular of the point to get a point perpendicular to the center line (left or right)
    //TODO delete vec2
    MathVector foundSegment { m_map[seg] ,m_map[seg + 1]};
    MathVector vec2 = MathVector(m_map[seg], m_map[seg + 1]);
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
    Point ghostPoint = vec2 * abs(lenghtFromCenterLine) + pointOnLine; // here the point which we were finding
    //cout << "FouneSegmentVector: " << foundSegment.x << " " << foundSegment.y << endl;
    //cout << "Ghost point: (" << ghostPoint.x << " ; " << ghostPoint.y << " ; "<< ghostPoint.z<<")" << " Segment:" << seg << endl;
    ref_check = ghostPoint;
    ref_segment = seg;
    ref_pointOnLine = pointOnLine;
    //ref_orintation.x = foundSegment.GetAngleToOrtoi()-180;              // TODO make the last segment workable 

    MathVector orti{Point{0,0},Point{0,1}};

    double dot = foundSegment.x*orti.x + foundSegment.y*orti.y ;    
    double det = foundSegment.x*orti.y - foundSegment.y*orti.x ;     
    double angle = atan2(det, dot) * 180/M_PI;

    ref_orintation.x =  angle;//foundSegment.GetAngleBetweenVectors(MathVector{Point{0,0},Point{1,0}}) -180; // 
    //cout << "Angle: "<< angle << endl;
    //a_orintation.y = 
    //a_orintation.z = 
    if (angle >=180 || angle < -180)
        ref_orintation.x = 180 - angle;

    m_ghostCar.push_back(ghostPoint); // for check
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
    for (vector<Point>::iterator it = m_ghostCar.begin(); it != m_ghostCar.end(); ++it)
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
    if (!file){
        cout << "Can not load " <<fileName<< endl;
        return SimMove_ERROR;
    }

    vector<double> values;
    copy(istream_iterator<double>(file), istream_iterator<double>(), back_inserter(values));
    file.close();
    m_map.clear();
    for (size_t i = 0; i < values.size(); i = i + 2){        
        Point a;
        a.x = values[i];
        a.y = values[i + 1];
        m_map.push_back(a);
    }
    //cout <<"The last element" << *(m_distanceMap.end() - 1);
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

    for (vector<Point>::iterator it = m_map.begin() ; it!=m_map.end() ; ++it){
        outf<< it->x<<";"<< it->y<<";"<<endl;
    }
    
    return SimMove_OK;
}

SimMove_Status SimMove::devideMapIntoSegments(){
    m_distanceMap.clear();
    m_distanceMap.push_back(0.0);
    double tempDistance = 0;

    for (size_t i = 1; i < m_map.size(); i++){
        tempDistance += MathVector{ m_map[i - 1], m_map[i] }.lenght;
        m_distanceMap.push_back(tempDistance);
    }
    //cout <<"The last element" << *(m_distanceMap.end() - 1);
    return SimMove_OK;

}
SimMove_Status SimMove::enuParserLoad(const char* fileName){
    Document document;   
    ifstream ifs(fileName);
    IStreamWrapper isw(ifs); 

    if (document.ParseStream(isw).HasParseError() || !document.IsObject()) {
        cout<<"JSON parse error!";}// TODO}
    //cout<< "Is array:" << document["result"]["extras"]["rrMapSource"].IsArray() << endl;
    //cout<< document.HasMember("result");

    const char *mapString = document["result"]["extras"]["rrMapSource"].GetString();

    Document mapParser;
    mapParser.Parse(mapString);

    const rapidjson::Value& b = mapParser["Centre"];
    Point mapPoint;
    m_map.clear();
    for (rapidjson::SizeType i = 0; i < b.Size(); i++){
         const rapidjson::Value& a = b[i];
      //cout<<"!!!!!!!!!!!!!!!!! "<< a.Size()<<endl;
         mapPoint.x= a[rapidjson::SizeType(0)].GetDouble();
         mapPoint.y= a[rapidjson::SizeType(1)].GetDouble();
         m_map.push_back(mapPoint);
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

    if(w.Size()==0 || !w.IsArray()){
        cout<< "Nothing to parse in "<< fileName << endl;
        return SimMove_ERROR;
    }

    for(size_t i=0;i<w.Size();i++){
        Point temp;
        temp.x = w[i][1].GetDouble();
        temp.y = w[i][0].GetDouble();
        temp.z = w[i][2].GetDouble();
        m_llaCoordinates.push_back(temp);
        //cout<<"Number: " << w[i][j].IsNumber() << endl;
    }

//TODO: be carefull with points. x=y, 
    refPoint.x = m_llaCoordinates[0].x;
    refPoint.y = m_llaCoordinates[0].y;
    refPoint.z = m_llaCoordinates[0].z;
    geodeticConverter.initialiseReference(refPoint.x,refPoint.y ,refPoint.z );
//Convert to enu and write the map

    for (size_t i =0 ; i<m_llaCoordinates.size();i++){
    
		if (geodeticConverter.isInitialised()) {
                Point enuPoint;
				geodeticConverter.geodetic2Enu(m_llaCoordinates[i].x, m_llaCoordinates[i].y,m_llaCoordinates[i].z, &enuPoint.x,&enuPoint.y,&enuPoint.z);
               // outf<<"["<< m_llaCoordinates[i].y << "," << m_llaCoordinates[i].x << "," << m_llaCoordinates[i].z <<"],"<< endl;
                m_map.push_back(enuPoint);
        }
            
    }
    //outf<<"]";
//Write the enu array

//Convert back enu-lla, do not need here, just for test
    m_llaCoordinates.clear();
    Point llaPoint;
    for (size_t i =0; i<m_map.size();i++){
		if (geodeticConverter.isInitialised()) {
				geodeticConverter.enu2Geodetic(m_map[i].x, m_map[i].y, m_map[i].z ,&llaPoint.y,&llaPoint.x,&llaPoint.z);
                m_llaCoordinates.push_back(llaPoint);
        }
    }

    cout<< "Map Size: "<< m_map.size()<< endl ;
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
    m_map.push_back(a);
    m_map.push_back(b);
    m_map.push_back(c);

    // split the track lenght into segments
    m_distanceMap.push_back(0.0);
    double tempDistance = 0;
    for (size_t i = 1; i < m_map.size(); i++)
    {
        tempDistance += MathVector{ m_map[i - 1],m_map[i] }.lenght;
        m_distanceMap.push_back(tempDistance);
    }
    cout << "Simulation lib initialization is completed"<< endl;
    return SimMove_OK;
}

