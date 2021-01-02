// RaceTest.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//
#include "SimMove.h"

vector <MV_Spatial> ghostCarMV;
SimMove SimMove;

using namespace std;
geodetic_converter::GeodeticConverter geodeticConverter2;
//Point refPoint2{52.2370937,-0.4684315,88.9243};
//Point refPoint2{55.5647425,37.9911456,0.0}; //LV
void checkPrecision(){
Point refPoint;
refPoint.x = SimMove.m_llaCoordinates[0].x;
refPoint.y = SimMove.m_llaCoordinates[0].y;
refPoint.z = SimMove.m_llaCoordinates[0].z;
geodeticConverter2.initialiseReference(refPoint.x,refPoint.y ,refPoint.z );
SimMove.m_map[0];
Point llaPoint;
Point enuPoint;
int a,b,c;

llaPoint.x = SimMove.m_llaCoordinates[0].x;
llaPoint.y = SimMove.m_llaCoordinates[0].y;
llaPoint.z = SimMove.m_llaCoordinates[0].z;
a = llaPoint.x * 10000000;
b = llaPoint.y * 10000000;
c = llaPoint.z * 100;
// trannsformation without int 
cout << "lla point x:" << llaPoint.x << " ; "<< llaPoint.y  << " ; "<< llaPoint.z << endl;
    geodeticConverter2.geodetic2Enu(llaPoint.x,
                                    llaPoint.y,
                                    llaPoint.z,
                                    &enuPoint.x, &enuPoint.y, &enuPoint.z);

    cout << "enu point x:" << enuPoint.x << " ; "<< enuPoint.y  << " ; "<< enuPoint.z << endl;

    geodeticConverter2.enu2Geodetic(enuPoint.x, 
                                    enuPoint.y, 
                                    enuPoint.z ,
                                   &llaPoint.x, &llaPoint.y, &llaPoint.z);
cout << "back lla point x:" << llaPoint.x << " ; "<< llaPoint.y  << " ; "<< llaPoint.z << endl << endl<< endl;

    geodeticConverter2.geodetic2Enu(llaPoint.x,
                                    llaPoint.y,
                                    llaPoint.z,
                                    &enuPoint.x, &enuPoint.y, &enuPoint.z);

    cout << "enu point x:" << enuPoint.x << " ; "<< enuPoint.y  << " ; "<< enuPoint.z << endl;

// transformation after int

    geodeticConverter2.geodetic2Enu((double)(a/10000000.0),
                                    (double)(b/10000000.0),
                                    (double)(c/100.0),
                                    &enuPoint.x, &enuPoint.y, &enuPoint.z);

    cout << "enu point rought x:" << enuPoint.x << " ; "<< enuPoint.y  << " ; "<< enuPoint.z << endl;


    geodeticConverter2.enu2Geodetic(enuPoint.x, 
                                    enuPoint.y, 
                                    enuPoint.z ,
                                   &llaPoint.y, &llaPoint.x, &llaPoint.z);
    cout << "back lla point x:" << llaPoint.x << " ; "<< llaPoint.y  << " ; "<< llaPoint.z << endl;

    a = llaPoint.x * 10000000;
    b = llaPoint.y * 10000000;
    c = llaPoint.z * 100;

    geodeticConverter2.geodetic2Enu((double)(a/10000000.0),
                                    (double)(b/10000000.0),
                                    (double)(c/100.0),
                                    &enuPoint.x, &enuPoint.y, &enuPoint.z);

    cout << "enu point 2 x:" << enuPoint.x << " ; "<< enuPoint.y  << " ; "<< enuPoint.z << endl;


}


int main()
{   
    
    MV_ObjectPositioning Object;
    //SimMove.init("llaLVone.json");
    SimMove.init("llaLVone.json");
    SimMove.saveRawIntoFile("RawMapLV.txt");
    checkPrecision();
    //SimMove.enuParserLoad("enu.json");
     //if (SimMove.loadMapFromFileENU("testenu.txt") != SimMove_OK) {
     //  cout << "Can not load the file" << endl;
      //  return -1;
    //}
    

    Point check;
    check.x = SimMove.m_map[0].x;
    check.y = SimMove.m_map[0].y;
    //check.x = -0.4670052483890642;
    //check.y = 52.237186297262397;
    //check.z = 88.925141052780788;

    //MV_Spatial checkMV; //Remember position[x] - is int!
    //Object.positioning.position[0] = -0.4670052483890642 * 1000 * 1000 * 10 ;
    //Object.positioning.position[1] = 52.237186297262397 *  1000 * 1000 * 10 ;
    //Object.positioning.position[2] = 88.925141052780788 * 100 ;

    Object.positioning.position[0] = SimMove.m_llaCoordinates[0].x * 1000 * 1000 * 10 ;
    Object.positioning.position[1] = SimMove.m_llaCoordinates[0].y *  1000 * 1000 * 10 ;
    Object.positioning.position[2] = SimMove.m_llaCoordinates[0].z * 100 ;
    Object.positioning.velocity[0]  =7.0;
    Object.positioning.velocity[1]  =8.0;
//Point refPoint{52.2370937,-0.4684315,88.9243}; //TODO find out where the reference point is and if it's right
    //geodeticConverter2.initialiseReference(refPoint2.y,refPoint2.x ,refPoint2.z); 
	//geodeticConverter2.geodetic2Enu((double)(checkMV.position[0]/10000000.0),
    //                                (double)(checkMV.position[1]/10000000.0),
    //                                (double)(checkMV.position[2]/100.0),
     //                               &check.y,&check.x,&check.z);

 
    for (int i = 0; i < 20000; i++)
    {
      Point orientation;
      int64_t segment;

    //SimMove.calculateGhostPoint(check, orientation ,segment,1 /*speed*/, 0.01 /*time*/, 5 /*distance from center line*/);  //DOUBLE_NULL
     //SimMove.getGhostPoint(Object, segment, 0.1/*time*/);//distance from center line*/);

     }
    SimMove.saveIntoFileGhostCar("ghostCar.txt");
    //saveIntoFile("ghostCar.txt");
    //saveIntoFileMV("ghostCarMV.txt");
    cout << "done" << endl;
    std::cin.get();
} 