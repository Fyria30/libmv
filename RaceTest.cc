// RaceTest.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//
#include "SimMove.h"

vector <MV_Spatial> ghostCarMV;

using namespace std;
geodetic_converter::GeodeticConverter geodeticConverter2;
//Point refPoint2{52.2370937,-0.4684315,88.9243};
//Point refPoint2{55.5647425,37.9911456,0.0}; //LV

int main()
{   
    SimMove SimMove;
    MV_ObjectPositioning Object;
    //SimMove.init("llaLVone.json");
    SimMove.init("llaLVone.json");
    SimMove.saveRawIntoFile("RawMapLV.txt");
  
    //SimMove.enuParserLoad("enu.json");
     //if (SimMove.loadMapFromFileENU("testenu.txt") != SimMove_OK) {
     //  cout << "Can not load the file" << endl;
      //  return -1;
    //}
    

    Point check;
    check.x = SimMove.map[0].x;
    check.y = SimMove.map[0].y;
    //check.x = -0.4670052483890642;
    //check.y = 52.237186297262397;
    //check.z = 88.925141052780788;

    //MV_Spatial checkMV; //Remember position[x] - is int!
    //Object.positioning.position[0] = -0.4670052483890642 * 1000 * 1000 * 10 ;
    //Object.positioning.position[1] = 52.237186297262397 *  1000 * 1000 * 10 ;
    //Object.positioning.position[2] = 88.925141052780788 * 100 ;

    Object.positioning.position[0] = SimMove.llaCoordinates[0].x * 1000 * 1000 * 10 ;
    Object.positioning.position[1] = SimMove.llaCoordinates[0].y *  1000 * 1000 * 10 ;
    Object.positioning.position[2] = SimMove.llaCoordinates[0].z * 100 ;
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

      //SimMove.calculateGhostPoint(check, orientation ,1 /*speed*/, 0.01 /*time*/, 5 /*distance from center line*/, segment);  //DOUBLE_NULL
     SimMove.getGhostPoint(Object, 0.1/*time*/,segment);//distance from center line*/);

     }
    SimMove.saveIntoFileGhostCar("ghostCar.txt");
    //saveIntoFile("ghostCar.txt");
    //saveIntoFileMV("ghostCarMV.txt");
    cout << "done" << endl;
    std::cin.get();
} 