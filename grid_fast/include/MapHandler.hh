#ifndef MAPHANDLER_H
#define MAPHANDLER_H

#include "datatypes.hh"
using namespace std;

class MapHandler {
public:
  MapHandler();
  ~MapHandler();
  void updateMap(vector<int>, int, int, float, double newMapOffsetX = 0,
                 double newMapOffsetY = 0, double newMapOffsetZ = 0,
                 double newMapHight = 0);
  void updateMap(MapHandler *);
  int getMapSizeX();
  int getMapSizeY();
  double getMapOffsetX();
  double getMapOffsetY();
  double getMapOffsetZ();
  double getMapHight();
  float getMapResolution();
  int getMap(int, int);
  void setMap(int, int, int);
  int getMapUnsafe(int,
                   int); // Do not have any protection for out of array access
  vector<int> getMapComplete();
  ant_data ant_step(ant_data, bool);
  int checkForWallRay(point_int p1, point_int p2);

private:
  vector<int> map;
  int mapSizeX, mapSizeY;
  double mapOffsetX, mapOffsetY, mapOffsetZ, mapHight;
  float resulution;
  point_int rotate_dir(point_int dir, bool cw);
};

#endif
