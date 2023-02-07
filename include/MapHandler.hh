#pragma once

#include "datatypes.hh"
using namespace std;

class MapHandler
{
public:
    MapHandler();
    ~MapHandler();
    void updateMap(vector<int>,int,int,int,int,int);
    void updateMap(MapHandler*);
    int getMapSizeX();
    int getMapSizeY();
    int getMapOffsetX();
    int getMapOffsetY();
    int getMapResolution();
    int getMap(int,int);
    void setMap(int,int,int);
    int getMapUnsafe(int,int); //Do not have any protection for out of array access
    vector<int> getMapComplete();
    ant_data ant_step(ant_data,bool);
private:
    vector<int> map;
    int mapSizeX, mapSizeY, mapOffsetX, mapOffsetY, resulution;
    point_int rotate_dir(point_int dir, bool cw);
    void initializeMap(int,int);

};



