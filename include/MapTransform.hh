#pragma once

#include "datatypes.hh"
#include "MapHandler.hh"

class MapTransform
{
public:
    MapTransform();
    ~MapTransform();
    void updateTransform(MapHandler*);
    int getMaptransformSizeY(int);
    int getMaptransformSizeX(int,int);
    mapTransformCell getMapTransformCell(int,int,int);
    int getMapAtTransform(int,int,int,MapHandler*);
    void setMapAtTransform(int,int,int,int,MapHandler*);
    point_int getMapIndexTransform(int,int,int);
private:
    int previusMapSizeX,previusMapSizeY;
    vector<mapTransformMap> mapTransformList;
    void generateMapTransform(int,int);

};
