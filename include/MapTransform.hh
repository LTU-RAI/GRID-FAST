#pragma once

#include "datatypes.hh"
#include "MapHandler.hh"

class MapTransform
{
public:
    MapTransform(int numberOfDir);
    ~MapTransform();
    void updateTransform(MapHandler*, bool forceUpdate=false);
    int getMaptransformSizeY(int);
    int getMaptransformSizeX(int,int);
    mapTransformCell getMapTransformCell(int,int,int);
    int getMapAtTransform(int,int,int,MapHandler*);
    void setMapAtTransform(int,int,int,int,MapHandler*);
    point_int getMapIndexTransform(int,int,int);
private:
    int numberOfDir;
    int previusMapSizeX,previusMapSizeY;
    vector<mapTransformMap> mapTransformList;
    void generateMapTransform(int,int);

};
