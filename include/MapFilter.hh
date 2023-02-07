#pragma once

#include "datatypes.hh"
#include "MapHandler.hh"
#include "MapTransform.hh"
#include "GapHandler.hh"

class MapFilter
{
public:
    MapFilter();
    ~MapFilter();
    void filterMap(MapHandler*,MapTransform*,GapHandler*);
private:
    int previusSizeX,previusSizeY;
    vector<bool> wallFollowLookUp;
    bool getLookUp(int,int);
    void setLookUp(int,int,bool);
    void removeOpenings(MapHandler*,MapTransform*,GapHandler*);
    void filterGapOverlapSize(GapHandler*,scanGroup*);
    void fillGapAtMap(MapHandler*,MapTransform*,int,scanGroup*);
    void removeObjects(MapHandler*,MapTransform*,GapHandler*);
    void checkAndRemoveObject(MapHandler*,scanGroup*,int);
    void fitGapToMap(MapHandler*,MapTransform*,GapHandler*,int,int,int);
};
