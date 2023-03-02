#pragma once

#include "datatypes.hh"
#include "MapHandler.hh"
#include "MapTransform.hh"
#include "GapHandler.hh"
using namespace std;

class OpeningHandler
{
public:
    vector<opening> openingDebug;
    OpeningHandler();
    ~OpeningHandler();
    void updateDetections(MapHandler*,MapTransform*,GapHandler*);
    void update(MapHandler*);
    int size();
    int gapDetectionsSize();
    openingDetection* get(int);
    opening* getDetection(int);
    openingDetection* add(opening);
    void remove(openingDetection*);
    void disable(openingDetection*,int);
    void clear();
    bool checkForWall(opening o, MapHandler*);
    wallCell getNextOpening(openingDetection*,bool,int,bool,bool checkFirst=false,bool stopAtEmpty=false,vector<wallCell*>* pointList=NULL);
    vector<point_int> getPointsBetweenOpenings(openingDetection*,bool,openingDetection*,bool, vector<wallCell*>* wallPoints=NULL); 
private:
    vector<opening> detectionList;
    vector<vector<vector<int>>> detectionMap;
    vector<openingDetection*> openingList;
    vector<wall*> wallList;
    bool intersectOpenings(openingDetection*,openingDetection*);
    bool intersectOpenings(opening*,opening*);
    void checkForDetection(int,int,int,MapHandler*,MapTransform*,GapHandler*);
    bool checkDepth(scanGroup*,bool,int);
    opening rotateOpening(opening,int,MapTransform*);
    void correctOpening(opening*,MapHandler*);
    void getWalls(MapHandler*);
    void getAndFilterWall(MapHandler*, point_int, vector<point_int>*);
    bool findOpenings(MapHandler*,int, vector<opening>*, vector<int>*);
    point_int findIntersectionPoint(opening, opening);
    bool check_unnecessary_openings(opening o,MapHandler*);
    //void fitNonFixedOpenings(MapHandler*);
    void fixOverlapingPoints(MapHandler*);
    bool ccw(point,point,point);
    int checkForOpenings(wallCell*);
    void fixOverlap(openingDetection*, openingDetection*,MapHandler*);
    void swapEnds(opening*,bool,opening*,bool);
    vector<point_int> generateOpeningPoints(openingDetection *o);
};