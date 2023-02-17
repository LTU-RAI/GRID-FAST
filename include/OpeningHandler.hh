#pragma once

#include "datatypes.hh"
#include "MapHandler.hh"
#include "MapTransform.hh"
#include "GapHandler.hh"
using namespace std;

class OpeningHandler
{
public:
    OpeningHandler();
    ~OpeningHandler();
    void updateDetections(MapHandler*,MapTransform*,GapHandler*);
    void update(MapHandler*);
    int size();
    int gapDetectionsSize();
    opening* get(int);
    opening* getDetection(int);
    void add(opening);
    void remove(opening*);
    void updateOpening(opening*, opening);
    void clear();
    bool intersectOpenings(opening *o1,opening *o2);

private:
    vector<opening> detectionList;
    vector<vector<vector<int>>> detectionMap;
    vector<opening*> openingList;
    vector<wall*> wallList;
    void checkForDetection(int,int,int,MapHandler*,MapTransform*,GapHandler*);
    bool checkDepth(scanGroup*,bool,int);
    opening rotateOpening(opening,int,MapTransform*);
    void correctOpening(opening*,MapHandler*);
    void getWalls(MapHandler*);
    void getAndFilterWall(MapHandler*, point_int, vector<point_int>*);
    bool findOpenings(MapHandler*,int, vector<opening>*, vector<int>*);
    point_int findIntersectionPoint(opening, opening);
    bool checkForWall(opening o, MapHandler*);
    bool check_unnecessary_openings(opening o,MapHandler*);
    void fitNonFixedOpenings(MapHandler*);
    void fixOverlapingPoints(MapHandler*);
    void connectToWall(opening*);
    void disconnectFromWall(opening*);
    bool ccw(point,point,point);
    int checkForOpenings(wallCell*);
    void fixOverlap(opening*, opening*,MapHandler*);
};