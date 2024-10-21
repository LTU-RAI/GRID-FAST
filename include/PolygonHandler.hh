#pragma once

#include "datatypes.hh"
#include "MapHandler.hh"
#include "OpeningHandler.hh"
using namespace std;

class PolygonHandler
{
public:
    PolygonHandler(int minGroupSize, int polygonRez, int voronoiRez, int minimumDesendingSteps, double dw, double minDistToCenter, double maxPenalty, double polygonMergingDist, bool optimizIntersections, bool polygonMerging);
    ~PolygonHandler();
    void updateIntersections(OpeningHandler*,MapHandler*);
    void optimize(OpeningHandler*,MapHandler*);
    void getPathways(OpeningHandler*);
    void generatePolygonArea(OpeningHandler*);
    void generateRobotPath(OpeningHandler*,MapHandler*, MapHandler*);
    bool checkIfObstructed(point_int, point_int, MapHandler*);
    int size();
    polygon* get(int);
    polygon* add(polygon);
    void remove(polygon*,OpeningHandler*,bool removeOpenings=true);
    void clear();
    void updateIndex();

private:
    int minGroupSize;
    int polygonRez;
    int voronoiRez;
    int minimumDesendingSteps;

    double dw;
    double minDistToCenter;
    double maxPenalty;
    double polygonMergingDist;
    
    bool optimizIntersections;
    bool polygonMerging;

    vector<polygon*> polygonList; 
    polygon* creatIntersection(OpeningHandler*,openingDetection*);
    void creatPathway(OpeningHandler*,openingDetection*);
    void getArea(int,OpeningHandler*);
    point_int getPolygonCenter(vector<point_int> sList);
    void getPathForPolygon(polygon*,OpeningHandler*, MapHandler*, MapHandler*);
    vector<point_int> generateVoronoi(polygon*,MapHandler* , MapHandler*, point_int start, point_int end={-1,-1});
    bool optimizeIntersection(polygon*,OpeningHandler* openingList, MapHandler* map);
    bool merge(polygon*,OpeningHandler* openingList, MapHandler* map);
    double DFunction(double);
    void removeSideFromPolygon(polygon*,openingDetection*, OpeningHandler*);
};