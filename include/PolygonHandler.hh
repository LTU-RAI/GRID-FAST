#pragma once

#include "datatypes.hh"
#include "MapHandler.hh"
#include "OpeningHandler.hh"
using namespace std;

class PolygonHandler
{
public:
    PolygonHandler();
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
    void remove(polygon*,OpeningHandler*);
    void clear();

private:
    vector<polygon*> polygonList; 
    void creatIntersection(OpeningHandler*,openingDetection*);
    void creatPathway(OpeningHandler*,openingDetection*);
    void getArea(int,OpeningHandler*);
    point_int getPolygonCenter(vector<point_int> sList);
    void getPathForPolygon(polygon*,OpeningHandler*, MapHandler*, MapHandler*);
    vector<point_int> generateVoronoi(polygon*,MapHandler* , MapHandler*, point_int start, point_int end={-1,-1});
    bool optimizeIntersection(polygon*,OpeningHandler* openingList, MapHandler* map);
    double DFunction(double);
    void removeSideFromPolygon(polygon*,openingDetection*, OpeningHandler*);
};