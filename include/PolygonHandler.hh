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
    void getPathways(OpeningHandler*);
    void generatePolygonArea(OpeningHandler*);
    int size();
    polygon* get(int);
    polygon* add(polygon);
    void remove(polygon*,OpeningHandler*);
    void clear();

private:
    vector<polygon*> polygonList; 
    void creatIntersection(OpeningHandler*,openingDetection*);
    void getArea(int,OpeningHandler*);
    point_int getPolygonCenter(vector<point_int> sList);
    void optimizeIntersection(polygon*,OpeningHandler* openingList, MapHandler* map);
    double DFunction(double);
    void removeSideFromPolygon(polygon*,openingDetection*, OpeningHandler*);
};