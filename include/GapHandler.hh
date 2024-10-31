#pragma once

#include "datatypes.hh"
#include "MapHandler.hh"
#include "MapTransform.hh"
using namespace std;

class GapHandler
{
public:
    GapHandler(int numberOfDir, int cfilterSize, int minGroupSize);
    ~GapHandler();
    void analysis(MapHandler*,MapTransform*);
    int getSizeRows(int);
    int getSizeGaps(int,int);
    scanGroup* get(int,int,int);
    void add(scanGroup,int,int);
    bool updateGap(scanGroup*);
    void cleanConnections(scanGroup*);
    void remove(scanGroup*);
    void clear();

private:
    int cfilterSize;
    int numberOfDir; 
    int minGroupSize;
    int size;
    
    vector<gapList> gaps;
    vector<point_int> toBeFilterdPoints;
    vector<int> toBeFilterdValues;
    void analysisAtAngle(int,MapHandler*,MapTransform*);
    void analysisAtRow(int,int,MapHandler*,MapTransform*);
    void fillGapAtMap(MapHandler*,MapTransform*,int,scanGroup*);
    bool checkForOverlap(scanGroup*,scanGroup*);
};

