#pragma once

#include "datatypes.hh"
#include "MapHandler.hh"
#include "MapTransform.hh"
using namespace std;

class GapHandler
{
public:
    GapHandler();
    ~GapHandler();
    void analysis(MapHandler*,MapTransform*);
    int getSizeRows(int);
    int getSizeGaps(int,int);
    scanGroup* get(int,int,int);
    void add(scanGroup,int,int,bool);
    bool updateGap(scanGroup*);
    void cleanConnections(scanGroup*);
    void remove(scanGroup*);
    void clear();

private:
    int size;
    vector<gapList> gaps;
    void analysisAtAngle(int,MapHandler*,MapTransform*);
    void checkForOverlap(scanGroup*,scanGroup*);
};

