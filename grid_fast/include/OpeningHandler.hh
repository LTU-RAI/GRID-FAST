#pragma once

#include "GapHandler.hh"
#include "MapHandler.hh"
#include "MapTransform.hh"
#include "datatypes.hh"
using namespace std;

class OpeningHandler {
public:
  vector<opening> openingDebug;
  OpeningHandler(int numberOfDir, int minGroupSize, int minFrontier,
                 int objectFilterMaxStep, bool show_removed_openings);
  ~OpeningHandler();
  void updateDetections(MapHandler *, MapTransform *, GapHandler *);
  void update(MapHandler *);
  int size();
  int gapDetectionsSize();
  openingDetection *get(int);
  opening *getDetection(int);
  openingDetection *add(opening);
  void remove(openingDetection *);
  void disable(openingDetection *, int);
  void clear();
  bool checkForWall(opening *o, MapHandler *);
  wallCell getNextOpening(openingDetection *, bool, int, bool,
                          bool checkFirst = false, bool stopAtEmpty = false,
                          vector<wallCell *> *pointList = NULL);
  vector<point_int>
  getPointsBetweenOpenings(openingDetection *, bool, openingDetection *, bool,
                           vector<wallCell *> *wallPoints = NULL);
  void addCustomOpening(opening op, int id);
  void removeCustomOpening(int id);
  int maxCustomOpeningId();

private:
  int numberOfDir;
  int minGroupSize;
  int minFrontier;
  int objectFilterMaxStep;
  bool show_removed_openings;

  vector<opening> detectionList;
  vector<opening> customOpeningList;
  vector<int> customOpeningIdList;
  vector<vector<vector<int>>> detectionMap;
  vector<openingDetection *> openingList;
  vector<wall *> wallList;
  bool intersectOpenings(openingDetection *, openingDetection *);
  bool intersectOpenings(opening *, opening *);
  void checkForDetection(int, int, int, MapHandler *, MapTransform *,
                         GapHandler *);
  opening rotateOpening(opening, int, MapTransform *);
  void correctOpening(opening *, MapHandler *);
  void getWalls(MapHandler *);
  void getAndFilterWall(MapHandler *, point_int, vector<point_int> *);
  void findOpenings(MapHandler *);
  point_int findIntersectionPoint(opening, opening);
  bool check_unnecessary_openings(openingDetection *o, MapHandler *map);
  // void fitNonFixedOpenings(MapHandler*);
  void fixOverlapingPoints(MapHandler *);
  bool ccw(point, point, point);
  int checkForOpenings(wallCell *);
  int fixOverlap(openingDetection *, openingDetection *, MapHandler *);
  void swapEnds(opening *, bool, opening *, bool);
  vector<point_int> generateOpeningPoints(openingDetection *o);
  vector<point_int> generateOpeningPoints(opening *o);
  int checkForPotensialOpening(MapHandler *map, int targetIndex,
                               vector<opening> *dList, vector<int> *ignorList,
                               opening *op = NULL, bool returnFirst = false);
};
