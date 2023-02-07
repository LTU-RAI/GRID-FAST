#pragma once
#include <math.h>
#include <vector>
using namespace std;
struct scanGroup{
    int start;
    int end;
    int row;
    int angle;
    bool traversable;
    vector<scanGroup*> prevGroup;
    vector<scanGroup*> nextGroup;
};
typedef vector<vector<scanGroup*>> gapList;

struct point{
    double x;
    double y;
};
struct point_int{
    int x;
    int y;

    point convert_to_point(double resulution, int offset=0){
        point p;
        p.x=(x-offset)*resulution;
        p.y=(y-offset)*resulution;
        return p;
    }
};

struct opening{
    point_int start;
    point_int end;
    vector<point_int> occupied_points;
    int sideToMove=3;//1:start,2:end,3:none
    int angle;
    bool start_is_outside;
    int label=1;
    int parent_poligon=-1;
    bool fliped=false;
    bool moved=false;
    int conected_to_path=-1;

    void flip(){
        point_int t=start;
        start=end;
        end=t;
    }

    point_int get_center(){
        point_int center={0,0};
        center.x=(end.x-start.x)/2+start.x;
        center.y=(end.y-start.y)/2+start.y;
        return center;
    }
};

struct gapGrupe{
    vector<opening> openings;
};

struct poligon{
    point_int center={0,0};
    vector<int> sidesIndex;
    vector<point_int> poligon_points;
    vector<point_int> poligon_points_desplay;
    vector<int> connectedPoligons;
    vector<vector<point_int>> connectedPaths;
    int label=1;
    bool inactiv=false;
    bool path=false;
    void add_point(point_int p, bool cw){
        if(cw){
            poligon_points.push_back(p);
        }else{
            poligon_points.insert(poligon_points.begin(),p);
        }
    }
    void add_point_d(point_int p, bool cw){
        if(cw){
            poligon_points_desplay.push_back(p);
        }else{
            poligon_points_desplay.insert(poligon_points_desplay.begin(),p);
        }
    }
    void add_sideIndex(int index, bool cw){
        if(cw){
            sidesIndex.push_back(index);
        }else{
            sidesIndex.insert(sidesIndex.begin(),index);
        }
    }
};

struct ant_data{
    point_int dir={0,0};
    point_int end;
    bool emty_cell=false;
};

struct mapTransformCell{
    point_int tpos, rpos;
};

typedef vector<vector<mapTransformCell>> mapTransformMap;
