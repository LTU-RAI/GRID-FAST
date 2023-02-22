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

struct opening;
struct wallCell{
    point_int position={-1,-1};
    bool emptyNeighbour=false;
    vector<opening*> connectedtOpeningEnd;
    vector<opening*> connectedtOpeningStart;
};

struct wall
{
    vector<wallCell> wall;

    wallCell* getCell(int* index){
        getIndex(index);
        return &wall[*index];
    }
    uint size(){
        return wall.size();
    }
    void push_back(wallCell w){
        wall.push_back(w);
    }

    void getIndex(int* index){
        if(*index<0) *index=*index%wall.size()+wall.size();
        if(*index>=wall.size()) *index=*index%wall.size();
    }

    int getDistans(int index1, int index2){
        getIndex(&index1);
        getIndex(&index2);
        int d1=std::abs(index1-index2);
        int d2=wall.size()-std::max(index1,index2)+std::min(index1,index2);
        if(d1<d2) return d1;
        return d2;
    }

    bool getCw(int index1, int index2){
        getIndex(&index1);
        getIndex(&index2);
        int d1=std::abs(index1-index2);
        int d2=wall.size()-std::max(index1,index2)+std::min(index1,index2);
        if(d1<d2) return true;
        return false;
    }
};

struct polygon;
struct opening{
    point_int start;
    point_int end;
    int sideToMove=3;//1:start,2:end,3:none
    int angle;
    bool start_is_outside;
    int label=1;
    polygon* parent_polygon=NULL;
    bool fliped=false;
    bool moved=false;
    int conected_to_path=-1;
    int connectedWallIndexStart=-1;
    int connectedWallIndexEnd=-1;
    wall* connectedWallStart=NULL;
    wall* connectedWallEnd=NULL;


    void flip(){
        point_int t=start;
        start=end;
        end=t;
        int wih=connectedWallIndexStart;
        connectedWallIndexStart=connectedWallIndexEnd;
        connectedWallIndexEnd=wih;
        wall* wh=connectedWallStart;
        connectedWallStart=connectedWallEnd;
        connectedWallEnd=wh;
    }

    point_int get_center(){
        point_int center={0,0};
        center.x=(end.x-start.x)/2+start.x;
        center.y=(end.y-start.y)/2+start.y;
        return center;
    }
};

struct polygon{
    point_int center={0,0};
    vector<opening*> openings;
    vector<point_int> polygon_points;
    vector<point_int> polygon_points_desplay;
    vector<polygon*> connectedpolygons;
    vector<vector<point_int>> connectedPaths;
    int label=1;
    bool inactiv=false;
    bool path=false;
    void add_point(point_int p, bool cw){
        if(cw){
            polygon_points.push_back(p);
        }else{
            polygon_points.insert(polygon_points.begin(),p);
        }
    }
    void add_point_d(point_int p, bool cw){
        if(cw){
            polygon_points_desplay.push_back(p);
        }else{
            polygon_points_desplay.insert(polygon_points_desplay.begin(),p);
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
