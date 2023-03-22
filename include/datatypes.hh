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

class openingDetection;
struct wall;
struct wallCell{
    point_int position={-1,-1};
    bool emptyNeighbour=false;
    wall* parent=NULL;
    int index=-1;
    vector<openingDetection*> connectedtOpeningEnd;
    vector<openingDetection*> connectedtOpeningStart;
};

struct wall
{
    vector<wallCell*> wall;
    int index=-1;

    wallCell* getCell(int* index){
        getIndex(index);
        return wall[*index];
    }
    uint size(){
        return wall.size();
    }
    wallCell* push_back(wallCell w){
        wallCell* newW=new wallCell(w);
        wall.push_back(newW);
        return newW;
    }

    void clear(){
        for(int i=0;i<wall.size();i++){
            delete wall[i];
        }
        wall.clear();
    }

    void getIndex(int* index){
        if(*index<0) *index=int(wall.size())-(-*index)%int(wall.size());
        if(*index>=wall.size()) *index=*index%int(wall.size());
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
    vector<point_int> occupiedPoints;
    int angle;
    bool start_is_outside;
    int label=1;
    polygon* parent_polygon=NULL;
    bool fliped=false;
    bool moved=false;
    wallCell* connectedWallStart=NULL;
    wallCell* connectedWallEnd=NULL;


    void flip(){
        point_int t=start;
        start=end;
        end=t;
        wallCell* wh=connectedWallStart;
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

class openingDetection{
private:
    wallCell* startPoint=NULL;
    wallCell* endPoint=NULL;
public:
    int label=1;
    polygon* parent=NULL;
    polygon* parentCoridor=NULL;
    vector<point_int> occupiedPoints;
    openingDetection(){}
    ~openingDetection(){}
    point_int start(){
        return startPoint->position;
    }
    point_int end(){
        return endPoint->position;
    }
    wallCell* getConnection(bool getStart){
        if(getStart){
            return startPoint;
        }else{
            return endPoint;
        }
    }
    void disconnect(bool changeStart){
        //disconnect
        wallCell* point=changeStart?startPoint:endPoint;
        if(point==NULL) return;
        vector<openingDetection*>* list=changeStart?&point->connectedtOpeningStart:&point->connectedtOpeningEnd;
        for(int i=0;i<list->size();i++){
            if(list->at(i)!=this) continue;
            list->erase(list->begin()+i);
        }  
    }
    void connect(bool changeStart,wallCell* newW){
        disconnect(changeStart);
        //connect
        if(changeStart){
            startPoint=newW;
        }else{
            endPoint=newW;
        }
        vector<openingDetection*>* list=changeStart?&newW->connectedtOpeningStart:&newW->connectedtOpeningEnd;
        list->push_back(this);
    }
    opening getOpening(){
        opening op;
        op.start=start();
        op.end=end();
        op.connectedWallStart=startPoint;
        op.connectedWallEnd=endPoint;
        op.label=label;
        return op;
    }
    point_int getCenter(){
        point_int center={0,0};
        center.x=(end().x-start().x)/2+start().x;
        center.y=(end().y-start().y)/2+start().y;
        return center;
    }
};

typedef vector<point_int> robotPath;
struct polygon{
    point_int center={0,0};
    vector<openingDetection*> openings;
    vector<point_int> polygon_points;
    vector<point_int> polygon_points_desplay;
    vector<polygon*> connectedpolygons;
    vector<vector<point_int>> connectedPaths;
    vector<robotPath> pathList;
    vector<point_int> fillPoints;
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
    vector<point_int> fronterPositions;
    bool emty_cell=false;
};

struct mapTransformCell{
    point_int tpos, rpos;
};

typedef vector<vector<mapTransformCell>> mapTransformMap;
