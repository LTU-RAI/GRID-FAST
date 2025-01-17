#ifndef UTILITY_H
#define UTILITY_H

#include "datatypes.hh"
#include <string>

using namespace std;

#define MAP_OCCUPIED 100
#define MAP_UNOCCUPIED 0
#define MAP_UNKNOWN -1
#define AREA_INTERSECTION 0
#define AREA_PATHWAY 10
#define AREA_PATH_TO_FRONTER 30
#define AREA_DEAD_END 20

bool operator==(const point_int &lhs, const point_int &rhs);
bool operator!=(const point_int &lhs, const point_int &rhs);
double dist(point_int p1, point_int p2);
double dist(point p1, point p2);

/*geometry_msgs::PolygonStamped get_polygon(polygon poly, float resolution,*/
/*                                          double mapOffsetX, double
 * mapOffsetY,*/
/*                                          double z = 0.1) {*/
/*  int MapOrigenX = -mapOffsetX / resolution;*/
/*  int MapOrigenY = -mapOffsetY / resolution;*/
/*  vector<point_int> points;*/
/*  for (int i = 0; i < poly.polygon_points_desplay.size(); i++) {*/
/*    bool test = true;*/
/*    for (int n = 0; n < points.size(); n++) {*/
/*      if (poly.polygon_points_desplay[i] == points[n]) {*/
/*        test = false;*/
/*      }*/
/*    }*/
/*    if (test) {*/
/*      points.push_back(poly.polygon_points_desplay[i]);*/
/*    }*/
/*  }*/
/*  geometry_msgs::PolygonStamped newPoly;*/
/*  newPoly.header.frame_id = "map";*/
/*  newPoly.header.stamp = ros::Time::now();*/
/*  newPoly.polygon.points.resize(points.size());*/
/**/
/*  for (int n = 0; n < points.size(); n++) {*/
/*    newPoly.polygon.points[n].x = (points[n].x - MapOrigenX) * resolution;*/
/*    newPoly.polygon.points[n].y = (points[n].y - MapOrigenY) * resolution;*/
/*    newPoly.polygon.points[n].z = z;*/
/*  }*/
/**/
/*  return newPoly;*/
/*}*/

double angleBetweenLines(point_int A, point_int B, point_int P, point_int Q);

// suport function for ant_step rotates the dir
point_int rotate_dir(point_int dir, bool cw);

vector<point_int> drawLine(point_int start, point_int end);

void print(string s);

vector<point_int> fillPoly(vector<point_int> PL);

vector<point> fillPoints(point start, point end, double spacing,
                         bool lastAndFirst = false);

vector<point_int> fillPoints(point_int start, point_int end, double spacing,
                             bool lastAndFirst = false);

/*//Returns int rep calss, 0=North, 1=West, 2=South, 3=East
int fillClass(int dx, int dy){
    if(dy>0) return 0;

    if(dy<0) return 2;

    if(dx<0) return 1;

    if(dx>0) return 3;

    return -1;

}


void fillPoly(vector<point_int> nPL, int value, int** map){
    vector<point_int> PL=nPL;
    for(int i=0;i<nPL.size();i++){
        PL[i]=nPL[nPL.size()-1-i];
    }
    point_int maxP={-1,-1},minP={-1,-1};
    vector<vector<bool>> newMap;
    for(int n=0; n<PL.size();n++){
        if(PL[n].x>maxP.x) maxP.x=PL[n].x;
        if(PL[n].y>maxP.y) maxP.y=PL[n].y;
        if(PL[n].x<minP.x || minP.x==-1) minP.x=PL[n].x;
        if(PL[n].y<minP.y || minP.y==-1) minP.y=PL[n].y;
    }
    newMap.resize(maxP.x-minP.x+2);
    minP.x-=1;
    for(int x=0;x<newMap.size();x++){
        newMap[x].resize(maxP.y-minP.y+1);
        for(int y=0; y<newMap[x].size();y++) newMap[x][y]=false;
    }
    int n=0,firstclass, oldclass, newclass;
    for(; n<PL.size();n++){
        int dx=PL[(n+1)%PL.size()].x-PL[n].x;
        int dy=PL[(n+1)%PL.size()].y-PL[n].y;
        if(dx==0 && dy==0) continue;
        firstclass=fillClass(dx,dy);
        n++;
        oldclass=firstclass;
        break;
    }
    for(; n<PL.size();n++){
        int dx=PL[(n+1)%PL.size()].x-PL[n].x;
        int dy=PL[(n+1)%PL.size()].y-PL[n].y;
        if(dx==0 && dy==0) continue;
        newclass=fillClass(dx,dy);
        if(newclass==0 && oldclass==2 ||
           newclass==2 && oldclass==0){
            int x=PL[n].x-minP.x;
            int y=PL[n].y-minP.y;
            ROS_INFO("1 %i,%i",x,y);
            newMap[x][y]=!newMap[x][y];
        }else if((newclass==0 || newclass==1) &&
                 (oldclass==0 || oldclass==4)){
                    int y=PL[n].y-minP.y;
                    for(int x=0;x<=PL[n].x-minP.x;x++){
                        ROS_INFO("2 %i,%i",x,y);
                        newMap[x][y]=!newMap[x][y];
                    }
        }else if((newclass==2 || newclass==3) &&
                 (oldclass==1 || oldclass==2)){
                    int y=PL[n].y-minP.y;
                    for(int x=0;x<=PL[n].x-minP.x;x++){
                        ROS_INFO("3 %i,%i",x,y);
                        newMap[x][y]=!newMap[x][y];
                    }
        }
        oldclass=newclass;
    }
    for(int x=0;x<newMap.size();x++){
        for(int y=0;y<newMap[x].size();y++){
            if(newMap[x][y])
                setMap(x+minP.x,y+minP.y,value,map);
        }
    }
}*/

#endif
