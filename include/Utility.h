#include "ros/ros.h"
#include <math.h>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <topology_mapping/opening_list.h>
#include <topology_mapping/topometricMap.h>
using namespace std;

//Map setings
int mapSizeX=10;
int mapSizeY=10;
float resolution=0;
float mapOffsetX=0;
float mapOffsetY=0;
float mapHight=0;
//scan setings
int minGroupSize=4;
int minCoridorSize=4;
int cfilterSize=2;
int objectFilterMaxStep=1;
int numberOfDir=6;
int numberOfDirFilter=numberOfDir;
int extendDevider=2;
double dw=0.2;
int searchLenght=200;

//ant para
int searchLenghtClean=50;
int sercheLenthAnt=600;
int sercheLenthAntConect=2000;
int sercheLenthAntConectPath=2000;
int maxAntGap=2;
int poligonRez=4;
int poligonRezPath=10;
int voronoiRez=6;
int minimumSercheLenght=5;

//Debugging
bool show_removed_openings=false;
int **dMap;
struct scanGroup{
    int start;
    int end;
    int row;
    vector<scanGroup*> prevGroup;
    vector<scanGroup*> nextGroup;
};

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

struct mapTransform{
    point_int tpos, rpos;
};

double dist(point_int p1, point_int p2){
    point_int d={p1.x-p2.x,p1.y-p2.y};
    return sqrt(d.x*d.x+d.y*d.y);
}

bool operator==(const point_int& lhs, const point_int& rhs)
{
    return lhs.x==rhs.x && lhs.y==rhs.y;
}

struct opening{
    point_int start;
    point_int end;
    vector<point_int> occupied_points;
    int sideToMove=3;//1:start,2:end,3:none
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

vector<opening> oplist;

geometry_msgs::PolygonStamped get_polygon(poligon poly, float resolution, double z=0.1){
    int MapOrigenX=-mapOffsetX/resolution;
    int MapOrigenY=-mapOffsetY/resolution;
    vector<point_int> points;
    for(int i=0; i<poly.poligon_points_desplay.size(); i++){
        bool test=true;
        for(int n=0; n<points.size(); n++){
            if(poly.poligon_points_desplay[i]==points[n]){
                test=false;
            }
        }
        if(test){
            points.push_back(poly.poligon_points_desplay[i]);
        }
    }
    geometry_msgs::PolygonStamped newPoly;
    newPoly.header.frame_id = "map";
    newPoly.header.stamp = ros::Time::now();
    newPoly.polygon.points.resize(points.size());

    for(int n=0; n<points.size(); n++){
        newPoly.polygon.points[n].x=(points[n].x-MapOrigenX)*resolution;
        newPoly.polygon.points[n].y=(points[n].y-MapOrigenY)*resolution;
        newPoly.polygon.points[n].z=z;
    }

    return newPoly;
}

int getMap(int x, int y,int **map){
    if(x<0||x>mapSizeX) return -1;
    if(y<0||y>mapSizeY) return -1;

    return map[x][y];
}

void setMap(int x, int y,int value,int **map){
    if(x<0||x>mapSizeX) return;
    if(y<0||y>mapSizeY) return;

    map[x][y]=value;
}

//suport function for ant_step rotates the dir 
point_int rotate_dir(point_int dir, bool cw){
    point_int newDir[]={{-1,1},{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0}, {0,0}};
    for(int i=0; i<8; i++){
        if(newDir[i]==dir){
            int next_index=cw?i+1:i-1;
            if(next_index==-1)next_index=7;
            next_index=next_index%8;
            return newDir[next_index];
        }
    }
    return newDir[8];
}

//Function to take one step using an ant algorithm 
ant_data ant_step(point_int start, bool clockwise, point_int direction, int** map){
    int newDirx, newDiry;
    ant_data step;
    
    step.end=start;

    if(direction.x==0 && direction.y==0){
        step.dir={1,0};
        int c=0;
        int l=1;
        while(getMap(start.x+step.dir.x*l,start.y+step.dir.y*l,map)==0){
            newDirx=clockwise?step.dir.y:-step.dir.y;
            newDiry=clockwise?-step.dir.x:step.dir.x;
            step.dir.x=newDirx;
            step.dir.y=newDiry;
            if(c>4){
                l++;
                c=0;
            }
            c++;
        }if(l>2){
            start.x+=step.dir.x*(l-1);
            start.y+=step.dir.y*(l-1);
        }
    }else{
        step.dir=rotate_dir(direction,!clockwise);
        step.dir=rotate_dir(step.dir,!clockwise);
    }

    for(int d=0;d<8;d++){
        bool check=false;
        if(getMap(start.x+step.dir.x,start.y+step.dir.y,map)==0){
            point_int dir=step.dir;
            if(dir.x*dir.y!=0){
                dir=rotate_dir(dir,clockwise);
                if(getMap(start.x+dir.x,start.y+dir.y,map)==0)
                    check=true;
            }else{check=true;}
        }
        if(check){
            step.end.x=start.x+step.dir.x;
            step.end.y=start.y+step.dir.y;
            step.emty_cell=false;
            point_int dir={0,1};
            step.emty_cell=false;
            for(int e=0;e<4;e++){
                    
                if(getMap(step.end.x+dir.x,step.end.y+dir.y,map)==-1){
                    step.emty_cell=true;
                    break;
                }
                dir=rotate_dir(dir,clockwise);
                dir=rotate_dir(dir,clockwise);
            }
            //ROS_INFO("%i, %i",step.dir.x,step.dir.y);
            return step;
        }

        step.dir=rotate_dir(step.dir,clockwise);
        //step.dir=rotate_dir(step.dir,clockwise);
    }
    //ROS_INFO("no step found");
    return step;
} 

bool checkForWall(opening o,int maximumWallCount, int** map){
    int wallcount=0;
    double l=2;
    double opLenght=dist(o.start,o.end);
    point normdVop={(o.end.x-o.start.x)/(opLenght*l),(o.end.y-o.start.y)/(opLenght*l)};
    for(int wallScan=0;wallScan<(int)(opLenght*l);wallScan++){
        point_int pp={o.start.x+(int)(std::round(normdVop.x*wallScan)), o.start.y+(int)(std::round(normdVop.y*wallScan))};
        if(getMap(pp.x,pp.y,map)==100){
            wallcount+=1;
        }
    }
    if(wallcount<maximumWallCount){
        return false;
    }else{
        return true;
    }
}

vector<point_int> generateOpeningPoints(opening *o){
    vector<point_int> p;
    if(o->occupied_points.size()!=0 && 
    (o->start==o->occupied_points[0] && 
    o->end==o->occupied_points[o->occupied_points.size()-1]||
    o->end==o->occupied_points[0] && 
    o->start==o->occupied_points[o->occupied_points.size()-1])){
        
        p=o->occupied_points;
    
        return p;
    }

    point_int start=o->start;
    point_int end=o->end;
    point_int lenght={end.x-start.x,end.y-start.y};
    point_int step={lenght.x/std::abs(lenght.x),lenght.y/std::abs(lenght.y)};
    lenght={std::abs(lenght.x),std::abs(lenght.y)};
    point_int steps_taken={0,0};
    
    p.push_back(start);
    
    for(int s=0;s<lenght.x+lenght.y;s++){
        if(steps_taken.x*lenght.y<steps_taken.y*lenght.x && lenght.x!=0 || lenght.y==0){
            steps_taken.x+=1;
            start.x+=step.x;
        }else{
            steps_taken.y+=1;
            start.y+=step.y;
        }

        p.push_back(start);
        
    }
    o->occupied_points=p;
        
    return p;
}

//Return true if line segments o1 and o2 intersect.
bool intersect_line(opening *o1,opening *o2){
    point_int p1Max, p1Min, p2Max,p2Min;
    p1Max.x=std::max(o1->start.x,o1->end.x);
    p1Max.y=std::max(o1->start.y,o1->end.y);
    p1Min.x=std::min(o1->start.x,o1->end.x);
    p1Min.y=std::min(o1->start.y,o1->end.y);
    p2Max.x=std::max(o2->start.x,o2->end.x);
    p2Max.y=std::max(o2->start.y,o2->end.y);
    p2Min.x=std::min(o2->start.x,o2->end.x);
    p2Min.y=std::min(o2->start.y,o2->end.y);
    if(!(p1Min.x<=p2Max.x && p1Max.x>=p2Min.x &&
         p1Min.y<=p2Max.y && p1Max.y>=p2Min.y)) return false;
    
    vector<point_int> p1,p2;
    p1= generateOpeningPoints(o1);
    p2= generateOpeningPoints(o2);
    
    for(int i1=0;i1<p1.size();i1++){
        for(int i2=0;i2<p2.size();i2++){
            if(p1[i1]==p2[i2]) return true;
        }
    }
    return false;
}

//Move an opening to the bigest opening of two walls
void moveOpeningIntoCoridor(opening *o, int **map){
    int bigestCoridor=0;
    int sIndex=-1;
    vector<point_int> p=generateOpeningPoints(o);
    for(int i=0;i<p.size();i++){
        if(sIndex==-1 && getMap(p[i].x,p[i].y,map)==0){
            sIndex=i;
        }else if(sIndex!=-1 && getMap(p[i].x,p[i].y,map)!=0 
                 && i!=0 && i-1-sIndex>bigestCoridor){
            o->start=p[sIndex];
            o->end=p[i-1];
            bigestCoridor=i-1-sIndex;
            sIndex=-1;
        }
    }
}

//checks and get index to first found opening at a point, type: 1 check for start, 2 check for end, 3 check for both. return -1 of no opening is found.
int check_and_get_opening(point_int position, int type, bool only_standard_opening=false, int exclude=-1){
    int oIndex=-1;
    
    for(int i=0; i<oplist.size(); i++){
        if(oplist[i].label<10 && (!only_standard_opening || oplist[i].label==1) &&
            (type==1 && oplist[i].start==position || type==2 && oplist[i].end==position ||
            type==3 && (oplist[i].start==position || oplist[i].end==position))){
                if(i==exclude) continue;
                oIndex=i;
        }
    }

    return oIndex;
}

//checks if a point is overlaping an opening start and end, type: 1 check for start, 2 check for end, 3 check for both.
bool check_for_opening(point_int position, int type){
    bool testCheck=true;
    
    if(check_and_get_opening(position, type)==-1) testCheck=false;

    return testCheck;
}
//Moves the start and end points of op to minimize its length, return true if op was successfully moved.
bool fitToCorridor(opening *op,int inSearchLenght,int** map, bool limitBothSids=false, bool oneDir=false){
    if(op->sideToMove==3) return true;
    point_int *moveP;
    point_int fixedP;
    if(op->sideToMove==1){
        moveP=&op->start;
        fixedP=op->end;
    }else{
        moveP=&op->end;
        fixedP=op->start;
    }
    point_int alt1, alt2;
    for(int sides=0;sides<2;sides++){
        vector<point_int> posP;
        ant_data step;
        step.end=*moveP;
        posP.push_back(step.end);

        for(int s=0;s<inSearchLenght;s++){
            step=ant_step(step.end,sides,step.dir,map);
            if(step.end==fixedP) break;
            if(step.emty_cell) continue;
            posP.push_back(step.end);
        }
        point_int test, hbest, best;
        test=*moveP;
        best=test;
        hbest=test;
        double bestLength=0;
        bool first=true, changed=false;
        int sIndex=0, eIndex=0, decrisCount=0;
        for(int i=0;i<=posP.size();i++){
            if(i==posP.size()){
                best=hbest;
                changed=true;
                break;
            }
            test=posP[i];
            double lenght=dist(test,fixedP);
            if(lenght<minGroupSize) break;
            if(first || lenght<bestLength){
                opening o;
                o.start=fixedP;
                o.end=test;
                if(!checkForWall(o,1,map)){
                    first=false;
                    bestLength=lenght;
                    hbest=test;
                }
                
            }else{
                if(decrisCount>=3){
                    best=hbest;
                    changed=true;
                    decrisCount=0;
                }else decrisCount+=1;
            }
        }
        if(sides){
            alt2=best;
        }else{
            alt1=best;
        }
    }
    
    if(dist(fixedP,alt1)<dist(fixedP,alt2)){
        *moveP=alt1;
    }else *moveP=alt2;
    
    return true;
}
bool fitToCorridor2(opening *op,int inSearchLenght,int** map, bool limitBothSids=false, bool oneDir=false, vector<point_int> *sida1=NULL,vector<point_int> *sida2=NULL){
    vector<point_int> s1, s2;
    ant_data search_step;
    int number_of_emty=0;
    double minimumDistans=-1;
    bool cw;
    int num_steps;
    bool returnValue=true;
    if(oneDir){
        op->start_is_outside=false;
    }
    for(int sides=0;sides<(oneDir?1:2);sides++){
        cw=sides==0;
        number_of_emty=0;
        num_steps=0;
        search_step.end=op->start_is_outside?op->start:op->end;
        search_step.dir={0,0};
        for(int m=0;m<inSearchLenght && number_of_emty<maxAntGap;m++){
            num_steps+=1;
            search_step=ant_step(search_step.end,cw,search_step.dir,map);
            if(sida1!=NULL){
                s1.push_back(search_step.end);
            }
            
            if(limitBothSids && check_for_opening(search_step.end,3))break;

            if(!search_step.emty_cell){
                double d=dist(search_step.end,!op->start_is_outside?op->start:op->end);
                if((minimumDistans<0 || d<minimumDistans)){
                    opening tOp;
                    tOp.start=!op->start_is_outside?op->start:op->end;
                    tOp.end=search_step.end;
                    //if(checkForWall(tOp,1,map)) continue;

                    for(int k=0; k<s1.size(); k++){
                        if(sides==0){
                            sida1->push_back(s1[k]);
                        }else{
                            sida1->insert(sida1->begin() ,s1[k]);
                        }
                    }
                    s1.clear();
                    m=0;
                    minimumDistans=d;
                    
                    if(op->start_is_outside){
                        op->start=search_step.end;
                    }else{
                        op->end=search_step.end;
                    }
                    
                }
                if(number_of_emty>0){
                    number_of_emty-=1;
                }
            }else{
                number_of_emty+=1;
            }
        }
    }
    if(minimumDistans<0){
        returnValue=false;
    }
    minimumDistans=-1;
    for(int sides=0; sides<1; sides++){
        cw=op->start_is_outside;
        number_of_emty=0;
        num_steps=0;
        search_step.end=!op->start_is_outside?op->start:op->end;
        search_step.dir={0,0};
        for(int m=0;m<inSearchLenght && number_of_emty<maxAntGap;m++){
            num_steps+=1;
            search_step=ant_step(search_step.end,cw,search_step.dir,map);
            if(sida2!=NULL){
                s2.push_back(search_step.end);
            }

            if(!limitBothSids && check_for_opening(search_step.end,op->start_is_outside?1:2)||
                limitBothSids && check_for_opening(search_step.end,3))break;

            if(!search_step.emty_cell){
                double d=dist(search_step.end,op->start_is_outside?op->start:op->end);
                if(minimumDistans<0 || d<minimumDistans){
                    opening tOp;
                    tOp.start=!op->start_is_outside?op->start:op->end;
                    tOp.end=search_step.end;
                    //if(checkForWall(tOp,1,map)) continue;
                    for(int k=0; k<s2.size(); k++){
                        if(sides==0){
                            sida2->push_back(s2[k]);
                        }else{
                            sida2->insert(sida2->begin() ,s2[k]);
                        }
                    }
                    s2.clear();
                    m=0;
                    minimumDistans=d;
                    if(!op->start_is_outside){
                        op->start=search_step.end;
                    }else{
                        op->end=search_step.end;
                    }
                }
                if(number_of_emty>0){
                    number_of_emty-=1;
                }
            }else{
                number_of_emty+=1;
            }
        }
    }
    if(minimumDistans<0){
        returnValue=false;
    }
    return returnValue;        
}
vector<point_int> fillPoly(vector<point_int> PL){
    point_int maxP={-1,-1},minP={-1,-1};
    vector<vector<bool>> newMap;
    for(int n=0; n<PL.size();n++){
        if(PL[n].x>maxP.x) maxP.x=PL[n].x;
        if(PL[n].y>maxP.y) maxP.y=PL[n].y;
        if(PL[n].x<minP.x || minP.x==-1) minP.x=PL[n].x;
        if(PL[n].y<minP.y || minP.y==-1) minP.y=PL[n].y;
    }
    newMap.resize(maxP.x-minP.x+1);
    for(int x=0;x<newMap.size();x++){
        newMap[x].resize(maxP.y-minP.y+1);
        for(int y=0; y<newMap[x].size();y++) newMap[x][y]=false;
    }
    for(int n=0; n<PL.size();n++){
        int dy=PL[(n+1)%PL.size()].y-PL[n].y;
        if(dy==0) continue;
        int y=dy==1? PL[n].y:PL[n].y-1;
        y-=minP.y;
        if(y<0) continue;
        for(int x=0;x<=PL[n].x-minP.x;x++){
            newMap[x][y]=!newMap[x][y];
        }
    }
    vector<point_int> filledPoints;
    for(int x=0;x<newMap.size();x++){
        for(int y=0;y<newMap[x].size();y++){
            if(newMap[x][y]){
                point_int p={x+minP.x,y+minP.y};
                filledPoints.push_back(p);
            }
        }
    }
    return filledPoints;
}

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
