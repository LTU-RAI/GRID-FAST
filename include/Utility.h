#include "ros/ros.h"
#include <math.h>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <topology_mapping/opening_list.h>
using namespace std;

//Map setings
int mapSize=1000;
float resolution=0.2;
//scan setings
int scanSize=mapSize;
int minGroupSize=3;
int minCoridorSize=1;
int cGroupeSize=0;
int cfilterSize=15;
int cfilterWallSize=0;
int groupeNumber=60;
int numberOfDir=6;
int numberOfDirFilter=4;
int maxGapDistans=5;
int extendDevider=4;
int searchLenght=8;

//ant para
int searchLenghtClean=100;
int sercheLenthAnt=600;
int sercheLenthAntConect=2000;
int sercheLenthAntConectPath=2000;
int maxAntGap=4;
int poligonRez=4;
int poligonRezPath=10;
int minimumSercheLenght=5;

struct scanGroup{
    int start;
    int end;
    int prevGroupIndex=0;
    scanGroup **prevGroup=new scanGroup*[6];
    int nextGroupIndex=0;
    scanGroup **nextGroup=new scanGroup*[6];
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

double dist(point_int p1, point_int p2){
    point_int d={p1.x-p2.x,p1.y-p2.y};
    return sqrt(d.x*d.x+d.y*d.y);
}

bool operator==(const point_int& lhs, const point_int& rhs)
{
    return lhs.x==rhs.x && lhs.y==rhs.y;
}
struct poligon;

struct opening{
    point_int start;
    point_int end;
    bool start_is_outside;
    int label=1;
    int parent_poligon=-1;
    bool fliped=false;
    int conected_to_path=-1;

    void flip(){
        point_int t=start;
        start=end;
        end=t;
    }
};

struct poligon{
    vector<int> sidesIndex;
    vector<point_int> poligon_points;
    int label=1;
    bool inactiv=false;
    void add_point(point_int p, bool cw){
        if(cw){
            poligon_points.push_back(p);
        }else{
            poligon_points.insert(poligon_points.begin(),p);
        }
    }
};

struct Poligon_list{
    vector<poligon> poly;
    vector<int> label;

    void add(poligon p, int l){
        poly.push_back(p);
        label.push_back(l);
    }

    void remove(int index){
        poly.erase(poly.begin()+index);
        label.erase(label.begin()+index);
    }

    void clear(){
        for(int i=0;i<poly.size();i++){
            poly[i].poligon_points.clear();
            poly[i].sidesIndex.clear();
        }
        poly.clear();
        label.clear();
    }

    int size(){
        return poly.size();
    }

    geometry_msgs::PolygonStamped get_polygon(int index, int sizeMap, float resolution, double z=10.1){
        int halfMapSize=sizeMap/2;
        vector<point_int> points;
        for(int i=0; i<poly[index].poligon_points.size(); i++){
            bool test=true;
            for(int n=0; n<points.size(); n++){
                if(poly[index].poligon_points[i]==points[n]){
                    test=false;
                }
            }
            if(test){
                points.push_back(poly[index].poligon_points[i]);
            }
        }
        geometry_msgs::PolygonStamped newPoly;
        newPoly.header.frame_id = "map";
        newPoly.header.stamp = ros::Time::now();
        newPoly.polygon.points.resize(points.size());

        for(int n=0; n<points.size(); n++){
            newPoly.polygon.points[n].x=(points[n].x-halfMapSize)*resolution;
            newPoly.polygon.points[n].y=(points[n].y-halfMapSize)*resolution;
            newPoly.polygon.points[n].z=z;
        }

        return newPoly;
    }
};

struct ant_data{
    point_int dir={0,0};
    point_int end;
    bool emty_cell=false;
};

vector<opening> oplist;

//rotate all points in op list around the maps center
vector<opening> rotate_points(vector<opening> op, const double rotation){
    vector<opening> newOp;
    newOp.resize(op.size());
    float cosA=cos(rotation);
    float sinA=sin(rotation);
    int halfMapSize=mapSize/2;
    for(int i=0; i<op.size();i++){
        for(int sids=0; sids<2;sids++){
            point_int p=op[i].start;
            if(sids==1){
                p=op[i].end;
            }
            //using a 2d rotation matrix to rotate points
            int newX=int(std::round(((p.x-halfMapSize) *cosA-(p.y-halfMapSize)*sinA)))+halfMapSize;
            int newY=int(std::round(((p.x-halfMapSize) *sinA+(p.y-halfMapSize)*cosA)))+halfMapSize;
            p.x=newX;
            p.y=newY;
            if(sids==0){
                newOp[i].start=p;
            }else{
                newOp[i].end=p;
            }
        }
        newOp[i].start_is_outside=op[i].start_is_outside;
    }
    return newOp;        
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
        while(map[start.x+step.dir.x*l][start.y+step.dir.y*l]==0){
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
    point_int checkDir=direction;
    int checkLenght=1;

    for(int d=0;d<8;d++){
        bool check=true;
        for(int c=0; c<checkLenght; c++){  
            if(map[start.x+step.dir.x+checkDir.x*c][start.y+step.dir.y+checkDir.y*c]!=0){
                check=false;
                break;
            }
            
        }
        if(check){
            step.end.x=start.x+step.dir.x;
            step.end.y=start.y+step.dir.y;
            step.emty_cell=false;
            point_int dir=step.dir;
            for(int e=0;e<8;e++){
                    
                if(map[step.end.x+dir.x][step.end.y+dir.y]==-1){
                    step.emty_cell=true;
                    break;
                }
                dir=rotate_dir(dir,clockwise);
            }

            return step;
        }

        step.dir=rotate_dir(step.dir,clockwise);
        checkDir=rotate_dir(checkDir,clockwise);
    }
    //ROS_INFO("no step found");
    return step;
} 

bool checkForWall(opening o,int maximumWallCount, int** map){
    int wallcount=0;
    double opLenght=dist(o.start,o.end);
    point normdVop={(o.end.x-o.start.x)/(opLenght*1.2),(o.end.y-o.start.y)/(opLenght*1.2)};
    for(int wallScan=0;wallScan<(int)(opLenght*1.2);wallScan++){
        point_int pp={o.start.x+(int)(std::round(normdVop.x*wallScan)), o.start.y+(int)(std::round(normdVop.y*wallScan))};
        if(map[pp.x][pp.y]==100){
            wallcount+=1;
        }
    }
    if(wallcount<maximumWallCount){
        return false;
    }else{
        return true;
    }
}

//suport function for intersect_line
bool ccw(point A,point B,point C){
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
}

//Return true if line segments o1 and o2 intersect.
bool intersect_line(opening o1,opening o2, int** map){
    double l=.6;
    double l1 = dist(o1.start,o1.end);
    double l2 = dist(o2.start,o2.end);
    point n1={((o1.end.x-o1.start.x)/l1)*l, ((o1.end.y-o1.start.y)/l1)*l};
    point n2={((o2.end.x-o2.start.x)/l2)*l, ((o2.end.y-o2.start.y)/l2)*l};
    point p1[]={{(double)o1.start.x-n1.x,(double)o1.start.y-n1.y},{(double)o1.end.x+n1.x,(double)o1.end.y+n1.y}};
    point p2[]={{(double)o2.start.x-n2.x,(double)o2.start.y-n2.y},{(double)o2.end.x+n2.x,(double)o2.end.y+n2.y}};
    point currentO[2], testingO[2];
    for(int side=0;side<2;side++){
        if(side==0){
            currentO[0]=p1[0];
            currentO[1]=p1[1];
            testingO[0]=p2[0];
            testingO[1]=p2[1];
        }else{
            currentO[0]=p2[0];
            currentO[1]=p2[1];
            testingO[0]=p1[0];
            testingO[1]=p1[1];
        }

        double ang=atan2(currentO[1].y-currentO[0].y,currentO[1].x-currentO[0].x);
        point oTest[]={testingO[0],testingO[1]};
        point oTest3[]={testingO[0],testingO[1]};
        double cosA=cos(-ang);
        double sinA=sin(-ang);
        for(int i=0; i<2;i++){
            //using a 2d rotation matrix to rotate points
            double newX=((oTest[i].x-currentO[0].x) *cosA-(oTest[i].y-currentO[0].y)*sinA);
            double newY=((oTest[i].x-currentO[0].x) *sinA+(oTest[i].y-currentO[0].y)*cosA);
            oTest[i].x=newX;
            oTest[i].y=newY;
        }
        
        if(oTest[0].y<0 && oTest[1].y<0 || oTest[0].y>0 && oTest[1].y>0){ return false;}
        else{
            //ROS_INFO("%f, %f, %f, %f, %f",ang,oTest3[0].x,oTest3[0].y,oTest3[1].x,oTest3[1].y);
            //ROS_INFO("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",ang,oTest[0].x,oTest[0].y,oTest[1].x,oTest[1].y,currentO[0].x,currentO[0].y,currentO[1].x,currentO[1].y,testingO[0].x,testingO[0].y,testingO[1].x,testingO[1].y);
             }
    }
    return true;
}

//checks and get index to first found opening at a point, type: 1 check for start, 2 check for end, 3 check for both. return -1 of no opening is found.
int check_and_get_opening(point_int position, int type, bool only_standard_opening=false){
    int oIndex=-1;
    
    for(int i=0; i<oplist.size(); i++){
        if(oplist[i].label!=-1 && (!only_standard_opening || oplist[i].label==1) &&
            (type==1 && oplist[i].start==position || type==2 && oplist[i].end==position ||
            type==3 && (oplist[i].start==position || oplist[i].end==position))){
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
bool fitToCorridor(opening *op,int inSearchLenght,int** map, bool limitBothSids=false, bool oneDir=false, vector<point_int> *sida1=NULL,vector<point_int> *sida2=NULL){
    vector<point_int> s1, s2;
    ant_data search_step;
    int number_of_emty=0;
    double minimumDistans=-1;
    bool cw;
    int num_steps;
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
                    for(int k=0; k<s1.size(); k++){
                        if(sides==0){
                            sida1->push_back(s1[k]);
                        }else{
                            sida1->insert(sida1->begin() ,s1[k]);
                        }
                    }
                    s1.clear();
                    minimumDistans=d;
                    m=0;
                    if(op->start_is_outside){
                        op->start=search_step.end;
                    }else{
                        op->end=search_step.end;
                    }
                    
                    
                }
            }else{
                number_of_emty+=1;
            }
        }
    }
    if(minimumDistans<0){
        return false;
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
            if(sida1!=NULL){
                s2.push_back(search_step.end);
            }

            if(!limitBothSids && check_for_opening(search_step.end,op->start_is_outside?1:2)||
                limitBothSids && check_for_opening(search_step.end,3))break;

            if(!search_step.emty_cell){
                double d=dist(search_step.end,op->start_is_outside?op->start:op->end);
                if(minimumDistans<0 || d<minimumDistans){
                    for(int k=0; k<s2.size(); k++){
                        if(sides==0){
                            sida2->push_back(s2[k]);
                        }else{
                            sida2->insert(sida1->begin() ,s2[k]);
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
            }else{
                number_of_emty+=1;
            }
        }
    }
    if(minimumDistans<0){
        return false;
    }
    return true;        
}