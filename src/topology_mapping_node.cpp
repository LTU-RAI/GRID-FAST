#include "ros/ros.h"
#include <thread>
#include <math.h>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>

using namespace std;

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
    int label=1;
    bool conected=false;
};

struct poligon{
    vector<opening> sides;
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
        poly.clear();
        label.clear();
    }

    int size(){
        return poly.size();
    }

    geometry_msgs::PolygonStamped get_polygon(int index, int sizeMap, float resolution, double z=10.1){
        int halfMapSize=sizeMap/2;
        vector<point_int> points;
        vector<double> angle;
        for(int i=0; i<poly[index].sides.size(); i++){
            bool test=true;
            for(int n=0; n<points.size(); n++){
                if(poly[index].sides[i].start==points[n]){
                    test=false;
                }
            }
            if(test){
                points.push_back(poly[index].sides[i].start);
            }
            test=true;
            for(int n=0; n<points.size(); n++){
                if(poly[index].sides[i].end==points[n]){
                    test=false;
                }
            }
            if(test){
                points.push_back(poly[index].sides[i].end);
            }
            
            
        }

        point center={0,0};
        for(int i=0; i<points.size(); i++){
            center.x+=points[i].x;
            center.y+=points[i].y;
        }
        center.x=center.x/points.size();
        center.y=center.y/points.size();
        for(int i=0; i<points.size(); i++){
            angle.push_back(atan2(points[i].y-center.y,points[i].x-center.x));
        }

        geometry_msgs::PolygonStamped newPoly;
        newPoly.header.frame_id = "map";
        newPoly.header.stamp = ros::Time::now();
        newPoly.polygon.points.resize(points.size());
        int c=0;
        while(points.size()>0){
            int minAngIndex=-1;
            double minAng=6;
            for(int i=0; i<points.size(); i++){
                if(angle[i]<minAng){
                    minAngIndex=i;
                    minAng=angle[i];
                }
            }
            newPoly.polygon.points[c].x=(points[minAngIndex].x-halfMapSize)*resolution;
            newPoly.polygon.points[c].y=(points[minAngIndex].y-halfMapSize)*resolution;
            newPoly.polygon.points[c].z=z;
            c+=1;
            
            points.erase(points.begin()+minAngIndex);
            angle.erase(angle.begin()+minAngIndex);
        }

        return newPoly;
    }
};

struct Opening_list
{
    vector<opening> *list;
    int lenght=0;

    void init(int size){
        list=new vector<opening>[size];
        lenght=size;
    }

    int size(){
        int s=0;
        for(int i=0; i<lenght; i++){
            s+=list[i].size();
        }
        return s;
    }

    opening get(int index){
        int c=0;
        for(int i=0; i<lenght; i++){
            if(index-c<list[i].size()){
                return list[i][index-c];
            }

            c+=list[i].size();
        }
        ROS_INFO("Warning: last index of vector reached");
        opening o;
        return o;
    }
};



struct ant_data{
    point_int dir={0,0};
    point_int end;
    bool emty_cell=false;
};
                                   

class TopologyMapping{
    private:
        //ROS nh
        ros::NodeHandle nh;
        //subs
        ros::Subscriber subOccupancyMap;
        //pub
        ros::Publisher pubTopoMap;
        ros::Publisher pubTopoPoly_debug;
        ros::Publisher pubTopoPoly;

        //msg
        nav_msgs::OccupancyGrid topoMapMsg;
        //Map
        int **Map;
        int **topMap;
        int mapSize=1000;
        float resolution=0.2;
        //scan setings
        int scanSize=mapSize;
        int minGroupSize=5;
        int minCoridorSize=6;
        int cGroupeSize=0;
        int cfilter=0;
        int cfilterSize=15;
        int groupeNumber=15;
        int numberOfDir=25;
        int maxGapDistans=10;
        int extendDevider=4;
        int searchLenght=8;

        //global var
        int **scanMap;
        int **scanMapOut;
        int **scanMapOutTransform;
        int **scanMapOutHolder;
        scanGroup **scanGarray;
        Opening_list oplist;
        Poligon_list poly_list;
        int *scanGroupIndex;
        int currentRotationIndex=0;

        //ant para
        int searchLenghtClean=100;
        int sercheLenthAnt=400;
        int sercheLenthAntConect=1000;
        int maxAntGap=4;

    public:
        TopologyMapping(){
            subOccupancyMap= nh.subscribe("/occupancy_map_global",5,&TopologyMapping::updateMap, this);
            pubTopoMap=nh.advertise<nav_msgs::OccupancyGrid>("/topology_map",5);
            pubTopoPoly_debug=nh.advertise<jsk_recognition_msgs::PolygonArray>("/topology_poly_debug",5);
            pubTopoPoly=nh.advertise<jsk_recognition_msgs::PolygonArray>("/topology_poly",5);
            loadMemory();
            initializeTopoMap();
        }
        

    void loadMemory(){
        //alocate for Maps
        oplist.init(numberOfDir);
        scanMap=new int*[scanSize];
        scanMapOut=new int*[scanSize];
        scanMapOutTransform=new int*[scanSize];
        scanMapOutHolder=new int*[scanSize];
        for(int i=0; i<scanSize;i++){
            scanMapOut[i]=new int[scanSize];
            scanMap[i]=new int[scanSize];
            scanMapOutTransform[i]=new int[scanSize];
            scanMapOutHolder[i]=new int[scanSize];
        }
        for (int i = 0; i < scanSize; ++i){
            for (int j = 0; j < scanSize; ++j){
                scanMapOut[i][j] = -1;
            }
        }
        Map=new int*[mapSize];
        topMap=new int*[mapSize];
        for(int i=0;i<mapSize;i++){
            Map[i]=new int[mapSize];
            topMap[i]=new int[mapSize];
        }
        
        for (int i = 0; i < mapSize; ++i){
            for (int j = 0; j < mapSize; ++j){
                Map[i][j] = -1;
                topMap[i][j] = -1;
            }
        }
        scanGarray=new scanGroup*[scanSize];
        scanGroupIndex=new int[scanSize];
        for (int i = 0; i < scanSize; i++){
            scanGarray[i]=new scanGroup[groupeNumber];
            scanGroupIndex[i]=0;
        }
    }


    void spin(){
        ros::Rate rate(100); // Hz
        
        while (ros::ok()){
            
            for(int index=0; index<oplist.lenght; index++) oplist.list[index].clear();
            
            for(int i=0; i<numberOfDir;i++) topologyScan();
            for(int i=0; i<oplist.lenght; i++){
                for(int j=0; j<oplist.list[i].size(); j++){
                    if(oplist.list[i][j].label==-1 && true){
                        oplist.list[i].erase(oplist.list[i].begin()+j);
                        j-=1;
                    }
                }
            }
           
            if(oplist.size()>1)creatPoligonList();
            
            pubMap();
            
            ros::spinOnce();
            rate.sleep();
        }
    }
    bool ccw(point_int A,point_int B,point_int C){
        return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
    }

    // Return true if line segments AB and CD intersect
    bool intersect_line(opening o1,opening o2){
        point_int A=o1.start, B=o1.end, C=o2.start, D=o2.end; 
        return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D);
    }

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

    ant_data ant_step(point_int start, bool clockwise, point_int direction){
        int newDirx, newDiry;
        ant_data step;
        
        step.end=start;

        if(direction.x==0 && direction.y==0){
            step.dir={1,0};
            int c=0;
            int l=1;
            while(Map[start.x+step.dir.x*l][start.y+step.dir.y*l]==0){
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
        }
        for(int d=0;d<8;d++){
            if(Map[start.x+step.dir.x][start.y+step.dir.y]==0){
                step.end.x=start.x+step.dir.x;
                step.end.y=start.y+step.dir.y;
                step.emty_cell=false;
                point_int dir=step.dir;
                for(int e=0;e<4;e++){
                        
                    if(Map[step.end.x+dir.x][step.end.y+dir.y]==-1){
                        step.emty_cell=true;
                        break;
                    }
                    newDirx=dir.y;
                    newDiry=-dir.x;
                    dir.x=newDirx;
                    dir.y=newDiry;
                }

                return step;
            }
            step.dir=rotate_dir(step.dir,clockwise);
            
        }
        ROS_INFO("no step found");
        return step;
    } 

    bool fitToCoridor(point_int *p1, point_int *p2,int inSearchLenght, int startX, bool direction, bool up){
        ant_data search_step;
        ant_data search_step_d1;
        ant_data search_step_d2;
        double minimumDistans1=-1;
        double minimumDistans2=-1;
        double minimumDistans;
        point_int newP[2];
        bool cw;
        int sides=0;
        for(;sides<2;sides++){
            cw=sides==0?!up:up;
            int prevX=startX;
            minimumDistans=-1;
            search_step.end=*p1;
            search_step.dir={0,0};
            for(int m=0;m<inSearchLenght;m++){
                search_step=ant_step(search_step.end,cw,search_step.dir);
                prevX=search_step.end.x;

                if(!search_step.emty_cell){
                    double d=dist(search_step.end,*p2);
                    if(minimumDistans<0 || d<minimumDistans){
                        minimumDistans=d;
                        newP[sides]=search_step.end;
                    }
                }
            }
            if(sides==0){
                minimumDistans1=minimumDistans;
                search_step_d1=search_step;
            }else{
                minimumDistans2=minimumDistans;
                search_step_d2=search_step;
            }
        }
        ROS_INFO("%f, %f",minimumDistans1,minimumDistans2);
        if(minimumDistans1<minGroupSize && minimumDistans2<minGroupSize){
            return false;
        }

        if(minimumDistans1<minimumDistans2 || minimumDistans2<minGroupSize){
            search_step=search_step_d1;
            minimumDistans=minimumDistans1;
            cw=up;
            sides=0;
        }else{
            sides=1;
        }
        
        int prevX=search_step.end.x;
        for(int m=0;m<inSearchLenght;m++){
            search_step=ant_step(search_step.end,cw,search_step.dir);
            prevX=search_step.end.x;

            if(!search_step.emty_cell){
                double d=dist(search_step.end,*p2);
                if(d<minimumDistans){
                    minimumDistans=d;
                    newP[sides]=search_step.end;
                    m=0;
                }
            }
        }
        *p1=newP[sides];
        minimumDistans=-1;
        search_step.end=*p2;
        search_step.dir={0,0};
        cw=!up;
        for(int m=0;m<inSearchLenght;m++){
            search_step=ant_step(search_step.end,cw,search_step.dir);
            if(search_step.end.x>prevX && direction || search_step.end.x<prevX && !direction){
                break;
            }
            prevX=search_step.end.x;

            if(!search_step.emty_cell){
                double d=dist(search_step.end,*p1);
                if(minimumDistans<0 || d<minimumDistans){
                    minimumDistans=d;
                    //*p2=search_step.end;
                    m=0;
                }
            }
        }
        if(minimumDistans<minGroupSize){
            return false;
        }

        return true;
    }

    void rotate_points(point_int *p,const int size, const double rotation, const int sizeMap){
        float cosA=cos(rotation);
        float sinA=sin(rotation);
        int halfMapSize=sizeMap/2;
        for(int i=0; i<size; i++){
            int newX=int((((p[i].x-halfMapSize) *cosA-(p[i].y-halfMapSize)*sinA)))+halfMapSize;
            int newY=int((((p[i].x-halfMapSize) *sinA+(p[i].y-halfMapSize)*cosA)))+halfMapSize;
            p[i].x=newX;
            p[i].y=newY;
        }

    }


    void rotateMap(float angel,int sizeMap,int **mapOut,int **map){
        float cosA=cos(angel);
        float sinA=sin(angel);
        int halfMapSize=sizeMap/2;
        for(int x=0;x<sizeMap;x++){
            for(int y=0;y<sizeMap;y++){
                int newX=int((((x-halfMapSize) *cosA-(y-halfMapSize)*sinA)))+halfMapSize;
                int newY=int((((x-halfMapSize) *sinA+(y-halfMapSize)*cosA)))+halfMapSize;
                if((newX<0||newX>=sizeMap)||(newY<0||newY>=sizeMap)){
                    mapOut[x][y]=-1;
                }else{
                    mapOut[x][y]=map[newX][newY];
                }

            }
        }
    }

    bool checkSquareIntersecting(point s1[4],point s2[4]){
        point *poly[]={s1,s2};
        for(int index=0; index<2;index++){
            for(int i=0;i<4;i++){
                point p1=poly[index][i];
                point p2=poly[index][(i+1)%4];

                point normal;
                normal.x=p2.y-p1.y;
                normal.y=p1.x-p2.x;

                int projected = normal.x * s1[0].x + normal.y * s1[0].y;
                int minA=projected;
                int maxA=projected;
                for(int j=1;j<4;j++){
                    projected = normal.x * s1[j].x + normal.y * s1[j].y;
                    if (projected < minA)
                        minA = projected;
                    if (projected > maxA)
                        maxA = projected;
                }
                
                projected = normal.x * s2[0].x + normal.y * s2[0].y;
                int minB=projected;
                int maxB=projected;
                for(int j=1;j<4;j++){
                    projected = normal.x * s2[j].x + normal.y * s2[j].y;
                    if (minB == 0 || projected < minB)
                        minB = projected;
                    if (maxB == 0 || projected > maxB)
                        maxB = projected;
                }

                if (maxA < minB || maxB < minA){
                    return false;
                }
            }
            
        }
        return true;
    }


    void initializeTopoMap(){
        // initialization of customized map message
        topoMapMsg.header.frame_id = "map";
        topoMapMsg.info.width = mapSize;
        topoMapMsg.info.height = mapSize;
        topoMapMsg.info.resolution = resolution;
        
        topoMapMsg.info.origin.orientation.x = 0.0;
        topoMapMsg.info.origin.orientation.y = 0.0;
        topoMapMsg.info.origin.orientation.z = 0.0;
        topoMapMsg.info.origin.orientation.w = 1.0;

        topoMapMsg.info.origin.position.x = -(mapSize*resolution)/2;
        topoMapMsg.info.origin.position.y = -(mapSize*resolution)/2;
        topoMapMsg.info.origin.position.z = 10; //add 10 for visualization

        topoMapMsg.data.resize(topoMapMsg.info.width * topoMapMsg.info.height);
        std::fill(topoMapMsg.data.begin(), topoMapMsg.data.end(), -1);
    }

    void updateMap(const nav_msgs::OccupancyGrid& mapMsg){
        int width=mapMsg.info.width;
        int height=mapMsg.info.height;
        //ROS_INFO("%i",height);
        //int mapResolution=mapMsg.occupancy.info.resolution;
        for(int x=0; x<width;x++){
            for(int y=0; y<height;y++){
                int index= x+y*width;
                Map[x][y]=mapMsg.data[index];
            }
        }
    }

    void correctOpening(opening *op){
        point_int *p1;
        point_int *p2;
        for(int k=0; k<2; k++){
            if(k==0){
                p1=&op->start;
                p2=&op->end;
            }else{
                p1=&op->end;
                p2=&op->start;
            }
            int tIndex=0;

            while (Map[p1->x][p1->y]!=0){
                if(abs(p2->x-p1->x)>abs(p2->y-p1->y)&&abs(p2->x-p1->x)>2){
                    p1->x+=(p2->x-p1->x)<0?-1:1;
                }
                else if(abs(p2->y-p1->y)>2){
                    p1->y+=(p2->y-p1->y)<0?-1:1;
                }else{
                    break;
                }
            }
            int dirX=0;
            int dirY=1;
            int lenght=0;
            while(Map[p1->x+dirX*(lenght+1)][p1->y+dirY*(lenght+1)]==0){
                if(dirX==0 && dirY==1){
                    dirX=1;
                    dirY=0;
                }else if(dirX==1 && dirY==0){
                    dirX=0;
                    dirY=-1;
                }else if(dirX==0 && dirY==-1){
                    dirX=-1;
                    dirY=0;
                }else if(dirX==-1 && dirY==0){
                    dirX=0;
                    dirY=1;
                    lenght+=1;
                }
            }
            p1->x+=dirX*lenght;
            p1->y+=dirY*lenght;
            
        }
    }
    void correctAllOpenings(){
        for(int i=0; i<oplist.lenght; i++){
            for(int j=0; j<oplist.list[i].size(); j++){
                correctOpening(&oplist.list[i][j]);
            }
        }
    }

    bool cleanOpenings(opening op){
        vector<point_int> vo;
        vector<int> vint;
        bool selfDel=true;
        //ROS_INFO("----------------");
        for(int sids=0; sids<2; sids++){
            for(int cw=0; cw<2; cw++){
                ant_data step;
                step.dir={0,0};
                int empty_count=0;
                if(sids==0){
                    step.end=op.start;
                }else{
                    step.end=op.end;
                }
                //ROS_INFO("---");
                for(int s=0; s<searchLenghtClean && empty_count<maxAntGap; s++){
                    //ROS_INFO("%i, %i",step.end.x,step.end.y);
                    //scanMapOutHolder[step.end.x][step.end.y]=100;
                    
                    if(step.emty_cell){
                        empty_count+=1;
                    }
                    if(sids==0 && step.end==op.end){
                        return false;
                    }

                    for(int i=0; i<oplist.lenght; i++){
                        for(int j=0; j<oplist.list[i].size(); j++){
                            if(sids==0 && oplist.list[i][j].start==step.end && oplist.list[i][j].label!=-1){
                                //ROS_INFO("start: %i, %i, end: %i, %i, step: %i, %i",oplist.list[i][j].start.x,oplist.list[i][j].start.y,oplist.list[i][j].end.x,oplist.list[i][j].end.y,step.end.x,step.end.y);
                                vo.push_back({i,j});
                                int sepL=cw==1?s:-s;
                                vint.push_back(sepL);
                            }
                            if(sids==1 && oplist.list[i][j].end==step.end && oplist.list[i][j].label!=-1){
                                point_int p={i,j};
                                for(int index=0; index<vo.size(); index++){
                                    if(vo[index]==p){
                                        //ROS_INFO("start: %i, %i, end: %i, %i, step: %i, %i",oplist.list[i][j].start.x,oplist.list[i][j].start.y,oplist.list[i][j].end.x,oplist.list[i][j].end.y,step.end.x,step.end.y);
                                        vint[index]+=cw==0?s:-s;
                                        if(dist(op.start,op.end)+vint[index]/extendDevider<dist(oplist.list[i][j].start,oplist.list[i][j].end)){
                                            oplist.list[i][j].label=-1;
                                        }else{
                                            selfDel = false;
                                        }
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    step=ant_step(step.end,(bool)cw, step.dir);
                }
            }
        }

        return selfDel;
    }

    void topologyScan(){
        //oplist.list[currentRotationIndex].clear();
        //correctAllOpenings();
        for (int i = 0; i < scanSize; ++i){
            scanGroupIndex[i]=0;
        }
        float rotation=M_PI*currentRotationIndex/numberOfDir;
        //Copy the map to scanMap
        rotateMap(rotation,mapSize,scanMap,Map);
        
        //Clear scanMapOutput from previus scan
        for (int i = 0; i < scanSize; ++i){
            scanGroupIndex[i]=0;
            for (int j = 0; j < scanSize; ++j){
                scanMapOut[i][j] = 0;
            }
        }
        
        //Loop thur all map cells
        for(int i=0;i<scanSize;i++){
            for(int j=1;j<scanSize-1;j++){
                //finde groups
                if(scanMap[i][j]==0){
                    //check if space is free
                    if(cGroupeSize==0){
                        scanGarray[i][scanGroupIndex[i]].start=j;
                    }
                    cGroupeSize+=1;
                    cfilter=0;
                //filter out smal point opstacals
                }else if (scanMap[i][j]==-1 && cfilter<cfilterSize){
                    cfilter+=1;
                //if findeing ostacals biger then filter end serche
                }else{
                    //if found groupe is larger then minGroupSize add it as group
                    if(cGroupeSize>minGroupSize){
                        scanGarray[i][scanGroupIndex[i]].end=j-1-cfilter;
                        scanGarray[i][scanGroupIndex[i]].prevGroupIndex=0;
                        scanGarray[i][scanGroupIndex[i]].prevGroup[0]=NULL;
                        scanGarray[i][scanGroupIndex[i]].nextGroupIndex=0;
                        scanGarray[i][scanGroupIndex[i]].nextGroup[0]=NULL;
                        scanGroupIndex[i]+=1;
                    }
                    cfilter=0;
                    cGroupeSize=0;
                }
            }
            //find conection_lenghts between groups
            //check if ther are les grupe then previus line
            for(int index1=0;index1<scanGroupIndex[i];index1++){
                //find if there is tow or more gropse conecteing to a previus group
                for(int index2=0;index2<scanGroupIndex[i-1];index2++){
                    if(scanGarray[i][index1].start<scanGarray[i-1][index2].end &&
                        scanGarray[i][index1].end>scanGarray[i-1][index2].start){
                            scanGarray[i][index1].prevGroup[scanGarray[i][index1].prevGroupIndex]=&scanGarray[i-1][index2];
                            scanGarray[i][index1].prevGroupIndex+=1;

                            scanGarray[i-1][index2].nextGroup[scanGarray[i-1][index2].nextGroupIndex]=&scanGarray[i][index1];
                            scanGarray[i-1][index2].nextGroupIndex+=1;
                        }
                }
            }
        }
        for(int i=1;i<scanSize;i++){
            for(int index= 0; index<scanGroupIndex[i];index++){
                //FLYTTA TILL FUNKTION
                for(int direction=0;direction<2;direction++){
                    if(scanGarray[i][index].prevGroupIndex>1&&direction==0||scanGarray[i][index].nextGroupIndex>1&&direction==1){
                        scanGroup *p=&scanGarray[i][index];
                        int depthCount=0;
                        scanGroup *entresnsCenter=p;
                        while (p->nextGroupIndex==1&&direction==0||p->prevGroupIndex==1&&direction==1){
                            depthCount+=1;
                            if(depthCount>=minCoridorSize){
                                break;
                            }
                            if(direction==0){
                                p=p->nextGroup[0];
                            }else{
                                p=p->prevGroup[0];
                            }
                        }

                        if(depthCount>=minCoridorSize){
                            int countRealRoads=0;
                            scanGroup *firstCoridorCenter;
                            scanGroup *scondCoridorCenter;
                            int loopAmount=scanGarray[i][index].prevGroupIndex;
                            if(direction==1){
                                loopAmount=scanGarray[i][index].nextGroupIndex;
                            }
                            bool first=true;
                            for(int h=0;h<loopAmount;h++){
                                depthCount=0;
                                if(direction==0){
                                    p=scanGarray[i][index].prevGroup[h];
                                }else{
                                    p=scanGarray[i][index].nextGroup[h];
                                }
                                if(first){
                                    firstCoridorCenter=p;
                                }else{
                                    scondCoridorCenter=p;
                                }
                                while (p!=NULL){

                                    depthCount+=1;
                                    if(depthCount==minCoridorSize){
                                        countRealRoads+=1;
                                        break;
                                    }

                                    if(direction==0){
                                        p=p->prevGroup[0];
                                    }else{
                                        p=p->nextGroup[0];
                                    }
                                }
                                if(countRealRoads==1){
                                    first=false;
                                }
                            }
                            if(countRealRoads>=2){
                                point_int p[4];
                                p[0].x=-1;
                                p[0].y=-1;
                                p[1].x=i;
                                p[1].y=firstCoridorCenter->end;
                                int inSearchLenght=5;
                                double minDist=-1;
                                scanGroup *coridor;
                                coridor=firstCoridorCenter;
                                for(int m=0; m<searchLenght && coridor!=NULL; m++){
                                    double d=dist({coridor->start,i+m},{firstCoridorCenter->end,i});
                                    if(scanMap[i-m*(1-2*direction)][coridor->start-1]==100 && (minDist<0 || d<minDist)){
                                        minDist=d;
                                        p[0].x=i-m*(1-2*direction);
                                        p[0].y=coridor->start;
                                    }
                                    if(direction==0){
                                        coridor=coridor->prevGroup[0];
                                    }else{
                                        coridor=coridor->nextGroup[0];
                                    }
                                }
                                coridor=entresnsCenter;
                                for(int m=0; m<searchLenght && coridor!=NULL; m++){
                                    double d=dist({coridor->start,i+m},{firstCoridorCenter->end,i});
                                    if(scanMap[i+m*(1-2*direction)][coridor->start-1]==100 && (minDist<0 || d<minDist)){
                                        minDist=d;
                                        p[0].x=i+m*(1-2*direction);
                                        p[0].y=coridor->start;
                                    }
                                    if(direction==1){
                                        coridor=coridor->prevGroup[0];
                                    }else{
                                        coridor=coridor->nextGroup[0];
                                    }
                                }
                                minDist=-1;
                                coridor=firstCoridorCenter;
                                for(int m=0; m<searchLenght && coridor!=NULL; m++){
                                    double d=dist({coridor->end,i+m},p[0]);
                                    if(scanMap[i-m*(1-2*direction)][coridor->end+1]==100 && (minDist<0 || d<minDist)){
                                        minDist=d;
                                        p[1].x=i-m*(1-2*direction);
                                        p[1].y=coridor->end;
                                    }
                                    if(direction==0){
                                        coridor=coridor->prevGroup[0];
                                    }else{
                                        coridor=coridor->nextGroup[0];
                                    }
                                }
                                
                                p[3].x=-1;
                                p[3].y=-1;
                                p[2].x=i;
                                p[2].y=scondCoridorCenter->start;

                                minDist=-1;
                                coridor=scondCoridorCenter;
                                for(int m=0; m<searchLenght && coridor!=NULL; m++){
                                    double d=dist({coridor->end,i+m},{scondCoridorCenter->start,i});
                                    if(scanMap[i-m*(1-2*direction)][coridor->end+1]==100 && (minDist<0 || d<minDist)){
                                        minDist=d;
                                        p[3].x=i-m*(1-2*direction);
                                        p[3].y=coridor->end;
                                    }
                                    if(direction==0){
                                        coridor=coridor->prevGroup[0];
                                    }else{
                                        coridor=coridor->nextGroup[0];
                                    }
                                }
                                coridor=entresnsCenter;
                                for(int m=0; m<searchLenght && coridor!=NULL; m++){
                                    double d=dist({coridor->end,i+m},{scondCoridorCenter->start,i});
                                    if(scanMap[i+m*(1-2*direction)][coridor->end+1]==100 && (minDist<0 || d<minDist)){
                                        minDist=d;
                                        p[3].x=i+m*(1-2*direction);
                                        p[3].y=coridor->end;
                                    }
                                    if(direction==1){
                                        coridor=coridor->prevGroup[0];
                                    }else{
                                        coridor=coridor->nextGroup[0];
                                    }
                                }
                                minDist=-1;
                                coridor=scondCoridorCenter;
                                for(int m=0; m<searchLenght && coridor!=NULL; m++){
                                    double d=dist({coridor->start,i+m},p[3]);
                                    if(scanMap[i-m*(1-2*direction)][coridor->start-1]==100 && (minDist<0 || d<minDist)){
                                        minDist=d;
                                        p[2].x=i-m*(1-2*direction);
                                        p[2].y=coridor->start;
                                    }
                                    if(direction==0){
                                        coridor=coridor->prevGroup[0];
                                    }else{
                                        coridor=coridor->nextGroup[0];
                                    }
                                }
                                if(p[0].y==-1 || p[1].y==-1 || p[2].y==-1 || p[3].y==-1){
                                    continue;
                                }

                                rotate_points(p,4,rotation,mapSize);

                                //ROS_INFO("%i,%i,%i,%i,%i,%i,%i,%i",p[0].x,p[0].y,p[1].x,p[1].y,p[2].x,p[2].y,p[3].x,p[3].y);
                                

                                for(int k=0; k<4;k+=2){
                                    opening o;
                                    if(direction==0){
                                        o.start=p[k+1];
                                        o.end=p[k];
                                    }else{
                                        o.start=p[k];
                                        o.end=p[k+1];
                                    }
                                    if(dist(o.start,o.end)<minGroupSize){
                                        continue;
                                    }
                                    correctOpening(&o);
                                    for(int sids=0; sids<2;sids++){
                                        ant_data step;
                                        step.end=sids==0?o.start:o.end;
                                        bool cheek=false;
                                        int c=0;
                                        while (!cheek && c<inSearchLenght){
                                            c++;
                                            cheek=true;
                                            for(int n=0; n<oplist.size(); n++){
                                                if(step.end==oplist.get(n).end || step.end==oplist.get(n).start){
                                                    step=ant_step(step.end,sids==0,step.dir);
                                                    if(sids==0){
                                                        o.start=step.end;
                                                    }else{
                                                        o.end=step.end;
                                                    }
                                                    cheek=false;
                                                    break;
                                                }
                                            }
                                        }
                                        if(c==inSearchLenght){
                                            //continue;
                                        }
                                    }
                                    
                                    bool skip=false;
                                    for(int l=0; l<oplist.lenght; l++){
                                        for(int h=0; h<oplist.list[l].size(); h++){
                                            opening oplistComp=oplist.list[l][h];
                                            if(intersect_line(o,oplistComp) && true){
                                                bool conected[]={false, false};
                                                point_int conect[2];
                                                int conection_lenght[]={0,0};//direction and distans to conection_lenght 0=no conection_lenght
                                                int conection_waite=-1;
                                                point_int conection_dir[2];
                                                bool conection_cw[2];
                                                ant_data step_info;
                                                
                                                for(int sides=0; sides<2; sides++){
                                                    for(int d=0; d<2; d++){
                                                        if(!conected[sides]){
                                                            step_info.end=sides==0?o.start:o.end;
                                                            bool clockwise= d==0;
                                                            step_info.dir={0,0};
                                                            int emty_count=0;
                                                            for(int s=0;s<sercheLenthAnt; s++){
                                                                if(step_info.end==oplistComp.end){
                                                                        if(sides==1){
                                                                            conection_waite=direction==1?s:-s;
                                                                        }

                                                                        conect[sides]=oplistComp.end;
                                                                        conected[sides]=true;
                                                                        conection_lenght[sides]=s;
                                                                        conection_dir[sides].x=step_info.dir.x;
                                                                        conection_dir[sides].y=step_info.dir.y;
                                                                        conection_cw[sides]=clockwise;
                                                                        break;

                                                                }else if(step_info.end==oplistComp.start){
                                                                        if(sides==0){
                                                                            conection_waite+=direction==0?s:-s;
                                                                        }
                                                                        conect[sides]=oplistComp.start;
                                                                        conected[sides]=true;
                                                                        conection_lenght[sides]=s;
                                                                        conection_dir[sides].x=step_info.dir.x;
                                                                        conection_dir[sides].y=step_info.dir.y;
                                                                        conection_cw[sides]=clockwise;
                                                                        break;
                                                                }

                                                                step_info=ant_step(step_info.end,clockwise,step_info.dir);
                                                                
                                                                if(step_info.emty_cell){
                                                                    emty_count+=1;
                                                                    if(emty_count>maxAntGap){
                                                                        break;
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                                
                                                int sides=1;
                                                if(!conected[0] && !conected[1]){
                                                    skip=true;
                                                    break;
                                                    
                                                }else if(conected[0] && conected[1] && conection_waite!=-1){
                                                    if(dist(o.start,o.end)+conection_waite/extendDevider<dist(oplistComp.start,oplistComp.end)){
                                                        oplist.list[l].erase(oplist.list[l].begin()+h);
                                                        h-=1;
                                                    }else{
                                                        skip=true;
                                                        break;
                                                    }

                                                }else if(conection_lenght[0]<conection_lenght[1] && conected[0] || !conected[1]){
                                                    sides=0;
                                                }
                                                
                                                step_info=ant_step(conect[sides],conection_cw[sides],conection_dir[sides]);
                                                if(sides==0){
                                                    o.start.x=step_info.end.x;
                                                    o.start.y=step_info.end.y;
                                                }else{
                                                    o.end.x=step_info.end.x;
                                                    o.end.y=step_info.end.y;
                                                }
                                                
                                                point_int *op;
                                                op=&step_info.end;
                                                int cIndex=-1;
                                                bool cheek=false;
                                                int deb=0;
                                                while (!cheek){
                                                    
                                                    /*for(int n=0; n<opening_list.size(); n++){
                                                        if(n!=cIndex){
                                                            if(*op==opening_list[n].start){
                                                                op=&opening_list[n].start;
                                                                step_info=ant_step(step_info.end,conection_cw[sides],step_info.dir);
                                                                op->x=step_info.end.x;
                                                                op->y=step_info.end.y;
                                                                cIndex=n;
                                                                deb++;
                                                                break;
                                                            }else if(*op==opening_list[n].end){
                                                                op=&opening_list[n].end;
                                                                step_info=ant_step(step_info.end,conection_cw[sides],step_info.dir);
                                                                op->x=step_info.end.x;
                                                                op->y=step_info.end.y;
                                                                cIndex=n;
                                                                deb++;
                                                                break;
                                                            }
                                                            
                                                        }
                                                        if(n>=opening_list.size()-1){
                                                            cheek=true;
                                                        }*/
                                                        cheek=true;
                                                        for(int n=0; n<oplist.size(); n++){
                                                            if(step_info.end==oplist.get(n).end || step_info.end==oplist.get(n).start){
                                                                step_info=ant_step(step_info.end,conection_cw[sides],step_info.dir);
                                                                if(sides==0){
                                                                    o.start.x=step_info.end.x;
                                                                    o.start.y=step_info.end.y;
                                                                }else{
                                                                    o.end.x=step_info.end.x;
                                                                    o.end.y=step_info.end.y;
                                                                }
                                                                cheek=false;
                                                                //ROS_INFO("%i, %i", step_info.end.x,step_info.end.y);
                                                            }
                                                        }
                                                        
                                                    }
                                                }
                                        }
                                    }
                                    if(direction==0){
                                        p[k+1]=o.start;
                                        p[k]=o.end;
                                    }else{
                                        p[k]=o.start;
                                        p[k+1]=o.end;
                                    }

                                    if(skip)continue;
                                    if(dist(o.start,o.end)<minGroupSize){
                                        continue;
                                    }
                                    if(!cleanOpenings(o)) continue;
                                    o.conected=false;
                                    oplist.list[currentRotationIndex].push_back(o);

                                }
                                for(int k=0; k<4; k++){
                                    //scanMapOutHolder[p[k].x][p[k].y]=100;
                                }
                                    
                            }
                        }
                    }
                }
            }
        }
            
            
            
        //rotateMap(-rotation,mapSize,scanMapOutTransform,scanMapOut);
        for(int x=0; x<mapSize;x++){
            for(int y=0;y<mapSize;y++){
                //scanMapOutHolder[x][y]+=scanMapOutTransform[x][y];
            }
        }
            
            
        
        for(int x=0; x<mapSize;x++){
            for(int y=0;y<mapSize;y++){
                topMap[x][y]=scanMapOutHolder[x][y]>=100?100:-1;
            }
        }

        currentRotationIndex+=1;
        if(currentRotationIndex==numberOfDir){
            currentRotationIndex=0;
            for (int i = 0; i < scanSize; ++i){
                for (int j = 0; j < scanSize; ++j){
                    scanMapOutHolder[i][j] = 0;
                }
            }
        }

    }

    void creatPoligonList(){
        poly_list.clear();
        int label=1;
        /*for(int i=0; i<opening_list.size(); i++){
            ROS_INFO("%i, start: %i, %i, end: %i, %i",i,opening_list[i].start.x, opening_list[i].start.y,opening_list[i].end.x, opening_list[i].end.y);
        }*/

        for(int l=0; l<oplist.lenght;l++){
            for(int i=0; i<oplist.list[l].size(); i++){
                oplist.list[l][i].conected=false;
            }
        }

        for(int l=0; l<oplist.lenght;l++){
            for(int i=0; i<oplist.list[l].size(); i++){
                if(oplist.list[l][i].conected==false){
                    bool cw=true;
                    poligon poly;
                    poly.sides.push_back(oplist.list[l][i]);
                    oplist.list[l][i].conected=true;
                    ant_data step_info;      
                    bool cheek=false, complet=false, nextOpening=false;
                    point_int firstIndex={l,i};
                    point_int targetIndex={l,i};
                    point_int lastIndex={l,-1};

                    int wait=-1;

                    ROS_INFO("---");
                    while(!cheek){
                        //ROS_INFO("%i",targetIndex);
                        int empty_cell_count=0;
                        if(cw){
                            step_info.end=oplist.list[targetIndex.x][targetIndex.y].start; 
                        }else{
                            step_info.end=oplist.list[targetIndex.x][targetIndex.y].end;
                        }
                        
                        step_info.dir={0,0};
                        nextOpening=false;
                        for(int s=0; s<=sercheLenthAntConect; s++){
                            for(int ll=0; ll<oplist.lenght; ll++){
                                for(int h=0; h<oplist.list[ll].size();h++){
                                    point_int tp={ll,h};
                                    if(!(tp==targetIndex)){
                                        if(step_info.end==oplist.list[ll][h].start && cw ||
                                            step_info.end==oplist.list[ll][h].end && !cw ||
                                            empty_cell_count>maxAntGap || s==sercheLenthAntConect){
                                                ROS_INFO(" tp: %i, %i, target: %i, %i",tp.x, tp.y,targetIndex.x, targetIndex.y);
                                                if(cw){
                                                    wait=s;

                                                    ROS_INFO("found no conecttion sershing other dir");
                                                    lastIndex=targetIndex;
                                                    targetIndex=firstIndex;
                                                    cw=false;
                                                    nextOpening=true;
                                                    break;
                                                }else{
                                                    if(poly.sides.size()>1){
                                                        opening newOp;
                                                        newOp.start=oplist.list[targetIndex.x][targetIndex.y].end;
                                                        newOp.end=oplist.list[lastIndex.x][lastIndex.y].start;
                                                        newOp.label=2;
                                                        newOp.conected=true;
                                                        oplist.list[targetIndex.x].push_back(newOp);
                                                        poly.sides.push_back(newOp);
                                                        complet=true;
                                                    }
                                                    ROS_INFO("found no conection don serching");
                                                    nextOpening=true;
                                                    cheek=true;
                                                    break;
                                                }
                                        }
                                        else if(step_info.end==oplist.list[ll][h].end && cw ||
                                            step_info.end==oplist.list[ll][h].start && !cw){
                                                if(oplist.list[ll][h].conected==false){
                                                    ROS_INFO("found conection");
                                                    poly.sides.push_back(oplist.list[ll][h]);
                                                    oplist.list[ll][h].conected=true;
                                                    targetIndex={ll,h};
                                                    nextOpening=true;
                                                    break;
                                                }else{
                                                    ROS_INFO("found complet");
                                                    
                                                    cheek=true;
                                                    complet=true;
                                                    break;
                                                }
                                        }  
                                    }
                                }
                                if(cheek || nextOpening){
                                    break;
                                }
                            }
                            if(cheek || nextOpening){
                                ROS_INFO("s: %i",s);
                                break;
                            }
                            //ROS_INFO("%i, %i, %i, %i",step_info.end.x,step_info.end.y,targetIndex,label);
                            step_info=ant_step(step_info.end,cw,step_info.dir);
                            //topMap[step_info.end.x][step_info.end.y]=100;

                            if(step_info.emty_cell){
                                empty_cell_count+=1;
                            }  
                        }
                        if(complet){
                            for(int j=0;j< poly.sides.size();j++){
                                ROS_INFO("%i, start: %i, %i, end: %i, %i",i,poly.sides[j].start.x, poly.sides[j].start.y,poly.sides[j].end.x, poly.sides[j].end.y);
                            }
                            poly_list.add(poly,label);
                            label++;
                        }
                    }
                }
            }
        }
        for(int i=0; i<poly_list.size();i++){
            if(poly_list.poly[i].sides.size()<3){
                poly_list.remove(i);
                i--;
            }
        }
    }

    void pubMap(){
        
        jsk_recognition_msgs::PolygonArray pubPolyArray_old;
        pubPolyArray_old.header.frame_id = "map";
        pubPolyArray_old.header.stamp = ros::Time::now();
        pubPolyArray_old.polygons.resize(oplist.size());
        pubPolyArray_old.labels.resize(oplist.size());
        pubPolyArray_old.likelihood.resize(oplist.size());
        int halfMap=mapSize/2;
        for(int i=0; i<oplist.size(); i++){
            
            opening op=oplist.get(i);
            point nNorm={-(double)(op.end.y-op.start.y),
                            (double)(op.end.x-op.start.x)};
            double l=sqrt(abs(nNorm.x*nNorm.x+nNorm.y*nNorm.y));
            nNorm.x=nNorm.x/l*resolution;
            nNorm.y=nNorm.y/l*resolution;
            geometry_msgs::PolygonStamped p;
            p.header.frame_id = "map";
            p.header.stamp = ros::Time::now();
            p.polygon.points.resize(4);
            topMap[op.start.x][op.start.y]=100;
            topMap[op.end.x][op.end.y]=100;
            p.polygon.points[0].x=(op.start.x-halfMap)*resolution;
            p.polygon.points[0].y=(op.start.y-halfMap)*resolution;
            p.polygon.points[0].z=10.1;

            p.polygon.points[1].x=(op.end.x-halfMap)*resolution;
            p.polygon.points[1].y=(op.end.y-halfMap)*resolution;
            p.polygon.points[1].z=10.1;
            
            p.polygon.points[2].x=(op.end.x-halfMap)*resolution+nNorm.x;
            p.polygon.points[2].y=(op.end.y-halfMap)*resolution+nNorm.y;
            p.polygon.points[2].z=10.1;

            p.polygon.points[3].x=(op.start.x-halfMap)*resolution+nNorm.x;
            p.polygon.points[3].y=(op.start.y-halfMap)*resolution+nNorm.y;
            p.polygon.points[3].z=10.1;
            
            pubPolyArray_old.polygons[i]=p;
            pubPolyArray_old.labels[i]=op.label;
            pubPolyArray_old.likelihood[i]=1;
        }
        pubTopoPoly_debug.publish(pubPolyArray_old);
        
        jsk_recognition_msgs::PolygonArray pubPolyArray;
        pubPolyArray.header.frame_id = "map";
        pubPolyArray.header.stamp = ros::Time::now();
        pubPolyArray.polygons.resize(poly_list.size());
        pubPolyArray.labels.resize(poly_list.size());
        pubPolyArray.likelihood.resize(poly_list.size());
        for(int i=0; i<poly_list.size(); i++){
            pubPolyArray.polygons[i]=poly_list.get_polygon(i,mapSize,resolution);
            pubPolyArray.labels[i]=poly_list.label[i];
            pubPolyArray.likelihood[i]=1;
        }
        pubTopoPoly.publish(pubPolyArray);

        topoMapMsg.header.stamp = ros::Time::now();
        for(int y=0; y<mapSize;y++){
            for(int x=0;x<mapSize;x++){  
                int index=x+y*mapSize;            
                topoMapMsg.data[index]=topMap[x][y];
            }
        }
        pubTopoMap.publish(topoMapMsg);

    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "topology_mapping");
    
    TopologyMapping topMapping;

    ROS_INFO("Topology Mapping Started.");
    
    topMapping.spin();
    
    return 0;
}