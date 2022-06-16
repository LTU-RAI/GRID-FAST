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
    bool complet_start;
    bool complet_end;
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

        point_int center={0,0};
        for(int i=0; i<points.size(); i++){
            center.x+=points[i].x;
            center.y+=points[i].y;
        }
        center.x=center.x/points.size();
        center.x=center.y/points.size();

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
        int numberOfDir=10;
        int maxGapDistans=10;
        int extendDevider=2;
        int searchLenght=8;

        //global var
        int **scanMap;
        int **scanMapOut;
        int **scanMapOutTransform;
        int **scanMapOutHolder;
        scanGroup **scanGarray;
        vector<opening> opening_list;
        Poligon_list poly_list;
        int *scanGroupIndex;

        //ant para
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
        ros::Rate rate(10); // Hz
        
        while (ros::ok()){

            topologyScan();
            //creatPoligonList();
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
    
    ant_data ant_step(point_int start, bool clockwise, point_int direction, int** map){
        ant_data step;
        int newDirx=clockwise?-direction.y:direction.y;
        int newDiry=clockwise?direction.x:-direction.x;
        step.dir.x=newDirx;
        step.dir.y=newDiry;
        step.end=start;

        if(step.dir.x==0 && step.dir.y==0){
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
            }
            
        }

        for(int d=0;d<3;d++){
            if(map[start.x+step.dir.x][start.y+step.dir.y]==0){
                step.end.x=start.x+step.dir.x;
                step.end.y=start.y+step.dir.y;
                step.emty_cell=false;
                point_int dir=step.dir;
                for(int d=0;d<4;d++){
                    if(map[step.end.x+step.dir.x][step.end.y+step.dir.y]==-1){
                        step.emty_cell=true;
                        break;
                    }
                    newDirx=dir.y;
                    newDiry=-dir.x;
                    step.dir.x=newDirx;
                    step.dir.y=newDiry;
                }

                return step;
            }
            newDirx=clockwise?step.dir.y:-step.dir.y;
            newDiry=clockwise?-step.dir.x:step.dir.x;
            step.dir.x=newDirx;
            step.dir.y=newDiry;
        }
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
            cw=sides==0?up:!up;
            int prevX=startX;
            minimumDistans=-1;
            search_step.end=*p1;
            search_step.dir={0,0};
            for(int m=0;m<inSearchLenght;m++){
                search_step=ant_step(search_step.end,cw,search_step.dir,scanMap);
                if(search_step.end.x>prevX && sides==0 || search_step.end.x<prevX && sides==1){
                    break;
                }
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
            search_step=ant_step(search_step.end,cw,search_step.dir,scanMap);
            if(search_step.end.x>prevX && sides==0 || search_step.end.x<prevX && sides==1){
                break;
            }
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
            search_step=ant_step(search_step.end,cw,search_step.dir,scanMap);
            if(search_step.end.x>prevX && direction || search_step.end.x<prevX && !direction){
                break;
            }
            prevX=search_step.end.x;

            if(!search_step.emty_cell){
                double d=dist(search_step.end,*p1);
                if(minimumDistans<0 || d<minimumDistans){
                    minimumDistans=d;
                    *p2=search_step.end;
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


    void topologyScan(){
        opening_list.clear();

        for (int i = 0; i < scanSize; ++i){
            scanGroupIndex[i]=0;
            for (int j = 0; j < scanSize; ++j){
                scanMapOutHolder[i][j] = 0;
            }
        }
        
        for(int turn=0;turn<numberOfDir;turn++){
            float rotation=M_PI*turn/numberOfDir;
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
                for(int j=1;j<scanSize;j++){
                    //finde groups
                    //if(scanMap[i][j]==0 && cGroupeSize!=0 ||scanMap[i][j]==0 && scanMap[i][j-1]==100){
                    if(scanMap[i][j]==0){
                        //check if space is free
                        if(cGroupeSize==0){
                            scanGarray[i][scanGroupIndex[i]].start=j;
                            
                            if(scanMap[i][j-1]==-1){
                                scanGarray[i][scanGroupIndex[i]].complet_start=false;
                            }else{
                                scanGarray[i][scanGroupIndex[i]].complet_start=true;
                            }
                        }
                        cGroupeSize+=1;
                        cfilter=0;
                    //filter out smal point opstacals
                    }else if (scanMap[i][j]==-1 && cfilter<cfilterSize){
                        cfilter+=1;
                    //if findeing ostacals biger then filter end serche
                    }else{
                        //if found groupe is larger then minGroupSize add it as group
                        //if(cGroupeSize>minGroupSize && scanMap[i][j]==100){
                        if(cGroupeSize>minGroupSize){
                            scanGarray[i][scanGroupIndex[i]].end=j-1-cfilter;
                            scanGarray[i][scanGroupIndex[i]].prevGroupIndex=0;
                            scanGarray[i][scanGroupIndex[i]].prevGroup[0]=NULL;
                            scanGarray[i][scanGroupIndex[i]].nextGroupIndex=0;
                            scanGarray[i][scanGroupIndex[i]].nextGroup[0]=NULL;
                            scanGroupIndex[i]+=1;

                            if(scanMap[i][j]==-1){
                                scanGarray[i][scanGroupIndex[i]].complet_end=false;
                            }else{
                                scanGarray[i][scanGroupIndex[i]].complet_end=true;
                            }
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
                            int entresnsCenter[3][searchLenght];
                            int entresnsCenterSize=0;
                            int x=i;
                            while (p->nextGroupIndex==1&&direction==0||p->prevGroupIndex==1&&direction==1){
                                if(depthCount<searchLenght){
                                    entresnsCenter[0][depthCount]= p->start;
                                    entresnsCenter[1][depthCount]= p->end;
                                    entresnsCenterSize=depthCount+1;
                                }
                                x+=1-2*direction;
                                depthCount+=1;
                                if(depthCount>=minCoridorSize && depthCount>=searchLenght){
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
                                int firstCoridorCenter[4][searchLenght];
                                int firstCoridorCenterSize=0;
                                int scondCoridorCenter[4][searchLenght];
                                int scondCoridorCenterSize=0;
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
                                    x=i;
                                    while (p!=NULL){
                                        x+=-1+2*direction;
                                        if(first && depthCount<searchLenght){
                                            if(!p->complet_end && depthCount==0){
                                                //break;
                                            }
                                            firstCoridorCenter[0][depthCount]= p->start;
                                            firstCoridorCenter[1][depthCount]= p->end;
                                            firstCoridorCenter[2][depthCount]= p->start+(p->end-p->start)/2;
                                            firstCoridorCenter[3][depthCount]= x;
                                            firstCoridorCenterSize=depthCount+1;
                                        }else if(depthCount<searchLenght){
                                            if(!p->complet_start && depthCount==0){
                                                //break;
                                            }
                                            scondCoridorCenter[0][depthCount]= p->start;
                                            scondCoridorCenter[1][depthCount]= p->end;
                                            scondCoridorCenter[2][depthCount]= p->start+(p->end-p->start)/2;
                                            scondCoridorCenter[3][depthCount]= x;
                                            scondCoridorCenterSize=depthCount+1;
                                        }

                                        depthCount+=1;
                                        if(depthCount==minCoridorSize){
                                            countRealRoads+=1;
                                        }
                                        if(depthCount>=minCoridorSize && depthCount>=searchLenght){
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
                                    p[0].x=i;
                                    p[0].y=firstCoridorCenter[0][0];
                                    p[1].x=i;
                                    p[1].y=firstCoridorCenter[1][0];
                                    int inSearchLenght=5;

                                    if(!fitToCoridor(&p[0],&p[1],inSearchLenght,i,direction==1,false)){
                                        continue;
                                    }
                                    
                                    p[2].x=i;
                                    p[2].y=scondCoridorCenter[0][0];
                                    p[3].x=i;
                                    p[3].y=scondCoridorCenter[1][0];

                                    if(!fitToCoridor(&p[3],&p[2],inSearchLenght,i,direction==1,true)){
                                        continue;
                                    }

                        
                                    rotate_points(p,4,rotation,mapSize);

                                    for(int k=0; k<4; k++){
                                        int tIndex=0;
                                        switch (k)
                                        {
                                        case 0:
                                            tIndex=1;
                                            break;
                                        case 2:
                                            tIndex=3;
                                            break;
                                        case 3:
                                            tIndex=2;
                                            break;
                                        }
                                        while (Map[p[k].x][p[k].y]!=0){
                                            if(abs(p[tIndex].x-p[k].x)>abs(p[tIndex].y-p[k].y)&&abs(p[tIndex].x-p[k].x)>2){
                                                p[k].x+=(p[tIndex].x-p[k].x)<0?-1:1;
                                            }
                                            else if(abs(p[tIndex].y-p[k].y)>2){
                                                p[k].y+=(p[tIndex].y-p[k].y)<0?-1:1;
                                            }else{
                                                break;
                                            }
                                        }
                                        int dirX=0;
                                        int dirY=1;
                                        int lenght=0;
                                        while(Map[p[k].x+dirX*(lenght+1)][p[k].y+dirY*(lenght+1)]==0){
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
                                        p[k].x+=dirX*lenght;
                                        p[k].y+=dirY*lenght;
                                        
                                    }

                                    for(int k=0; k<4;k+=2){
                                        opening o;
                                        if(direction==0){
                                            o.start=p[k+1];
                                            o.end=p[k];
                                        }else{
                                            o.start=p[k];
                                            o.end=p[k+1];
                                        }
                                        
                                        ant_data step;
                                        step.end=o.start;
                                        bool cheek=false;
                                        while (!cheek){
                                            cheek=true;
                                            for(int n=0; n<opening_list.size(); n++){
                                                
                                                if(step.end==opening_list[n].end || step.end==opening_list[n].start){
                                                    step=ant_step(step.end,false,step.dir,Map);
                                                        o.start.x=step.end.x;
                                                        o.start.y=step.end.y;
                                                    
                                                    cheek=false;
                                                }
                                            }
                                        }
                                        step.end=o.end;
                                        step.dir={0,0};
                                        cheek=false;
                                        while (!cheek){
                                            cheek=true;
                                            for(int n=0; n<opening_list.size(); n++){
                                                
                                                if(step.end==opening_list[n].end || step.end==opening_list[n].start){
                                                    step=ant_step(step.end,true,step.dir,Map);
                                                        o.end.x=step.end.x;
                                                        o.end.y=step.end.y;
                                                    
                                                    cheek=false;
                                                }
                                            }
                                        }
                                        
                                        
                                        bool skip=false;
                                        for(int h=0; h<opening_list.size(); h++){
                                            if(intersect_line(o,opening_list[h]) && false){
                                                point_int *conect[]={NULL, NULL};
                                                int conection_lenght[]={0,0};//direction and distans to conection_lenght 0=no conection_lenght
                                                point_int conection_dir[2];
                                                bool conection_cw[2];
                                                ant_data step_info;
                                                
                                                for(int sides=0; sides<2; sides++){
                                                    for(int d=0; d<2; d++){
                                                        if(conect[sides]==NULL){
                                                            step_info.end=sides==0?o.start:o.end;
                                                            bool clockwise= d==0;
                                                            step_info.dir={0,0};
                                                            int emty_count=0;
                                                            for(int s=0;s<sercheLenthAnt; s++){
                                                                if(step_info.end.x==opening_list[h].end.x && step_info.end.y==opening_list[h].end.y){
                                                                        conect[sides]=&opening_list[h].end;
                                                                        conection_lenght[sides]=s;
                                                                        conection_dir[sides].x=step_info.dir.x;
                                                                        conection_dir[sides].y=step_info.dir.y;
                                                                        conection_cw[sides]=clockwise;
                                                                        break;

                                                                }else if(step_info.end.x==opening_list[h].start.x && step_info.end.y==opening_list[h].start.y){
                                                                        conect[sides]=&opening_list[h].start;
                                                                        conection_lenght[sides]=s;
                                                                        conection_dir[sides].x=step_info.dir.x;
                                                                        conection_dir[sides].y=step_info.dir.y;
                                                                        conection_cw[sides]=clockwise;
                                                                        break;
                                                                }

                                                                step_info=ant_step(step_info.end,clockwise,step_info.dir,Map);
                                                                
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
                                                if(conect[0]==NULL && conect[1]==NULL){
                                                    skip=true;
                                                    break;
                                                    
                                                }else if(conection_lenght[0]<conection_lenght[1] && conect[0]!=NULL || conect[1]==NULL){
                                                    sides=0;
                                                }
                                                
                                                step_info=ant_step(*conect[sides],conection_cw[sides],conection_dir[sides],Map);
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
                                                        for(int n=0; n<opening_list.size(); n++){
                                                            cheek=true;
                                                            if(step_info.end==opening_list[n].end || step_info.end==opening_list[n].start){
                                                                step_info=ant_step(step_info.end,conection_cw[sides],step_info.dir,Map);
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
                                        if(direction==0){
                                            p[k+1]=o.start;
                                            p[k]=o.end;
                                        }else{
                                            p[k]=o.start;
                                            p[k+1]=o.end;
                                        }

                                        if(skip)continue;
                                        o.conected=false;
                                        opening_list.push_back(o);

                                    }
                                    for(int k=0; k<4; k++){
                                        scanMapOutHolder[p[k].x][p[k].y]=100;
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
                    scanMapOutHolder[x][y]+=scanMapOutTransform[x][y];
                }
            }
            
            
        }
        for(int x=0; x<mapSize;x++){
            for(int y=0;y<mapSize;y++){
                topMap[x][y]=scanMapOutHolder[x][y]>=100?100:-1;
            }
        }

    }

    void creatPoligonList(){
        poly_list.clear();
        int label=1;
        for(int i=0; i<opening_list.size(); i++){
            ROS_INFO("%i, start: %i, %i, end: %i, %i",i,opening_list[i].start.x, opening_list[i].start.y,opening_list[i].end.x, opening_list[i].end.y);
        }
        for(int i=0; i<opening_list.size(); i++){
            if(opening_list[i].conected==false){
                bool cw=true;
                poligon poly;
                poly.sides.push_back(opening_list[i]);
                opening_list[i].conected=true;
                ant_data step_info;      
                bool cheek=false, complet=false, nextOpening=false;
                int firstIndex=i;
                int targetIndex=i;
                int lastIndex=-1;
                ROS_INFO("---");
                while(!cheek){
                    //ROS_INFO("%i",targetIndex);
                    int empty_cell_count=0;
                    if(cw){
                        step_info.end=opening_list[targetIndex].start; 
                    }else{
                        step_info.end=opening_list[targetIndex].end;
                    }
                    
                    step_info.dir={0,0};
                    nextOpening=false;
                    for(int s=0; s<=sercheLenthAntConect; s++){
                        for(int h=0; h<opening_list.size();h++){
                            if(h!=targetIndex){
                                if(step_info.end==opening_list[h].start && cw ||
                                    step_info.end==opening_list[h].end && !cw ||
                                    empty_cell_count>maxAntGap || s==sercheLenthAntConect){
                                        if(cw){
                                            ROS_INFO("found no conecttion sershing other dir");
                                            lastIndex=targetIndex;
                                            targetIndex=firstIndex;
                                            cw=false;
                                            nextOpening=true;
                                            break;
                                        }else{
                                            if(poly.sides.size()>1){
                                                opening newOp;
                                                newOp.start=opening_list[targetIndex].end;
                                                newOp.end=opening_list[lastIndex].start;
                                                newOp.conected=true;
                                                opening_list.push_back(newOp);
                                                poly.sides.push_back(opening_list[opening_list.size()-1]);
                                                complet=true;
                                            }
                                            ROS_INFO("found no conection don serching");
                                            nextOpening=true;
                                            cheek=true;
                                            break;
                                        }
                                }
                                else if(step_info.end==opening_list[h].end && cw ||
                                    step_info.end==opening_list[h].start && !cw){
                                        if(opening_list[h].conected==false){
                                            ROS_INFO("found conection");
                                            poly.sides.push_back(opening_list[h]);
                                            opening_list[h].conected=true;
                                            targetIndex=h;
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
                        //ROS_INFO("%i, %i, %i, %i",step_info.end.x,step_info.end.y,targetIndex,label);
                        step_info=ant_step(step_info.end,cw,step_info.dir,Map);
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
        for(int i=0; i<poly_list.size();i++){
            if(poly_list.poly[i].sides.size()<3){
                poly_list.remove(i);
                i--;
            }
        }
    }

    void pubMap(){
        topoMapMsg.header.stamp = ros::Time::now();
        for(int y=0; y<mapSize;y++){
            for(int x=0;x<mapSize;x++){  
                int index=x+y*mapSize;            
                topoMapMsg.data[index]=topMap[x][y];
            }
        }
        pubTopoMap.publish(topoMapMsg);

        jsk_recognition_msgs::PolygonArray pubPolyArray_old;
        pubPolyArray_old.header.frame_id = "map";
        pubPolyArray_old.header.stamp = ros::Time::now();
        pubPolyArray_old.polygons.resize(opening_list.size());
        pubPolyArray_old.labels.resize(opening_list.size());
        pubPolyArray_old.likelihood.resize(opening_list.size());
        int halfMap=mapSize/2;
        for(int i=0; i<opening_list.size(); i++){
            point_int nNorm={-(opening_list[i].end.y-opening_list[i].start.y),
                            opening_list[i].end.x-opening_list[i].start.x};
            int l=sqrt(nNorm.x*nNorm.x+nNorm.y*nNorm.y);
            nNorm.x=nNorm.x/l;
            nNorm.y=nNorm.y/l;

            geometry_msgs::PolygonStamped p;
            p.header.frame_id = "map";
            p.header.stamp = ros::Time::now();
            p.polygon.points.resize(4);

            p.polygon.points[0].x=(opening_list[i].start.x-halfMap)*resolution;
            p.polygon.points[0].y=(opening_list[i].start.y-halfMap)*resolution;
            p.polygon.points[0].z=10.1;

            p.polygon.points[1].x=(opening_list[i].end.x-halfMap)*resolution;
            p.polygon.points[1].y=(opening_list[i].end.y-halfMap)*resolution;
            p.polygon.points[1].z=10.1;
            
            p.polygon.points[2].x=(opening_list[i].end.x+nNorm.x-halfMap)*resolution;
            p.polygon.points[2].y=(opening_list[i].end.y+nNorm.y-halfMap)*resolution;
            p.polygon.points[2].z=10.1;

            p.polygon.points[3].x=(opening_list[i].start.x+nNorm.x-halfMap)*resolution;
            p.polygon.points[3].y=(opening_list[i].start.y+nNorm.y-halfMap)*resolution;
            p.polygon.points[3].z=10.1;

            pubPolyArray_old.polygons[i]=p;
            pubPolyArray_old.labels[i]=i+1;
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

    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "topology_mapping");
    
    TopologyMapping topMapping;

    ROS_INFO("Topology Mapping Started.");
    
    topMapping.spin();
    
    return 0;
}