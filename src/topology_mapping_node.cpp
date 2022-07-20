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
        /*vector<point_int> points;
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
        }*/
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
        int numberOfDir=30;
        int maxGapDistans=10;
        int extendDevider=4;
        int searchLenght=8;

        //global var
        int **scanMap;
        int **scanMapOut;
        int **scanMapOutTransform;
        int **scanMapOutHolder;
        scanGroup **scanGarray;
        vector<opening> oplist;
        Poligon_list poly_list;
        poligon poly;
        int *scanGroupIndex;
        int currentRotationIndex=0;

        //ant para
        int searchLenghtClean=100;
        int sercheLenthAnt=600;
        int sercheLenthAntConect=2000;
        int sercheLenthAntConectPath=2000;
        int maxAntGap=4;
        int poligonRez=4;
        int poligonRezPath=10;
        int minimumSercheLenght=5;

        double minimumAreaTest=0.3;
        


    public:
        TopologyMapping(){
            subOccupancyMap= nh.subscribe("/occupancy_map_global",1,&TopologyMapping::updateMap, this);
            pubTopoMap=nh.advertise<nav_msgs::OccupancyGrid>("/topology_map",5);
            pubTopoPoly_debug=nh.advertise<jsk_recognition_msgs::PolygonArray>("/topology_poly_debug",5);
            pubTopoPoly=nh.advertise<jsk_recognition_msgs::PolygonArray>("/topology_poly",5);
            loadMemory();
            initializeTopoMap();
        }
        

    void loadMemory(){
        oplist.resize(200);
        poly_list.poly.resize(250);
        poly_list.label.resize(250);
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
        ros::Rate rate(100); // Hz
        
        while (ros::ok()){
            oplist.clear();
            poly_list.clear();

            for(int i=0; i<numberOfDir;i++){
                topologyScan();
                for(int j=0; j<oplist.size(); j++){
                    if(oplist[j].label==-1){
                        oplist.erase(oplist.begin()+j);
                        j-=1;
                    }
                }
                
            }
            for(int j=0; j<oplist.size(); j++){
                fitToCoridor(&oplist[j],40,true, true);
                if(dist(oplist[j].start,oplist[j].end)<minGroupSize){
                    oplist.erase(oplist.begin()+j);
                    j-=1;;
                }
            }
            
            if(oplist.size()>1){
                creatPoligonList();
                if(poly_list.size()>0){
                    creatPathPoligons();
                }
            
                
                for(int b=0; b<oplist.size(); b++){
                    if(oplist[b].parent_poligon==-1 || oplist[b].label==-1){
                        oplist.erase(oplist.begin()+b);
                        b-1;
                    }
                }
                
                for(int b=0; b<poly_list.poly.size(); b++){
                    if(poly_list.poly[b].inactiv){
                        poly_list.poly.erase(poly_list.poly.begin()+b);
                        b-1;
                    }
                }
                
                
            }
            pubMap();
            ros::spinOnce();
            rate.sleep();
        }
    }
    double polyAreaTest(poligon poly){
        double area = 0.0;
        vector<point> p;
        double tdist=0.0;
        for(int i=0; i<poly.sidesIndex.size(); i++){
            p.push_back(oplist[poly.sidesIndex[i]].end.convert_to_point(resolution));
            p.push_back(oplist[poly.sidesIndex[i]].start.convert_to_point(resolution));
            tdist+=dist(oplist[poly.sidesIndex[i]].start,oplist[poly.sidesIndex[i]].end)*resolution;
        }

        int j = p.size() - 1;
        for (int i = 0; i < p.size(); i++)
        {
            area += (p[j].x + p[i].x) * (p[j].y - p[i].y);
            j = i;  
        }

        return abs(area / 2.0)/tdist;
    }

    bool ccw(point A,point B,point C){
        return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
    }

    // Return true if line segments o1 and o2 intersect
    bool intersect_line(opening o1,opening o2){
        double l=1.2;
        double l1 = dist(o1.start,o1.end);
        double l2 = dist(o2.start,o2.end);
        point n1={((o1.end.x-o1.start.x)/l1)*l, ((o1.end.y-o1.start.y)/l1)*l};
        point n2={((o2.end.x-o2.start.x)/l2)*l, ((o2.end.y-o2.start.y)/l2)*l};

        point A={o1.start.x-n1.x,o1.start.y-n1.y}, B={o1.end.x+n1.x,o1.end.y+n1.y};
        point C={o2.start.x-n2.x,o2.start.y-n2.y}, D={o2.end.x+n2.x,o2.end.y+n2.y}; 
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
            step.dir=rotate_dir(step.dir,!clockwise);
        }
        point_int checkDir=direction;
        int checkLenght=1;//(int)(std::round(minGroupSize/2));

        for(int d=0;d<8;d++){
            bool check=true;
            for(int c=0; c<checkLenght; c++){  
                if(Map[start.x+step.dir.x+checkDir.x*c][start.y+step.dir.y+checkDir.y*c]!=0){
                    check=false;
                    break;
                }
                
            }
            if(check){
                //scanMapOutHolder[step.end.x][step.end.y]=100;
                step.end.x=start.x+step.dir.x;
                step.end.y=start.y+step.dir.y;
                step.emty_cell=false;
                point_int dir=step.dir;
                for(int e=0;e<8;e++){
                        
                    if(Map[step.end.x+dir.x][step.end.y+dir.y]==-1){
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

    //checks if a point is overlaping an opening start and end, typ: 1 check for start, 2 check for end, 3 check for both.
    bool check_for_opening(point_int position, int typ){
        bool testCheck=true;
        
        if(check_and_get_opening(position, typ)==-1) testCheck=false;

        return testCheck;
    }
    //checks and get index to first found opening at a point, typ: 1 check for start, 2 check for end, 3 check for both.
    int check_and_get_opening(point_int position, int typ, bool only_standard_opening=false){
        int oIndex=-1;
        
        for(int i=0; i<oplist.size(); i++){
            if(oplist[i].label!=-1 && (!only_standard_opening || oplist[i].label==1) &&
                (typ==1 && oplist[i].start==position || typ==2 && oplist[i].end==position ||
                typ==3 && (oplist[i].start==position || oplist[i].end==position))){
                    oIndex=i;
            }
        }

        return oIndex;
    }

    bool fitToCoridor(opening *op,int inSearchLenght, bool limitBothSids=false, bool oneDir=false, vector<point_int> *sida1=NULL,vector<point_int> *sida2=NULL){
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
                search_step=ant_step(search_step.end,cw,search_step.dir);
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
                search_step=ant_step(search_step.end,cw,search_step.dir);
                if(sida1!=NULL){
                    s2.push_back(search_step.end);
                }

                if(!limitBothSids && check_for_opening(search_step.end,op->start_is_outside?1:2) ||
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
        point *polyp[]={s1,s2};
        for(int index=0; index<2;index++){
            for(int i=0;i<4;i++){
                point p1=polyp[index][i];
                point p2=polyp[index][(i+1)%4];

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
        topoMapMsg.info.origin.position.z = 10.05; //add 10 for visualization

        topoMapMsg.data.resize(topoMapMsg.info.width * topoMapMsg.info.height);
        std::fill(topoMapMsg.data.begin(), topoMapMsg.data.end(), -1);
    }

    void updateMap(const nav_msgs::OccupancyGrid& mapMsg){
        int width=mapMsg.info.width;
        int height=mapMsg.info.height;
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

    bool cleanOpenings(opening op){
        vector<int> vo;
        vector<int> vint;
        bool selfDel=true;
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
                for(int s=0; s<searchLenghtClean && empty_count<maxAntGap; s++){
                    
                    if(check_for_opening(step.end,sids==0?2:1))break;


                    if(step.emty_cell){
                        empty_count+=1;
                    }
                    if(sids==0 && step.end==op.end){
                        //return false;
                    }

                    for(int j=0; j<oplist.size(); j++){
                        if(sids==0 && oplist[j].start==step.end && oplist[j].label!=-1){
                            vo.push_back(j);
                            int sepL=cw==1?s:-s;
                            vint.push_back(sepL);
                        }
                        if(sids==1 && oplist[j].end==step.end && oplist[j].label!=-1){

                            for(int index=0; index<vo.size(); index++){
                                if(vo[index]==j){
                                    vint[index]+=cw==0?s:-s;
                                
                                    if(dist(op.start,op.end)+vint[index]/extendDevider<dist(oplist[j].start,oplist[j].end)){
                                        oplist[j].label=-1;
                                        return true;
                                    }else{
                                        selfDel = false;
                                        return false;
                                    }
                                    break;
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

    bool checkForWall(opening o,int maximumWallCount){
        int wallcount=0;
        double opLenght=dist(o.start,o.end);
        point normdVop={(o.end.x-o.start.x)/(opLenght*1.2),(o.end.y-o.start.y)/(opLenght*1.2)};
        for(int wallScan=0;wallScan<(int)(opLenght*1.2);wallScan++){
            point_int pp={o.start.x+(int)(std::round(normdVop.x*wallScan)), o.start.y+(int)(std::round(normdVop.y*wallScan))};
            if(Map[pp.x][pp.y]==100){
                wallcount+=1;
            }
        }
        if(wallcount<maximumWallCount){
            return false;
        }else{
            return true;
        }
    }

    void topologyScan(){
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
                                int inSearchLenght=40;
                                
                                p[0].x=i;
                                p[0].y=firstCoridorCenter->start;
                                p[1].x=i;
                                p[1].y=firstCoridorCenter->end;
                                p[2].x=i;
                                p[2].y=scondCoridorCenter->start;
                                p[3].x=i;
                                p[3].y=scondCoridorCenter->end;
                                rotate_points(p,4,rotation,mapSize);

                                for(int k=0; k<4;k+=2){
                                    opening o;
                                    if(direction==0){
                                        o.start=p[k+1];
                                        o.end=p[k];
                                    }else{
                                        o.start=p[k];
                                        o.end=p[k+1];
                                    }

                                    o.start_is_outside=k==0&&direction==1||k==2&&direction==0;
                                    correctOpening(&o);
                                   
                                    if(!fitToCoridor(&o,inSearchLenght)) continue;
                                    double opLenght=dist(o.start,o.end);
                                    if(opLenght<minGroupSize){
                                        continue;
                                    }
                                    
                                    for(int sids=0; sids<2;sids++){
                                        ant_data step;
                                        step.end=sids==0?o.start:o.end;
                                        bool cheek=false;
                                        int c=0;
                                        while (!cheek && c<inSearchLenght){
                                            c++;
                                            cheek=true;
                                            for(int n=0; n<oplist.size(); n++){
                                                if(step.end==oplist[n].end || step.end==oplist[n].start){
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
                                    }
                                    bool skip=false;
                                for(int h=0; h<oplist.size() && !skip && o.label!=-1; h++){
                                        opening oplistComp=oplist[h];
                                        if(intersect_line(o,oplistComp) && oplistComp.label!=-1){

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
                                                if(dist(o.start,o.end)<dist(oplistComp.start,oplistComp.end)){
                                                    oplist.erase(oplist.begin()+h);
                                                    h-=1;
                                                    break;
                                                }else{
                                                    skip=true;
                                                    break;
                                                }
                                                
                                            }else if(conected[0] && conected[1] && conection_waite!=-1){
                                                if(dist(o.start,o.end)+conection_waite/extendDevider<dist(oplistComp.start,oplistComp.end)){
                                                    oplist.erase(oplist.begin()+h);
                                                    h-=1;
                                                    break;
                                                }else{
                                                    skip=true;
                                                    break;
                                                }

                                            }else if(conection_lenght[0]<conection_lenght[1] && conected[0] || !conected[1]){
                                                sides=0;
                                            }
                                            int c=0;
                                            step_info.end=conect[sides];
                                            step_info.dir=conection_dir[sides];
                                            
                                            while(intersect_line(oplistComp,o) && c<inSearchLenght){
                                                c+=1;
                                                step_info=ant_step(step_info.end,conection_cw[sides],step_info.dir);
                                                if(sides==0){
                                                    o.start.x=step_info.end.x;
                                                    o.start.y=step_info.end.y;
                                                }else{
                                                    o.end.x=step_info.end.x;
                                                    o.end.y=step_info.end.y;
                                                }
                                            }
                                            
                                            
                                            point_int *op;
                                            op=&step_info.end;
                                            int cIndex=-1;
                                            bool cheek=false;
                                            int deb=0;
                                            c=0;
                                            while (!cheek && c<inSearchLenght){
                                                c++;
                                                cheek=true;
                                                for(int n=0; n<oplist.size(); n++){
                                                    if(step_info.end==oplist[n].end || step_info.end==oplist[n].start){
                                                        step_info=ant_step(step_info.end,conection_cw[sides],step_info.dir);
                                                        if(sides==0){
                                                            o.start.x=step_info.end.x;
                                                            o.start.y=step_info.end.y;
                                                        }else{
                                                            o.end.x=step_info.end.x;
                                                            o.end.y=step_info.end.y;
                                                        }
                                                        cheek=false;
                                                    }
                                                }
                                                    
                                            }
                                            if(intersect_line(o,oplistComp)){
                                                if(dist(o.start,o.end)<dist(oplistComp.start,oplistComp.end)){
                                                    oplist.erase(oplist.begin()+h);
                                                    h-=1;
                                                    break;
                                                }else{
                                                    skip=true;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    for(int h=0; h<oplist.size() && !skip; h++){
                                        opening oplistComp1=oplist[h];
                                        if(intersect_line(o,oplistComp1) && oplistComp1.label!=-1){
                                            if(dist(o.start,o.end)<dist(oplistComp1.start,oplistComp1.end)){
                                                oplist.erase(oplist.begin()+h);
                                                h-=1;
                                                break;
                                            }else{
                                                skip=true;
                                                break;
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
                                    if(checkForWall(o, 2))continue;
                                    if(!cleanOpenings(o)){
                                         continue;//o.label=-1;
                                    }
                                    oplist.push_back(o);
                                    
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

    void remove_parent_poligon(int opIndex, bool remove_openign=false){
        if(opIndex>=0){
            if(oplist[opIndex].parent_poligon>=0){
                int pIndex=oplist[opIndex].parent_poligon;
                for(int i=0; i<poly_list.poly[pIndex].sidesIndex.size();i++){
                    oplist[poly_list.poly[pIndex].sidesIndex[i]].parent_poligon=-1;
                    poly_list.poly[pIndex].inactiv=true;
                }
            }
            if(remove_openign){
                oplist[opIndex].label=-1;
            }
        }
    }

    void creatPoligonList(){
        for(int i=0; i<oplist.size(); i++){
            if(oplist[i].parent_poligon==-1 && oplist[i].label==1){
                bool cw=true;
                poly.poligon_points.clear();
                poly.sidesIndex.clear();
                poly.inactiv=false;
                poly.label=1;
                poly.sidesIndex.push_back(i);
                oplist[i].parent_poligon=-2;
                ant_data step_info;      
                bool cheek=false, complet=false;
                int firstIndex=i;
                int targetIndex=i;
                int lastIndex=-1;
                vector<point_int> poly_points;
                int wait=-1;
                int totalSlenght=0;
                int opIndex;
                while(!cheek){
                    int empty_cell_count=0;
                    if(cw){
                        step_info.end=oplist[targetIndex].start; 
                    }else{
                        step_info.end=oplist[targetIndex].end;
                    }
                    poly.add_point(step_info.end,cw);
                    int rezCounter=0;
                    poly_points.clear();
                    step_info.dir={0,0};
                    for(int s=0; s<=sercheLenthAntConect; s++){
                         
                        if(rezCounter>=poligonRez){
                            rezCounter=0;
                            poly_points.push_back(step_info.end);
                        }else{
                            rezCounter+=1;
                        }
                        step_info=ant_step(step_info.end,cw,step_info.dir);

                        opIndex=check_and_get_opening(step_info.end,cw?1:2,true);
                        
                        // found bad conection
                        if(opIndex!=-1 ||
                            empty_cell_count>maxAntGap || s==sercheLenthAntConect){
                                if(cw){
                                    wait=s;

                                    ROS_INFO("found no conecttion sershing other dir");
                                    lastIndex=targetIndex;
                                    targetIndex=firstIndex;
                                    cw=false;
                                    break;
                                }else{
                                    if(poly.sidesIndex.size()>1){
                                        ROS_INFO("creat new opening");
                                        //creat mising openign
                                        opening newOp;
                                        newOp.start=oplist[targetIndex].end;
                                        newOp.end=oplist[lastIndex].start;
                                        vector<point_int> s1, s2;
                                        fitToCoridor(&newOp,40,true,true,&s1,&s2);
                                        newOp.label=2;
                                        newOp.parent_poligon=-2;
                                        poly.sidesIndex.push_back(oplist.size());
                                        int newIndex=oplist.size();
                                        oplist.push_back(newOp);
                                        if(!checkForWall(newOp,2)){
                                            for(int k=0; k<s2.size();k+=poligonRez){
                                                poly.add_point(s2[k],cw);
                                            }
                                            if(s2.size()>1) poly.add_point(newOp.end,cw);
                                            if(s1.size()>1) poly.add_point(newOp.start,cw);
                                            for(int k=s1.size()-1; k>0;k-=poligonRez){
                                                poly.add_point(s1[k],cw);
                                            }
                                    
                                            complet=true;
                                        }else{//bad new opening, remove conflicting opening insted
                                            oplist[newIndex].parent_poligon=-1;
                                            if(opIndex>=0){
                                                remove_parent_poligon(opIndex,true);
                                                i-=1;
                                            }
                                            cheek=true;
                                            
                                        }
                                    }
                                    
                                    ROS_INFO("found no conection don serching");
                                    cheek=true;
                                    break;
                                }
                        }
                        
                        opIndex=check_and_get_opening(step_info.end,cw?2:1,true);
                        //Good conection
                        
                        if(opIndex!=-1){
                                poly_points.push_back(step_info.end);
                                for(int k=0;k<poly_points.size(); k++){
                                    poly.add_point(poly_points[k],cw);
                                    
                                }
                                totalSlenght+=s;
                                if(oplist[opIndex].parent_poligon!=-1 &&oplist[opIndex].parent_poligon!=-2){
                                    remove_parent_poligon(opIndex);
                                }
                                if(oplist[opIndex].parent_poligon==-1){
                                    oplist[opIndex].parent_poligon=-2;
                                    poly.sidesIndex.push_back(opIndex);
                                    targetIndex=opIndex;
                                    break;
                                }else if(poly.sidesIndex.size()==2 && totalSlenght<=minimumSercheLenght){//to smal conection fliping openings
                                    ROS_INFO("small conection");
                                    oplist[targetIndex].parent_poligon=-1;
                                    if(oplist[i].fliped){//remove opening if flipt two times
                                        oplist[i].label=-1;
                                    }else{
                                        oplist[i].parent_poligon=-1;
                                        oplist[targetIndex].flip();
                                        oplist[i].flip();
                                        oplist[i].fliped=true;
                                        
                                    }
                                    i-=1;
                                    cheek=true;
                                    break;
                                }else{
                                    ROS_INFO("found complet");
                                    
                                    cheek=true;
                                    complet=true;
                                    break;
                                }
                        }

                        if(step_info.emty_cell){
                            empty_cell_count+=1;
                        }  
                    }
                    if(complet){
                        int label=0;
                        for(int p=0;p< poly.sidesIndex.size();p++){
                            ROS_INFO("%i, start: %i, %i, end: %i, %i",i,oplist[poly.sidesIndex[p]].start.x, oplist[poly.sidesIndex[p]].start.y,oplist[poly.sidesIndex[p]].end.x, oplist[poly.sidesIndex[p]].end.y);
                        }
                        if(poly.sidesIndex.size()==1){
                            label=15;
                        }else if(poly.sidesIndex.size()==2){
                            label=20;
                        }else{
                            label=30;
                        }

                        for(int p=0;p<poly.sidesIndex.size();p++){
                            oplist[poly.sidesIndex[p]].parent_poligon=poly_list.poly.size();
                        }
                        poly_list.add(poly,label);
                    }else if(cheek){
                        for(int p=0;p<poly.sidesIndex.size();p++){
                            oplist[poly.sidesIndex[p]].parent_poligon=-1;
                        }
                    }
                }
            }
        }
    }

    void creatPathPoligons(){
        for(int i=0; i<oplist.size(); i++){
            if(oplist[i].conected_to_path==-1 && oplist[i].parent_poligon>=0 && !poly_list.poly[oplist[i].parent_poligon].inactiv){
                
                poly.poligon_points.clear();
                poly.sidesIndex.clear();
                poly.inactiv=false;
                poly.label=1;
                poly.sidesIndex.push_back(i);
                oplist[i].conected_to_path=-2;
                ant_data step_info;      
                bool check=false;
                bool complet=false;
                int opIndex, firstOpIndex=-2, pathType=0;
                for(int sids=0; sids<2 && !check; sids++){
                    int empty_cell_count=0;
                    bool cw=sids==0;
                    if(cw){
                        step_info.end=oplist[i].end; 
                    }else{
                        step_info.end=oplist[i].start;
                    }
                    point_int firstPos=step_info.end;
                    poly.add_point(step_info.end,cw);
                    int rezCounter=0;
                    
                    step_info.dir={0,0};
                    for(int s=0; s<=sercheLenthAntConect; s++){
                         
                        if(rezCounter>=poligonRezPath){
                            rezCounter=0;
                            poly.add_point(step_info.end,cw);
                        }else{
                            rezCounter+=1;
                        }
                        step_info=ant_step(step_info.end,cw,step_info.dir);

                        if(step_info.end==firstPos){
                            check=true;
                            break;
                        }

                        opIndex=check_and_get_opening(step_info.end,cw?1:2);
                        
                        // found conection
                        if(opIndex!=-1 || empty_cell_count>maxAntGap || s==sercheLenthAntConect){
                            if(oplist[opIndex].parent_poligon>=0){
                                oplist[opIndex].conected_to_path=-2;
                                poly.sidesIndex.push_back(opIndex);
                                poly.add_point(step_info.end,cw);

                                if(cw){
                                    firstOpIndex=opIndex;
                                    if(opIndex==i){
                                        check=true;
                                        complet=true;
                                        pathType=1;
                                    }else if(opIndex==-1){
                                        pathType=2;
                                    }
                                    break;
                                }else{
                                    if(opIndex!=firstOpIndex){
                                        
                                        ROS_INFO("warning, path wiht more then two coneciton");
                                        pathType=4;
                                    }else if(pathType==0){
                                        pathType=3;
                                    }

                                    complet=true;
                                        
                                    break;
                                }
                            }
                        }
                        if(step_info.emty_cell){
                            empty_cell_count+=1;
                        }  
                    }
                }
                if(complet){
                    int label=0;
                    switch (pathType)
                    {
                    case 1:
                        label=41;
                        break;

                    case 2:
                        label=52;
                        break;
                    
                    case 3:
                        label=64;
                        break;

                    case 4:
                        label=76;
                        break;

                    default:
                        break;
                    }

                    for(int p=0;p<poly.sidesIndex.size();p++){
                        oplist[poly.sidesIndex[p]].conected_to_path=poly_list.poly.size();
                    }
                    poly_list.add(poly,label);
                }else if(check){
                    for(int p=0;p<poly.sidesIndex.size();p++){
                        oplist[poly.sidesIndex[p]].conected_to_path=-1;
                    }
                }
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
            
            opening op=oplist[i];
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
            topMap[op.end.x][op.end.y]=50;
            p.polygon.points[0].x=(op.start.x-halfMap)*resolution;
            p.polygon.points[0].y=(op.start.y-halfMap)*resolution;
            p.polygon.points[0].z=10.2;

            p.polygon.points[1].x=(op.end.x-halfMap)*resolution;
            p.polygon.points[1].y=(op.end.y-halfMap)*resolution;
            p.polygon.points[1].z=10.2;
            
            p.polygon.points[2].x=(op.end.x-halfMap)*resolution+nNorm.x;
            p.polygon.points[2].y=(op.end.y-halfMap)*resolution+nNorm.y;
            p.polygon.points[2].z=10.2;

            p.polygon.points[3].x=(op.start.x-halfMap)*resolution+nNorm.x;
            p.polygon.points[3].y=(op.start.y-halfMap)*resolution+nNorm.y;
            p.polygon.points[3].z=10.2;
            
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