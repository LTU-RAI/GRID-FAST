#include "Utility.h"                                

class TopologyMapping{
    private:
        //ROS nh
        ros::NodeHandle nh;
        //subs
        ros::Subscriber subOccupancyMap;
        //pub
        ros::Publisher pubTopoMap;
        ros::Publisher pubOpeningList;

        //msg
        nav_msgs::OccupancyGrid topoMapMsg;
        //Map
        int **Map;
        int **topMap;

        //global var
        int **scanMap;
        int **scanMapOut;
        int **scanMapOutTransform;
        scanGroup ***scanGarray;
        int **scanGroupIndex;
        point_int ***mapTransform;
        


    public:
        TopologyMapping(){
            subOccupancyMap= nh.subscribe("/occupancy_map_global",1,&TopologyMapping::updateMap, this);
            pubTopoMap=nh.advertise<nav_msgs::OccupancyGrid>("/topology_map_filterd",5);
            pubOpeningList=nh.advertise<topology_mapping::opening_list>("/opening_list_int",5);
            loadMemory();
            initializeTopoMap();
        }
        

    void loadMemory(){
        generateMapTransform();
        oplist.resize(200);
        //alocate for Maps
        scanMap=new int*[scanSize];
        scanMapOut=new int*[scanSize];
        scanMapOutTransform=new int*[scanSize];
        for(int i=0; i<scanSize;i++){
            scanMapOut[i]=new int[scanSize];
            scanMap[i]=new int[scanSize];
            scanMapOutTransform[i]=new int[scanSize];
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
                scanMapOutTransform[i][j] = -1;
            }
        }

        scanGarray=new scanGroup**[numberOfDir];
        scanGroupIndex=new int*[numberOfDir];
        for(int i=0; i<numberOfDir; i++){
            scanGarray[i]=new scanGroup*[scanSize];
            scanGroupIndex[i]=new int[scanSize];
            for(int k=0; k<scanSize; k++){
                scanGarray[i][k]=new scanGroup[groupeNumber];
                scanGroupIndex[i][k]=0;
            }
        }
    }


    void spin(){
        ros::Rate rate(100); // Hz
        
        while (ros::ok()){
            oplist.clear();

            topologyScan();
            for(int j=0; j<oplist.size(); j++){
                if(oplist[j].label==-1){
                    oplist.erase(oplist.begin()+j);
                    j-=1;
                }
            }
                
            
            for(int j=0; j<oplist.size(); j++){
                if(oplist[j].label!=-1){
                    fitToCoridor(&oplist[j],40,topMap,true, true);
                    if(dist(oplist[j].start,oplist[j].end)<minGroupSize){
                        oplist.erase(oplist.begin()+j);
                        j-=1;
                    }
                }
            }
            for(int j=0; j<oplist.size(); j++){
                for(int b=0; b<oplist.size(); b++){
                    if(intersect_line(oplist[j],oplist[b],topMap) && oplist[b].label!=-1 && oplist[j].label!=-1 && j!=b){
                        ROS_INFO("overlap");
                        if(dist(oplist[j].start,oplist[j].end)<dist(oplist[b].start,oplist[b].end)){
                            oplist[b].label=-1;
                        }else{
                            oplist[j].label=-1;
                        }
                    }
                    
                }
                
            }
            pubMap();
            ros::spinOnce();
            rate.sleep();
        }
    }

    void generateMapTransform(){
        mapTransform= new point_int**[numberOfDir];

        for(int angle=0; angle<numberOfDir; angle++){
            mapTransform[angle]=new point_int*[mapSize];
            for(int i=0; i<mapSize;i++){
                mapTransform[angle][i]=new point_int[mapSize];
            }
            float rotation=M_PI*angle/numberOfDir;
            float cosA=cos(rotation);
            float sinA=sin(rotation);
            int halfMapSize=mapSize/2;
            for(int x=0;x<mapSize;x++){
                for(int y=0;y<mapSize;y++){
                    int newX=int(std::round(((x-halfMapSize) *cosA-(y-halfMapSize)*sinA)))+halfMapSize;
                    int newY=int(std::round(((x-halfMapSize) *sinA+(y-halfMapSize)*cosA)))+halfMapSize;
                    if((newX<0||newX>=mapSize)||(newY<0||newY>=mapSize)){
                        mapTransform[angle][x][y]={-1,-1};
                    }else{
                        mapTransform[angle][x][y]={newX,newY};
                    }

                }
            }
        }
    }

    int getMapTransform(int **map, const int x,const int y, const int angIndex){
        if(mapTransform[angIndex][x][y].x==-1) return -1;
        return map[mapTransform[angIndex][x][y].x][mapTransform[angIndex][x][y].y];
    }

    void setMapTransform(int **map, const int x,const int y, const int angIndex, int value){
        if(mapTransform[angIndex][x][y].x==-1) return;
        map[mapTransform[angIndex][x][y].x][mapTransform[angIndex][x][y].y]=value;
    }

    /*void rotate_points(int **map, point_int *p,const int angIndex){
        *p= mapTransform[angIndex][p->x][p->y];
    }*/
    vector<opening> rotate_points(vector<opening> op,const int size, const double rotation, const int sizeMap){
        vector<opening> newOp;
        newOp.resize(op.size());
        float cosA=cos(rotation);
        float sinA=sin(rotation);
        int halfMapSize=sizeMap/2;
        for(int i=0; i<op.size();i++){
            for(int sids=0; sids<2;sids++){
                point_int p=op[i].start;
                if(sids==1){
                    p=op[i].end;
                }
                
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

    void rotateMap(float angel,int sizeMap,int **mapOut,int **map){
        float cosA=cos(angel);
        float sinA=sin(angel);
        int halfMapSize=sizeMap/2;
        for(int x=0;x<sizeMap;x++){
            for(int y=0;y<sizeMap;y++){
                int newX=int(std::round(((x-halfMapSize) *cosA-(y-halfMapSize)*sinA)))+halfMapSize;
                int newY=int(std::round(((x-halfMapSize) *sinA+(y-halfMapSize)*cosA)))+halfMapSize;
                if((newX<0||newX>=sizeMap)||(newY<0||newY>=sizeMap)){
                    mapOut[x][y]=-1;
                }else{
                    mapOut[x][y]=map[newX][newY];
                }

            }
        }
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

            while (topMap[p1->x][p1->y]!=0){
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
            while(topMap[p1->x+dirX*(lenght+1)][p1->y+dirY*(lenght+1)]==0){
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
                    step=ant_step(step.end,(bool)cw, step.dir,topMap);
                }
            }
        }

        return selfDel;
    }

    void topologyScan(){
        for(int x=0; x<mapSize;x++){
            for(int y=0;y<mapSize;y++){
                topMap[x][y]=-1;
            }
        }

        for(int angle=0; angle<numberOfDir; angle++){
            float rotation=M_PI*angle/numberOfDir;
            bool filter=numberOfDir%((int)(numberOfDir/numberOfDirFilter))==0;
            //Copy the map to scanMap
            //rotateMap(rotation,mapSize,scanMap,Map);
            
            //Clear scanMapOutput from previus scan
            for (int i = 0; i < scanSize; ++i){
                scanGroupIndex[angle][i]=0;
            }
            
            //Loop thur all map cells
            for(int i=0;i<scanSize;i++){

                for(int j=1;j<scanSize-1;j++){

                    //finde groups
                    if(getMapTransform(Map,i,j,angle)==0){
                        //check if space is free
                        if(cGroupeSize==0){
                            scanGarray[angle][i][scanGroupIndex[angle][i]].start=j;
                        }
                        cGroupeSize+=1;
                        cfilter=0;
                    //filter out smal point opstacals
                    }else if (getMapTransform(Map,i,j,angle)==-1 && cfilter<cfilterSize){
                        cfilter+=1;
                    //if findeing ostacals biger then filter end serche
                    }else{
                        //if found groupe is larger then minGroupSize add it as group
                        if(cGroupeSize>minGroupSize){
                            scanGarray[angle][i][scanGroupIndex[angle][i]].end=j-1-cfilter;
                            scanGarray[angle][i][scanGroupIndex[angle][i]].prevGroupIndex=0;
                            scanGarray[angle][i][scanGroupIndex[angle][i]].prevGroup[0]=NULL;
                            scanGarray[angle][i][scanGroupIndex[angle][i]].nextGroupIndex=0;
                            scanGarray[angle][i][scanGroupIndex[angle][i]].nextGroup[0]=NULL;
                            if(filter){
                                for(int m=scanGarray[angle][i][scanGroupIndex[angle][i]].start;
                                    m<j-cfilter; m++){
                                        setMapTransform(scanMapOut,i,m,angle,0);
                                }
                            }
                            scanGroupIndex[angle][i]+=1;

                        }else if(cGroupeSize>0 && filter){
                            for(int m=scanGarray[angle][i][scanGroupIndex[angle][i]].start;
                                m<j-cfilter; m++){
                                    setMapTransform(scanMapOut,i,m,angle,getMapTransform(Map,i,scanGarray[angle][i][scanGroupIndex[angle][i]].start-1,angle));
                            }
                        }
                        cfilter=0;
                        cGroupeSize=0;
                    }
                }

                //find conection_lenghts between groups
                //check if ther are les grupe then previus line
                for(int index1=0;index1<scanGroupIndex[angle][i];index1++){
                    //find if there is tow or more gropse conecteing to a previus group
                    for(int index2=0;index2<scanGroupIndex[angle][i-1];index2++){
                        if(scanGarray[angle][i][index1].start<scanGarray[angle][i-1][index2].end &&
                            scanGarray[angle][i][index1].end>scanGarray[angle][i-1][index2].start){
                                scanGarray[angle][i][index1].prevGroup[scanGarray[angle][i][index1].prevGroupIndex]=&scanGarray[angle][i-1][index2];
                                scanGarray[angle][i][index1].prevGroupIndex+=1;

                                scanGarray[angle][i-1][index2].nextGroup[scanGarray[angle][i-1][index2].nextGroupIndex]=&scanGarray[angle][i][index1];
                                scanGarray[angle][i-1][index2].nextGroupIndex+=1;
                            }
                    }
                }
            }
            if(filter){
                //rotateMap(-rotation,mapSize,scanMapOutTransform,scanMapOut);
                
                for(int x=0; x<mapSize;x++){
                    for(int y=0;y<mapSize;y++){
                        if(scanMapOut[x][y]!=-1){
                            topMap[x][y]=(scanMapOut[x][y]>=70 || topMap[x][y]>=70)?100:0;
                        }
                        scanMapOut[x][y]=-1;
                    }
                } 
            }  
        }

        for(int x=0; x<mapSize;x++){
                for(int y=0;y<mapSize;y++){
                    if(topMap[x][y]!=-1 || Map[x][y]!=-1){
                        topMap[x][y]=(Map[x][y]>=70 || topMap[x][y]>=70)?100:0;
                    }
                    scanMapOutTransform[x][y]=-1;
                    scanMapOut[x][y]=-1;
                }
        }
        for(int angle=0; angle<numberOfDir; angle++){
            float rotation=M_PI*angle/numberOfDir;
            for(int i=1;i<scanSize;i++){
                for(int index= 0; index<scanGroupIndex[angle][i];index++){
                    for(int direction=0;direction<2;direction++){
                        if(scanGarray[angle][i][index].prevGroupIndex>1&&direction==0||scanGarray[angle][i][index].nextGroupIndex>1&&direction==1){
                            scanGroup *p=&scanGarray[angle][i][index];
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

                            if(depthCount<minCoridorSize) continue;

                            vector<opening> newOpList;

                            int loopAmount=scanGarray[angle][i][index].prevGroupIndex;
                            if(direction==1){
                                loopAmount=scanGarray[angle][i][index].nextGroupIndex;
                            }
                            for(int h=0;h<loopAmount;h++){
                                depthCount=0;
                                if(direction==0){
                                    p=scanGarray[angle][i][index].prevGroup[h];
                                }else{
                                    p=scanGarray[angle][i][index].nextGroup[h];
                                }

                                while (p!=NULL){

                                    depthCount+=1;
                                    if(depthCount==minCoridorSize){
                                        opening newOp;
                                        if(direction==0){
                                            newOp.start={i,scanGarray[angle][i][index].prevGroup[h]->end};
                                            newOp.end={i,scanGarray[angle][i][index].prevGroup[h]->start};
                                        }else{
                                            newOp.start={i,scanGarray[angle][i][index].nextGroup[h]->start};
                                            newOp.end={i,scanGarray[angle][i][index].nextGroup[h]->end};
                                        }
                                        newOpList.push_back(newOp);
                                        break;
                                    }

                                    if(direction==0){
                                        p=p->prevGroup[0];
                                    }else{
                                        p=p->nextGroup[0];
                                    }
                                }
                            }

                            if(newOpList.size()<2) continue;

                            if(direction==0){
                                newOpList[0].start_is_outside=false;
                                newOpList[newOpList.size()-1].start_is_outside=true;
                            }else{
                                newOpList[0].start_is_outside=true;
                                newOpList[newOpList.size()-1].start_is_outside=false;
                            }
                            
                            int inSearchLenght=40;
                            
                            //for(int k=0;k<4;k++) rotate_points(topMap, &p[0],angle);
                            newOpList=rotate_points(newOpList,4,rotation,mapSize);

                            for(int k=0; k<newOpList.size();k++){
                                opening o=newOpList[k];

                                correctOpening(&o);
                            
                                if(!fitToCoridor(&o,inSearchLenght,topMap)) continue;

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
                                        
                                        if(check_for_opening(step.end,3)){
                                            step=ant_step(step.end,sids==0,step.dir,topMap);
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
                                bool skip=false;
                            for(int h=0; h<oplist.size() && !skip && o.label!=-1; h++){
                                    opening oplistComp=oplist[h];
                                    if(intersect_line(o,oplistComp,topMap) && oplistComp.label!=-1 && o.label!=-1){

                                        bool conected[]={false, false};
                                        point_int conect[2];
                                        int conection_lenght[]={0,0};//direction and distans to conection_lenght 0=no conection_lenght
                                        int conection_waite=-1;
                                        point_int conection_dir[2];
                                        bool conection_cw[2];
                                        ant_data step_info;
                                        
                                        for(int sides=0; sides<2; sides++){
                                            for(int d=0; d<2; d++){
                                                if(conected[sides]) continue;

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
                                                    step_info=ant_step(step_info.end,clockwise,step_info.dir,topMap);
                                                    
                                                    if(step_info.emty_cell){
                                                        emty_count+=1;
                                                        if(emty_count>maxAntGap){
                                                            break;
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
                                            
                                        }else if(conection_lenght[0]<conection_lenght[1] && conected[0] || !conected[1]){
                                            sides=0;
                                        }
                                        int c=0;
                                        step_info.end=conect[sides];
                                        step_info.dir=conection_dir[sides];
                                        
                                        while(intersect_line(oplistComp,o,topMap) && c<inSearchLenght){
                                            c+=1;
                                            step_info=ant_step(step_info.end,conection_cw[sides],step_info.dir,topMap);
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
                                            
                                            if(check_for_opening(step_info.end,3)){
                                                step_info=ant_step(step_info.end,conection_cw[sides],step_info.dir,topMap);
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
                                        if(intersect_line(o,oplistComp,topMap)){
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
                                    if(intersect_line(o,oplistComp1,topMap) && oplistComp1.label!=-1 && o.label!=-1){
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

                                if(skip)o.label=-1;
                                if(dist(o.start,o.end)<minGroupSize){
                                    continue;
                                }
                                if(checkForWall(o, 2,topMap))o.label=-1;
                                if(o.label!=-1){
                                    if(!cleanOpenings(o)){
                                        o.label=-1;
                                    }
                                }
                                oplist.push_back(o);
                                
                            } 
                        }
                    }
                }
            }
        }  
    }

    void pubMap(){
        topology_mapping::opening_list OpeningListMsg;
        OpeningListMsg.list.resize(oplist.size());
        for(int i=0; i<oplist.size(); i++){
            OpeningListMsg.list[i].start.x=oplist[i].start.x;
            OpeningListMsg.list[i].start.y=oplist[i].start.y;
            OpeningListMsg.list[i].end.x=oplist[i].end.x;
            OpeningListMsg.list[i].end.y=oplist[i].end.y;
            OpeningListMsg.list[i].label=oplist[i].label;
        }
        pubOpeningList.publish(OpeningListMsg);
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