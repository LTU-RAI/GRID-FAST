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
        //Maps
        int **Map;
        int **topMap;

        //global var
        bool **ObjectFilterLookup;
        int **scanMap;
        int **scanMapOut;
        int **scanMapOutTransform;
        scanGroup ***scanGarray;
        int **scanGroupIndex;
        point_int ***mapTransform;
        


    public:
        //setup
        TopologyMapping(){
            subOccupancyMap= nh.subscribe("/map",1,&TopologyMapping::updateMap, this);
            pubTopoMap=nh.advertise<nav_msgs::OccupancyGrid>("/topology_map_filterd",5);
            pubOpeningList=nh.advertise<topology_mapping::opening_list>("/opening_list_int",5);
            loadMemory();
            initializeTopoMap();
        }
        
    //Load all maps, and map transforms into the memory
    void loadMemory(){

        generateMapTransform();

        //Creat all maps in memory
        ObjectFilterLookup=new bool*[mapSize];
        scanMap=new int*[mapSize];
        scanMapOut=new int*[mapSize];
        scanMapOutTransform=new int*[mapSize];
        Map=new int*[mapSize];
        topMap=new int*[mapSize];
        for(int i=0;i<mapSize;i++){
            ObjectFilterLookup[i]=new bool[mapSize];
            Map[i]=new int[mapSize];
            topMap[i]=new int[mapSize];
            scanMapOut[i]=new int[mapSize];
            scanMap[i]=new int[mapSize];
            scanMapOutTransform[i]=new int[mapSize];
        }
        //set in. val. for all maps as -1 
        for (int i = 0; i < mapSize; ++i){
            for (int j = 0; j < mapSize; ++j){
                Map[i][j] = -1;
                topMap[i][j] = -1;
                scanMapOut[i][j] = -1;
                scanMapOutTransform[i][j] = -1;
            }
        }
        //Create scanGarray, used for storing gaps in the gap analysis
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
            //for a cleaner output, fitToCorridor is used once more
            for(int j=0; j<oplist.size(); j++){
                if(oplist[j].label<10){
                    //fitToCorridor(&oplist[j],40,topMap,true, true);
                    if(dist(oplist[j].start,oplist[j].end)<minGroupSize){
                        oplist.erase(oplist.begin()+j);
                        j-=1;
                    }
                }
            }
            //remove openings tags for removal (openings with label>10)
            for(int j=0; j<oplist.size() && !show_removed_openings; j++){
                if(oplist[j].label>10){
                    oplist.erase(oplist.begin()+j);
                    j-=1;
                }
            } 
            //if there are any overlap missed or created since las cheek they are removed.
            for(int j=0; j<oplist.size(); j++){
                for(int b=0; b<oplist.size(); b++){
                    if(intersect_line(oplist[j],oplist[b],topMap) && oplist[b].label<10 && oplist[j].label<10 && j!=b){
                        //ROS_INFO("overlap");
                        if(dist(oplist[j].start,oplist[j].end)<dist(oplist[b].start,oplist[b].end)){
                            oplist[b].label=18;
                        }else{
                            oplist[j].label=18;
                        }
                    }
                    
                }
                
            }
            pubMap();
            ros::spinOnce();
            rate.sleep();
        }
    }

    //pre calculates the rotation transform for the map, this saves on performance.
    void generateMapTransform(){
        mapTransform= new point_int**[numberOfDir];

        for(int angle=0; angle<numberOfDir; angle++){
            mapTransform[angle]=new point_int*[mapSize];
            for(int i=0; i<mapSize;i++){
                mapTransform[angle][i]=new point_int[mapSize];
            }
            //using a 2d rotation matrix to rotate map around its center, original map is -1 extended
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

    //Get value at original map, at coordinates x and y. using transformation matrix with index angIndex
    int getMapTransform(int **map, const int x,const int y, const int angIndex){
        if(x<0||x>scanSize-1||y<0||y>scanSize-1) return -1;
        if(mapTransform[angIndex][x][y].x==-1) return -1;
        return getMap(mapTransform[angIndex][x][y].x,mapTransform[angIndex][x][y].y,map);
    }

    //set value at original map, at coordinates x and y. using transformation matrix with index angIndex
    void setMapTransform(int **map, const int x,const int y, const int angIndex, int value){
        if(x<0||x>scanSize-1||y<0||y>scanSize-1) return;
        if(mapTransform[angIndex][x][y].x==-1) return;
        setMap(mapTransform[angIndex][x][y].x,mapTransform[angIndex][x][y].y,value,map);
    }

    // initialization of map message
    void initializeTopoMap(){
        topoMapMsg.header.frame_id = "map";
        topoMapMsg.info.width = mapSize;
        topoMapMsg.info.height = mapSize;
        topoMapMsg.info.resolution = resolution;
        
        topoMapMsg.info.origin.orientation.x = 0.0;
        topoMapMsg.info.origin.orientation.y = 0.0;
        topoMapMsg.info.origin.orientation.z = 0.0;
        topoMapMsg.info.origin.orientation.w = 1.0;

        topoMapMsg.info.origin.position.x = mapOffsetX;
        topoMapMsg.info.origin.position.y = mapOffsetY;
        topoMapMsg.info.origin.position.z = mapHight+0.05; //for visualization

        topoMapMsg.data.resize(topoMapMsg.info.width * topoMapMsg.info.height);
        std::fill(topoMapMsg.data.begin(), topoMapMsg.data.end(), -1);
    }

    //Get new occupancy map and move its value into Map.
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

    //Ensures that the end and start point is next to a wall.
    bool correctOpening(opening *op, int maxMlenght){
        point_int *p1;
        point_int *p2;
        int mlenght=0;
        for(int k=0; k<2; k++){
            if(k==0){
                p1=&op->start;
                p2=&op->end;
            }else{
                p1=&op->end;
                p2=&op->start;
            }
            int tIndex=0;

            //move points outside a wall 
            while (getMap(p1->x,p1->y,topMap)!=0|| getMap(p1->x+1,p1->y,topMap)!=0 && getMap(p1->x-1,p1->y,topMap)!=0 &&
                    getMap(p1->x,p1->y+1,topMap)!=0 && getMap(p1->x,p1->y-1,topMap)!=0){
                if(abs(p2->x-p1->x)>abs(p2->y-p1->y)&&abs(p2->x-p1->x)>2){
                    p1->x+=(p2->x-p1->x)<0?-1:1;
                }
                else if(abs(p2->y-p1->y)>2){
                    p1->y+=(p2->y-p1->y)<0?-1:1;
                }else{
                    break;
                }
            }

            //moves point so they are next to a wall 
            int dirX=0;
            int dirY=1;
            int lenght=0;
            while(getMap(p1->x+dirX*(lenght+1),p1->y+dirY*(lenght+1),topMap)==0){
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

            if(mlenght<lenght) mlenght=lenght;
            
        }
        return mlenght<maxMlenght;
    }

    //Find all openings detection occupying the same opening, then removing all except the most fitnign opening detection. Return false if o should be deleted.
    bool cleanOpenings(opening op){
        vector<int> vo;
        vector<int> vint;
        bool selfDel=true;
        //Follow the wall at the start and end points of op in cw and ccw direction.
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
                //will follow the wall for at least searchLenghtClean steps
                for(int s=0; s<searchLenghtClean && empty_count<maxAntGap; s++){
                    
                    //Search will stop once a opening facing the other direction is found, 
                    //this is to stop cleaning of opening detection that does not belong to the current opening
                    if(check_for_opening(step.end,sids==0?2:1))break;

                    if(step.emty_cell){
                        empty_count+=1;
                    }else if(empty_count>0){
                        empty_count-=1;
                    }
                    
                    if(sids==0){
                        int opIndex=check_and_get_opening(step.end,1);
                        if(opIndex!=-1){
                            vo.push_back(opIndex);
                            int sepL=cw==1?s:-s;
                            vint.push_back(sepL);
                        }
                    }else{
                        int opIndex=check_and_get_opening(step.end,2);
                        if(opIndex!=-1){
                            //check if opening was found when the other side was searched
                            for(int index=0; index<vo.size(); index++){
                                if(vo[index]==opIndex){
                                    vint[index]+=cw==0?s:-s;
                                    //remove the last fitting opening
                                    if(dist(op.start,op.end)+vint[index]/extendDevider<dist(oplist[opIndex].start,oplist[opIndex].end)){
                                        oplist[opIndex].label=14;
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
        //set all valus of topMap to -1
        for(int x=0; x<mapSize;x++){
            for(int y=0;y<mapSize;y++){
                topMap[x][y]=-1;
                ObjectFilterLookup[x][y]=false;
            }
        }
        for(int angle=0; angle<numberOfDir; angle++){
            //check if this loop cycle should be filtered
            bool filter=numberOfDir%((int)(numberOfDir/numberOfDirFilter))==0;
            
            //Clear scanMapOutput from previus scan
            for (int i = 0; i < scanSize; ++i){
                scanGroupIndex[angle][i]=0;
            }

            int cfilter=0;
            //Loop thru all map cells
            for(int i=0;i<scanSize;i++){
                for(int j=1;j<scanSize-1;j++){

                    //Finde groups
                    if(getMapTransform(Map,i,j,angle)==0){
                        //check if space is free
                        if(cGroupeSize==0){
                            scanGarray[angle][i][scanGroupIndex[angle][i]].start=j;
                        }
                        cGroupeSize+=1;
                        cfilter=0;
                    //filter out small point obstacles
                    }else if (getMapTransform(Map,i,j,angle)==-1 && cfilter<cfilterSize && cGroupeSize!=0){
                        cfilter+=1;
                    
                    //if findeing ostacals biger then filter end serche
                    }else if(cGroupeSize!=0){
                        int endpoint=j-1-cfilter;
                        //if found groupe is larger then minGroupSize add it as gap
                        if(cGroupeSize>minGroupSize){
                            scanGarray[angle][i][scanGroupIndex[angle][i]].end=endpoint;
                            scanGarray[angle][i][scanGroupIndex[angle][i]].prevGroupIndex=0;
                            scanGarray[angle][i][scanGroupIndex[angle][i]].prevGroup[0]=NULL;
                            scanGarray[angle][i][scanGroupIndex[angle][i]].nextGroupIndex=0;
                            scanGarray[angle][i][scanGroupIndex[angle][i]].nextGroup[0]=NULL;
                            if(filter){
                                //save filtered values
                                for(int m=scanGarray[angle][i][scanGroupIndex[angle][i]].start;
                                    m<endpoint; m++){
                                        setMapTransform(scanMapOut,i,m,angle,0);
                                }
                            }
                            scanGroupIndex[angle][i]+=1;

                        }else if(cGroupeSize>0 && filter){
                            //save filtered values, pads value from the gaps left side  
                            for(int m=scanGarray[angle][i][scanGroupIndex[angle][i]].start;
                                m<j-cfilter; m++){
                                    setMapTransform(scanMapOut,i,m,angle,getMapTransform(Map,i,scanGarray[angle][i][scanGroupIndex[angle][i]].start-1,angle));
                            }
                        }
                        cfilter=0;
                        cGroupeSize=0;
                    }
                }

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
        //merge Map into topMap
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
                        //If the gap has less than two connections then skip this gap.
                        if(scanGarray[angle][i][index].prevGroupIndex<2 &&direction==0||
                           scanGarray[angle][i][index].nextGroupIndex<2 &&direction==1) continue;
                        
                        //Count depth of original gap
                        scanGroup *p=&scanGarray[angle][i][index];
                        int depthCount=0;
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

                        //check the depth of each gap connected to original gap
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
                                    //move the start and end pos in gaps to fit the filterd map
                                    int newStart, newEnd, S, E;
                                    if(direction==0){
                                        newStart=scanGarray[angle][i][index].prevGroup[h]->start;
                                        S=scanGarray[angle][i][index].prevGroup[h]->start;
                                        newEnd=scanGarray[angle][i][index].prevGroup[h]->end;
                                        E=scanGarray[angle][i][index].prevGroup[h]->end;
                                    }else{
                                        newStart=scanGarray[angle][i][index].nextGroup[h]->start;
                                        S=scanGarray[angle][i][index].nextGroup[h]->start;
                                        newEnd=scanGarray[angle][i][index].nextGroup[h]->end;
                                        E=scanGarray[angle][i][index].nextGroup[h]->end;
                                    }
                                    int bigestLenght=-1;
                                    int firstP=-1;
                                    for(int y=S;y<=E; y++){
                                        if(getMapTransform(topMap,i,y,angle)==0 && firstP==-1){
                                            firstP=y;
                                        }
                                        if(getMapTransform(topMap,i,y,angle)!=0 && firstP!=-1){
                                            if(y-1-firstP>bigestLenght){
                                                newStart=firstP;
                                                newEnd=y-1;
                                                firstP=-1;
                                            }
                                        }
                                    }

                                    opening newOp;
                                    if(direction==0){
                                        newOp.start={i,newEnd};
                                        newOp.end={i,newStart};
                                    }else{
                                        newOp.start={i,newStart};
                                        newOp.end={i,newEnd};
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
                        //If less then 2 connected gaps with a gap depth less than minCoridorSize, skip 
                        if(newOpList.size()<2) continue;
                        
                        //set the start_is_outside value, which is used by fitToCorridor() to determine which side of the opening it can move into the original gap
                        if(direction==0){
                            newOpList[0].start_is_outside=false;
                            newOpList[newOpList.size()-1].start_is_outside=true;
                        }else{
                            newOpList[0].start_is_outside=true;
                            newOpList[newOpList.size()-1].start_is_outside=false;
                        }
                        
                        int inSearchLenght=searchLenght;

                        //rotate point back to original rotation
                        newOpList=rotate_points(newOpList,rotation);


                        for(int k=0; k<newOpList.size();k++){
                            opening o=newOpList[k];

                            if(!correctOpening(&o,10)) continue;
                            //remove small objects in the map 
                            for(int sids=0; sids<2;sids++){
                                ant_data step;
                                step.end=sids==0?o.end:o.start;
                                vector<point_int> pointList;
                                for(int s=0;s<=objectFilterMaxStep;s++){
                                    step=ant_step(step.end,false,step.dir,topMap);
                                    if(ObjectFilterLookup[step.end.x][step.end.y]){
                                        break;
                                    }
                                    if(pointList.size()>0){
                                        if(step.end==pointList[0]){
                                            fillPoly(pointList,0,topMap);
                                            /*for(int i1=0;i1<PL.size();i1++){
                                                point_int nextDir={PL[(i1+1)%PL.size()].x-PL[i1].x,
                                                                    PL[(i1+1)%PL.size()].y-PL[i1].y};
                                                int dirTest=2*nextDir.y+nextDir.x;
                                                int i3=-1;
                                                for(int i2=i1+1;i2<PL.size();i2++){
                                                    if(PL[i1].y!=PL[i2].y) continue;
                                                    if(PL[i2].x-PL[i1].x<=0 && dirTest<0 ||
                                                        PL[i2].x-PL[i1].x>=0 && dirTest>0 ) continue;
                                                    if(i3==-1){
                                                        i3=i2;
                                                    }else{
                                                        if(std::abs(PL[i1].x-PL[i2].x)<std::abs(PL[i1].x-PL[i3].x)){
                                                            i3=i2;
                                                        }
                                                    }
                                                }
                                                if(i3==-1) continue;

                                                for(int x=std::min(PL[i1].x,PL[i3].x);
                                                    x<=std::max(PL[i1].x,PL[i3].x);x++){
                                                        setMap(x,PL[i1].y,0,topMap);
                                                }
                                                
                                            }*/
                                            
                                            point_int pEnd=o.end, pStart=o.start;
                                            correctOpening(&o,10);
                                            if(o.start==pStart && o.end==pEnd){
                                                for(int m=0;m<pointList.size();m++){
                                                    ObjectFilterLookup[pointList[m].x][pointList[m].y]=true;
                                                }
                                                break;
                                            }
                                            sids-=1;
                                            break;
                                        }
                                    }
                                    if(s==objectFilterMaxStep){
                                        for(int m=0;m<pointList.size();m++){
                                            ObjectFilterLookup[pointList[m].x][pointList[m].y]=true;
                                        }
                                    }
                                    pointList.push_back(step.end);
                                }
                                
                            }
                            if(!fitToCorridor(&o,inSearchLenght,topMap)) continue;

                            //remove too small openings
                            double opLenght=dist(o.start,o.end);
                            if(opLenght<minGroupSize){
                                continue;
                            }
                            
                            //Move openings point such that they don't overlap another openings points
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

                        //Check if opening is overlapping another opening.
                        for(int h=0; h<oplist.size() && !skip && o.label<10; h++){
                                opening oplistComp=oplist[h];
                                if(intersect_line(o,oplistComp,topMap) && oplistComp.label<10 && o.label<10){

                                    bool conected[]={false, false};
                                    point_int conect[2];
                                    int conection_lenght[]={0,0};//direction and distans to conection
                                    int conection_waite=-1;
                                    point_int conection_dir[2];
                                    bool conection_cw[2];
                                    ant_data step_info;
                                    //search both sides of o for the overlapping opening detection
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
                                                }else if(emty_count>0){
                                                    emty_count-=1;
                                                }
                                            }
                                        }
                                    }
                                    
                                    int sides=1;
                                    //If overlapping opening doesnt share a wall with o then keep the shortest opening. 
                                    if(!conected[0] && !conected[1]){
                                        if(dist(o.start,o.end)<dist(oplistComp.start,oplistComp.end)){
                                            oplist[h].label=16;
                                            break;
                                        }else{
                                            skip=true;
                                            break;
                                        }
                                    //If openings share one or two common wall check which side should be moved.
                                    }else if(conection_lenght[0]<conection_lenght[1] && conected[0] || !conected[1]){
                                        sides=0;
                                    }
                                    int c=0;
                                    step_info.end=conect[sides];
                                    step_info.dir=conection_dir[sides];
                                    //Move one side of o until there is no overlap. 
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
                                    //Move or stash it start and end does not overlap with an other opening. 
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
                                    //Check if the issue was sold otherwise delete the longest opening.
                                    if(intersect_line(o,oplistComp,topMap)){
                                        if(dist(o.start,o.end)<dist(oplistComp.start,oplistComp.end)){
                                            oplist[h].label=16;
                                            h-=1;
                                            break;
                                        }else{
                                            skip=true;
                                            break;
                                        }
                                    }
                                }
                            }

                            if(skip)o.label=16;//Removed because overlap couldn't be solved

                            if(dist(o.start,o.end)<minGroupSize){
                                continue;
                            }
                            
                            if(checkForWall(o, 1,topMap))o.label=12;

                            if(o.label<10){
                                if(!cleanOpenings(o)){
                                    o.label=14;
                                }
                            }

                            oplist.push_back(o);
                        } 
                    }
                }
            }
        }  
    }

    //Function to publish all topics. 
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

    ros::init(argc, argv, "topology_gap_analysis");
    
    TopologyMapping topMapping;

    ROS_INFO("Topology Gap Analysis Started.");
    
    topMapping.spin();
    
    return 0;
}