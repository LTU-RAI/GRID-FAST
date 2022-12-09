#include "Utility.h"                                

class TopologyMapping{
    private:
        //ROS nh
        ros::NodeHandle nh;
        //subs
        ros::Subscriber subOccupancyMap;
        //pub
        ros::Publisher pubTopoMap;
        ros::Publisher pubdMap;
        ros::Publisher pubOpeningList;

        //msg
        nav_msgs::OccupancyGrid topoMapMsg;
        nav_msgs::OccupancyGrid topoMapMsgD;
        point_int sizeD;
        //Maps
        int **Map;
        int **topMap;

        //global var
        bool **ObjectFilterLookup;
        int **scanMap;
        int **scanMapOut;
        int **scanMapOutTransform;
        vector<vector<vector<scanGroup>>> scanGarray;
        vector<vector<vector<mapTransform>>> mapTransformList;
        


    public:
        //setup
        TopologyMapping(){
            subOccupancyMap= nh.subscribe("/map",1,&TopologyMapping::updateMap, this);
            pubTopoMap=nh.advertise<nav_msgs::OccupancyGrid>("/topology_map_filterd",5);
            pubdMap=nh.advertise<nav_msgs::OccupancyGrid>("/topology_map_d",5);
            pubOpeningList=nh.advertise<topology_mapping::opening_list>("/opening_list_int",5);
            loadMemory();
        }
        
    //Load all maps, and map transforms into the memory
    void loadMemory(){
        
        //Creat all maps in memory
        ObjectFilterLookup=new bool*[mapSizeX];
        scanMap=new int*[mapSizeX];
        scanMapOut=new int*[mapSizeX];
        scanMapOutTransform=new int*[mapSizeX];
        Map=new int*[mapSizeX];
        dMap=new int*[mapSizeX];
        topMap=new int*[mapSizeX];
        for(int i=0;i<mapSizeX;i++){
            ObjectFilterLookup[i]=new bool[mapSizeY];
            Map[i]=new int[mapSizeY];
            dMap[i]=new int[mapSizeY];
            topMap[i]=new int[mapSizeY];
            scanMapOut[i]=new int[mapSizeY];
            scanMap[i]=new int[mapSizeY];
            scanMapOutTransform[i]=new int[mapSizeY];
        }
        //set in. val. for all maps as -1 
        for (int i = 0; i < mapSizeX; ++i){
            for (int j = 0; j < mapSizeY; ++j){
                Map[i][j] = -1;
                dMap[i][j] = -1;
                topMap[i][j] = -1;
                scanMapOut[i][j] = -1;
                scanMapOutTransform[i][j] = -1;
            }
        }
        //Create scanGarray, used for storing gaps in the gap analysis
        scanGarray.resize(numberOfDir);
        generateMapTransform();
        initializeTopoMap();
    }
    void unloadMemory(){

        mapTransformList.clear();

        //Creat all maps in memory
        for(int i=0;i<mapSizeX;i++){
            delete[] ObjectFilterLookup[i];
            delete[] Map[i];
            delete[] topMap[i];
            delete[] scanMapOut[i];
            delete[] scanMap[i];
            delete[] scanMapOutTransform[i];
        }
        delete[] ObjectFilterLookup;
        delete[] scanMap;
        delete[] scanMapOut;
        delete[] scanMapOutTransform;
        delete[] Map;
        delete[] topMap;
        
        //Create scanGarray, used for storing gaps in the gap analysis
        scanGarray.clear();
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
                    if(intersect_line(&oplist[j],&oplist[b]) && oplist[b].label<10 && oplist[j].label<10 && j!=b){
                        fixOverlap(&oplist[j],&oplist[b],searchLenght);
                    }
                }
            }
            for(int j=0; j<oplist.size(); j++){
                for(int b=0; b<oplist.size(); b++){
                    if(intersect_line(&oplist[j],&oplist[b]) && oplist[b].label<10 && oplist[j].label<10 && j!=b){
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
        mapTransformList.resize(numberOfDir);

        for(int angle=0; angle<numberOfDir; angle++){
            //using a 2d rotation matrix to rotate map around its center, original map is -1 extended
            float rotation=M_PI*angle/numberOfDir;
            float cosA=cos(-rotation);
            float sinA=sin(-rotation);
            point_int d1,d2,d1x,d2x;
            for(int x=0;x<mapSizeX;x+=mapSizeX-1){
                for(int y=0;y<mapSizeY;y+=mapSizeY-1){
                    int newX=int(std::round(((x-mapSizeX/2)*cosA-(y-mapSizeY/2)*sinA)+mapSizeX/2));
                    int newY=int(std::round(((x-mapSizeX/2)*sinA+(y-mapSizeY/2)*cosA)+mapSizeY/2));
                    if(x==0 && y==0){
                        d1.x=newY;
                        d1x.x=newX;
                    } 
                    if(x==mapSizeX-1 && y==mapSizeY-1){
                        d1.y=newY;
                        d1x.y=newX;
                    } 

                    if(x==mapSizeX-1 && y==0){
                        d2.x=newY;
                        d2x.x=newX;
                        }
                    if(x==0 && y==mapSizeY-1){
                        d2.y=newY;
                        d2x.y=newX;
                    }
                }
            }
            int ySize,xSize,yoffset,xoffset;
            if(std::abs(d2.x-d2.y)<std::abs(d1.x-d1.y)){
                ySize=std::abs(d1.x-d1.y)+1;
                xSize=std::abs(d2x.x-d2x.y)+1;
            }else{
                ySize=std::abs(d2.x-d2.y)+1;
                xSize=std::abs(d1x.x-d1x.y)+1;
            }
            vector<vector<mapTransform>> tMap;
            tMap.resize(ySize);
            cosA=cos(rotation);
            sinA=sin(rotation);
            for(int y=0;y<ySize;y++){
                for(int x=0;x<xSize;x++){
                    int newX=int(std::round(((x-xSize/2)*cosA-(y-ySize/2)*sinA)+xSize/2));
                    int newY=int(std::round(((x-xSize/2)*sinA+(y-ySize/2)*cosA)+ySize/2));
                    if(newX<0 || newX>=mapSizeX) continue;
                    if(newY<0 || newY>=mapSizeY) continue;
                    mapTransform T;
                    T.rpos.x=newX;
                    T.rpos.y=newY;
                    T.tpos.x=x;
                    T.tpos.y=y;
                    tMap[y].push_back(T);
                }
            }
            mapTransformList[angle]=tMap;
        }
    }
    
    //Get value at original map, at coordinates x and y. using transformation matrix with index angIndex
    int getMapTransform(int **map, const int x,const int y, const int angIndex){
        int indexY=y;
        if(indexY<0||indexY>=mapTransformList[angIndex].size()) return -1;
        int indexX=x-mapTransformList[angIndex][indexY][0].tpos.x;
        if(indexX<0||indexX>=mapTransformList[angIndex][indexY].size()) return -1;
        return getMap(mapTransformList[angIndex][indexY][indexX].rpos.x,mapTransformList[angIndex][indexY][indexX].rpos.y,map);
    }
    //Givs index to the orignal map
    point_int getMapIndexTransform(const int x,const int y, const int angIndex){
        point_int mIndex={-1,-1};
        int indexY=y;
        if(indexY<0||indexY>=mapTransformList[angIndex].size()) return mIndex;
        int indexX=x-mapTransformList[angIndex][indexY][0].tpos.x;
        if(indexX<0||indexX>=mapTransformList[angIndex][indexY].size()) return mIndex;
        mIndex={mapTransformList[angIndex][indexY][indexX].rpos.x,mapTransformList[angIndex][indexY][indexX].rpos.y};
        return mIndex;
    }
    
    //set value at original map, at coordinates x and y. using transformation matrix with index angIndex
    void setMapTransform(int **map, const int x,const int y, const int angIndex, int value){
        int indexY=y;
        if(indexY<0||indexY>=mapTransformList[angIndex].size()) return;
        int indexX=x-mapTransformList[angIndex][indexY][0].tpos.x;
        if(indexX<0||indexX>=mapTransformList[angIndex][indexY].size()) return;
        setMap(mapTransformList[angIndex][indexY][indexX].rpos.x,mapTransformList[angIndex][indexY][indexX].rpos.y,value,map);
    }
    //rotate all points in op list around the maps center
    vector<opening> rotate_points(vector<opening> op, const int angIndex){
        vector<opening> newOp;
        newOp.resize(op.size());
        for(int i=0; i<op.size();i++){
            newOp[i]=op[i];
            for(int sids=0; sids<2;sids++){
                point_int p=op[i].start;
                if(sids==1){
                    p=op[i].end;
                }
                int indexY=p.y;
                int indexX=p.x-mapTransformList[angIndex][indexY][0].tpos.x;
                
                p=mapTransformList[angIndex][indexY][indexX].rpos;
                if(sids==0){
                    newOp[i].start=p;
                }else{
                    newOp[i].end=p;
                }
            }
        }
        return newOp;        
    }
    // initialization of map message
    void initializeTopoMap(){
        topoMapMsg.header.frame_id = "map";
        topoMapMsg.info.width = mapSizeX;
        topoMapMsg.info.height = mapSizeY;
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
        resolution=mapMsg.info.resolution;
        mapOffsetX=mapMsg.info.origin.position.x;
        mapOffsetY=mapMsg.info.origin.position.y;
        mapHight=mapMsg.info.origin.position.z;
        initializeTopoMap();
        if(width!=mapSizeX || height!=mapSizeY){
            ROS_INFO("Detected a change in map size reinitializes!");
            unloadMemory();
            mapSizeX=width;
            mapSizeY=height;
            loadMemory();
        }
        
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
            while (getMap(p1->x,p1->y,topMap)!=0||getMap(p1->x+1,p1->y,topMap)!=0 && getMap(p1->x-1,p1->y,topMap)!=0 &&
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
            while (getMap(p1->x+1,p1->y,topMap)==0 && getMap(p1->x-1,p1->y,topMap)==0 &&
                    getMap(p1->x,p1->y+1,topMap)==0 && getMap(p1->x,p1->y-1,topMap)==0){
                if(abs(p2->x-p1->x)>abs(p2->y-p1->y)&&abs(p2->x-p1->x)>2){
                    p1->x-=(p2->x-p1->x)<0?-1:1;
                }
                else if(abs(p2->y-p1->y)>2){
                    p1->y-=(p2->y-p1->y)<0?-1:1;
                }else{
                    break;
                }
            }
        }
        return true;
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
    //finging the overlap between o1 and o2 by moving o1, returns true if sucsesful
    void fixOverlap(opening *o1,opening *o2, int inSearchLenght){
        ant_data conections[2];
        bool conection_cw[2];
        int conection_lenght[]={-1,-1};//direction and distans to conection
        for(int sides=0;sides<2;sides++){
            for(int dir=0;dir<2;dir++){
                ant_data step;
                step.end=sides?o1->end:o1->start;
                for(int s=0;s<sercheLenthAnt;s++){
                    if(step.end==o2->start || step.end==o2->end){
                        if(conection_lenght[sides]==-1 ||
                            s<conection_lenght[sides]){
                            conection_lenght[sides]=s;
                            conection_cw[sides]=dir;
                            conections[sides]=step;
                            break;
                        }
                    }
                    step=ant_step(step.end,dir,step.end,topMap);
                }
            }
        }
        //If overlapping opening doesnt share a wall with o then keep the shortest opening.
        if(conection_lenght[0]==-1 && conection_lenght[1]==-1){
            if(dist(o1->start,o1->end)<dist(o2->start,o2->end)){
                o2->label=16;
            }else{
                o1->label=16;
            }
            return;
        }
        //If openings share one or two common wall check which side should be moved.
        int sides=1;
        if(conection_lenght[1]==-1 || 
            conection_lenght[0]!=-1 && conection_lenght[0]<conection_lenght[1]){
            sides=0;
        }
        int c=0;
        ant_data step=conections[sides];
        while(intersect_line(o1,o2) && c<inSearchLenght){
            c+=1;
            opening test=*o1;
            step=ant_step(step.end,conection_cw[sides],step.dir,topMap);
            if(!sides){
                test.start=step.end;
            }else{
                test.end=step.end;
            }
            if(!checkForWall(test,1,topMap)) *o1=test;
        } 
        c=0;
        step=conections[sides];
        while(intersect_line(o1,o2) && c<inSearchLenght){
            c+=1;
            opening test=*o1;
            step=ant_step(step.end,!conection_cw[sides],step.dir,topMap);
            if(!sides){
                test.start=step.end;
            }else{
                test.end=step.end;
            }
            if(!checkForWall(test,1,topMap)) *o1=test;
        }
        //Check if the issue was sold otherwise delete the longest opening.
        if(intersect_line(o1,o2)){
            if(dist(o1->start,o1->end)<dist(o2->start,o2->end)){
                o2->label=16;
                
            }else{
                o1->label=16;
            }
        }
        return;
    }

    //Fitts gaps to map, remove overlaping gaps 
    int fitGapToMap(int angelindex, int row, int gIndex, bool nextGroupe){
        vector<scanGroup*> gapG;
        if(nextGroupe){
            gapG= scanGarray[angelindex][row][gIndex].nextGroup;
        }else{
            gapG= scanGarray[angelindex][row][gIndex].prevGroup;
        }
        for(int i=0;i<gapG.size();i++){
            bool stopL=false;
            for(int d=0;d<2;d++){
                int pos=d?gapG[i]->end:gapG[i]->start;
                if(getMapTransform(topMap,pos,gapG[i]->row,angelindex)==0){
                    while(true){
                        if(getMapTransform(topMap,pos-1+2*d,gapG[i]->row,angelindex)==0){
                            pos=pos-1+2*d;
                        }else break;
                    }
                }else{
                    while(true){
                        pos=pos+1-2*d;
                        if(getMapTransform(topMap,pos,gapG[i]->row,angelindex)==0) break;
                        if(!d && pos>=gapG[i]->end || d && pos<=gapG[i]->start){
                            gapG.erase(gapG.begin()+i);
                            stopL=true;
                            i--;
                            break;
                        }
                    }
                }
                if(stopL) break;
                if(d){
                    gapG[i]->end=pos;
                }else{
                    gapG[i]->start=pos;
                }
            }
            if(stopL) continue;
            for(int i2=i+1;i2<gapG.size();){
                if(gapG[i]->end>gapG[i2]->start){
                    gapG.erase(gapG.begin()+i2);
                }else break;
            }
        }
        if(nextGroupe){
            scanGarray[angelindex][row][gIndex].nextGroup=gapG;
        }else{
            scanGarray[angelindex][row][gIndex].prevGroup=gapG;
        }
        return gapG.size();
    }

    void topologyScan(){
        //set all valus of topMap to -1
        for(int x=0; x<mapSizeX;x++){
            for(int y=0;y<mapSizeY;y++){
                topMap[x][y]=-1;
                dMap[x][y]=-1;
                ObjectFilterLookup[x][y]=false;
            }
        }
        int gSize=1;
        for(int angle=0; angle<numberOfDir; angle++){
            //check if this loop cycle should be filtered
            bool filter=numberOfDir%((int)(numberOfDir/numberOfDirFilter))==0;
            
            //Clear scanMapOutput from previus scan
            
            scanGarray[angle].clear();
            scanGarray[angle].resize(mapTransformList[angle].size()/gSize);

            
            //Loop thru all map cells
            for(int i=0;i<mapTransformList[angle].size()-1;i+=gSize){
                scanGroup sg;
                vector<scanGroup> sgList;
                int cfilter=0;
                int cGroupeSize=0;
                for(int j=0;j<mapTransformList[angle][i].size();j++){
                    mapTransform mt=mapTransformList[angle][i][j];
                    //Finde groups
                    if(Map[mt.rpos.x][mt.rpos.y]==0){ //&& getMapTransform(Map,mt.tpos.x,i+1,angle)==0){
                        //check if space is free
                        if(cGroupeSize==0){
                            sg.start=mt.tpos.x;
                        }
                        cGroupeSize+=1;
                        cfilter=0;
                    //filter out small point obstacles
                    }else if (Map[mt.rpos.x][mt.rpos.y]==-1 && cfilter<cfilterSize && cGroupeSize!=0){
                        cfilter+=1;
                    
                    //if findeing ostacals biger then filter end serche
                    }else if(cGroupeSize!=0){
                        int endpoint=mt.tpos.x-1-cfilter;
                        //if found groupe is larger then minGroupSize add it as gap
                        if(cGroupeSize>minGroupSize){
                            sg.end=endpoint;
                            sg.row=i;
                            sg.prevGroup.clear();
                            sg.nextGroup.clear();
                            if(filter){
                                //save filtered values
                                /*for(int m=sg.start;
                                    m<=endpoint; m++){
                                        setMapTransform(scanMapOut,m,i,angle,0);
                                }*/
                            }
                            sgList.push_back(sg);

                        }else if(cGroupeSize>0 && filter){
                            //save filtered values, pads value from the gaps left side  
                            int value=100;
                            if(getMapTransform(Map,sg.start-1,i,angle)==-1 && 
                                getMapTransform(Map,mt.tpos.x-cfilter,i,angle)==-1){
                                    value=-1;
                                }
                            for(int m=sg.start;
                                m<mt.tpos.x-cfilter; m++){
                                    setMapTransform(scanMapOut,m,i,angle,value);
                            }
                        }
                        cfilter=0;
                        cGroupeSize=0;
                    }
                }
                scanGarray[angle][i/gSize]=sgList;
                for(int index1=0;index1<scanGarray[angle][i/gSize].size() && i!=0;index1++){
                    //find if there is tow or more gropse conecteing to a previus group
                    for(int index2=0;index2<scanGarray[angle][i/gSize-1].size();index2++){
                        if(scanGarray[angle][i/gSize][index1].start<scanGarray[angle][i/gSize-1][index2].end &&
                            scanGarray[angle][i/gSize][index1].end>scanGarray[angle][i/gSize-1][index2].start){
                                scanGarray[angle][i/gSize][index1].prevGroup.push_back(&scanGarray[angle][i/gSize-1][index2]);

                                scanGarray[angle][i/gSize-1][index2].nextGroup.push_back(&scanGarray[angle][i/gSize][index1]);
                            }
                    }
                }
            }
            if(filter){              
                for(int x=0; x<mapSizeX;x++){
                    for(int y=0;y<mapSizeY;y++){
                        if(scanMapOut[x][y]!=-1){
                            topMap[x][y]=(scanMapOut[x][y]>=70 || topMap[x][y]>=70)?100:0;
                        }
                        scanMapOut[x][y]=-1;
                    }
                } 
            }  
        }
        //merge Map into topMap
        for(int x=0; x<mapSizeX;x++){
                for(int y=0;y<mapSizeY;y++){
                    if(topMap[x][y]!=-1 || Map[x][y]!=-1){
                        topMap[x][y]=(Map[x][y]>=70 || topMap[x][y]>=70)?100:0;
                    }
                    scanMapOutTransform[x][y]=-1;
                    scanMapOut[x][y]=-1;
                }
        }
        //Remove samll objects
        for(int angle=0; angle<numberOfDir; angle++){
            for(int i=1;i<scanGarray[angle].size();i++){
                for(int index= 0; index<scanGarray[angle][i].size();index++){
                    for(int direction=0;direction<2;direction++){
                        if(scanGarray[angle][i][index].prevGroup.size()<2 &&direction==0||
                           scanGarray[angle][i][index].nextGroup.size()<2 &&direction==1) continue;
                        fitGapToMap(angle,i,index,direction);
                        int gIndex=0;
                        while(true){
                            if(scanGarray[angle][i][index].prevGroup.size()<2 &&direction==0||
                            scanGarray[angle][i][index].nextGroup.size()<2 &&direction==1) break;
                            vector<scanGroup*> gapG;
                            if(direction){
                                gapG= scanGarray[angle][i][index].nextGroup;
                            }else{
                                gapG= scanGarray[angle][i][index].prevGroup;
                            }
                            ant_data step;
                            point_int startP=getMapIndexTransform(gapG[gIndex]->start,gapG[gIndex]->row,angle);
                            step.end=getMapIndexTransform(gapG[gIndex]->end,gapG[gIndex]->row,angle);
                            vector<point_int> pointList;
                            for(int s=0;s<=objectFilterMaxStep;s++){
                                step=ant_step(step.end,false,step.dir,topMap);
                                if(ObjectFilterLookup[step.end.x][step.end.y] || step.end==startP){
                                    gIndex++;
                                    break;
                                }
                                if(pointList.size()>0){
                                    if(step.end==pointList[0]){
                                        vector<point_int> filledPoints=fillPoly(pointList);
                                        for(int e=0;e<filledPoints.size();e++) 
                                            setMap(filledPoints[e].x,filledPoints[e].y,0,topMap);
                                        int pSize=gapG.size();
                                        if(fitGapToMap(angle,i,index,direction)==pSize){
                                            for(int m=0;m<pointList.size();m++){
                                                ObjectFilterLookup[pointList[m].x][pointList[m].y]=true;
                                            }
                                            gIndex++;
                                        }else gIndex=0;
                                        break;
                                    }
                                }
                                if(s==objectFilterMaxStep){
                                    for(int m=0;m<pointList.size();m++){
                                        ObjectFilterLookup[pointList[m].x][pointList[m].y]=true;
                                    }
                                    gIndex++;
                                    break;
                                }
                                pointList.push_back(step.end);
                            }
                            if(gIndex>=gapG.size()-1) break;     
                        }
                    }
                }
            }
        }
        for(int angle=0; angle<numberOfDir; angle++){
            for(int i=1;i<scanGarray[angle].size();i++){
                for(int index= 0; index<scanGarray[angle][i].size();index++){
                    for(int direction=0;direction<2;direction++){
                        //If the gap has less than two connections then skip this gap.
                        if(scanGarray[angle][i][index].prevGroup.size()<2 &&direction==0||
                           scanGarray[angle][i][index].nextGroup.size()<2 &&direction==1) continue;
                        
                        //Count depth of original gap
                        scanGroup *p=&scanGarray[angle][i][index];
                        int depthCount=0;
                        while (p->nextGroup.size()==1&&direction==0||p->prevGroup.size()==1&&direction==1){
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
                        int loopAmount=scanGarray[angle][i][index].prevGroup.size();
                        if(direction==1){
                            loopAmount=scanGarray[angle][i][index].nextGroup.size();
                        }
                        for(int h=0;h<loopAmount;h++){
                            depthCount=0;
                            int endIndex=0;
                            if(direction==0){
                                p=scanGarray[angle][i][index].prevGroup[h];
                                endIndex=scanGarray[angle][i][index].prevGroup.size()-1;
                            }else{
                                p=scanGarray[angle][i][index].nextGroup[h];
                                endIndex=scanGarray[angle][i][index].nextGroup.size()-1;
                            }

                            while (true){
                                depthCount+=1;
                                if(depthCount==minCoridorSize){
                                    int newStart, newEnd, row; 
                                    if(direction==0){
                                        newStart=scanGarray[angle][i][index].prevGroup[h]->start;
                                        newEnd=scanGarray[angle][i][index].prevGroup[h]->end;
                                        row=scanGarray[angle][i][index].prevGroup[h]->row;
                                    }else{
                                        newStart=scanGarray[angle][i][index].nextGroup[h]->start;
                                        newEnd=scanGarray[angle][i][index].nextGroup[h]->end;
                                        row=scanGarray[angle][i][index].nextGroup[h]->row;
                                    }

                                    opening newOp;
                                    if(direction==1){
                                        newOp.start={newEnd,row};
                                        newOp.end={newStart,row};
                                        if(h==0){
                                            newOp.sideToMove=2;
                                        }else if(h==endIndex){
                                            newOp.sideToMove=1;
                                        }
                                    }else{
                                        newOp.start={newStart,row};
                                        newOp.end={newEnd,row};
                                        if(h==0){
                                            newOp.sideToMove=1;
                                        }else if(h==endIndex){
                                            newOp.sideToMove=2;
                                        }
                                    }
                                    
                                    newOpList.push_back(newOp);
                                    break;
                                }
                                if(direction==0){
                                    if(p->prevGroup.size()==0) break;
                                    p=p->prevGroup[0];
                                }else{
                                    if(p->nextGroup.size()==0) break;
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
                        newOpList=rotate_points(newOpList,angle);
                        for(int k=0; k<newOpList.size();k++){
                            opening o=newOpList[k];
                            correctOpening(&o,10);
                            if(!fitToCorridor(&o,inSearchLenght,topMap)) continue;
                            //fitToCorridor(&o,200,angle);
                            //correctOpening(&o,10);
                            moveOpeningIntoCoridor(&o,topMap);
                            //remove too small openings
                            double opLenght=dist(o.start,o.end);
                            if(opLenght<minGroupSize){
                                continue;
                            }
                            //Move openings point such that they don't overlap another openings points
                            for(int sids=0; sids<2 && false;sids++){
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
                        vector<int> dIndex;
                        for(int h=0; h<oplist.size() && o.label<10;h++){
                            if(oplist[h].label>10 || !intersect_line(&o,&oplist[h])) continue;
                            fixOverlap(&o,&oplist[h],inSearchLenght);
                            if(oplist[h].label>10){
                                dIndex.push_back(h);
                            }else if(o.label>10){
                                for(int hI=0;hI<dIndex.size();hI++){
                                    oplist[dIndex[hI]].label=1;
                                }
                            }
                        }
                        /*for(int h=0; h<oplist.size() && !skip && o.label<10; h++){
                                opening oplistComp=oplist[h];
                                if(intersect_line(&o,&oplist[h]) && oplistComp.label<10 && o.label<10){

                                    bool conected[]={false, false};
                                    point_int conect[2];
                                    int conection_lenght[]={0,0};//direction and distans to conection
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
                                                        conect[sides]=oplistComp.end;
                                                        conected[sides]=true;
                                                        conection_lenght[sides]=s;
                                                        conection_dir[sides].x=step_info.dir.x;
                                                        conection_dir[sides].y=step_info.dir.y;
                                                        conection_cw[sides]=clockwise;
                                                        break;

                                                }else if(step_info.end==oplistComp.start){
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
                                        opening *spliting;
                                        opening *keeping;
                                        if(dist(o.start,o.end)<dist(oplistComp.start,oplistComp.end)){
                                            keeping=&oplist[h];
                                            spliting=&o;
                                            oplist[h].label=16;
                                        }else{
                                            spliting=&oplist[h];
                                            keeping=&o;
                                            skip=true;
                                        }
                                        point_int anker;
                                        if(dist(spliting->get_center(),keeping->start)<
                                           dist(spliting->get_center(),keeping->end)){
                                            anker=keeping->start;
                                        }else anker=keeping->end;
                                        point_int dir={0,0};
                                        anker=ant_step(anker,true,dir,topMap).end;
                                        if(dist(anker,spliting->start)>=minGroupSize){
                                            opening nO;
                                            nO.start=spliting->start;
                                            nO.end=anker;
                                            //oplist.push_back(nO);
                                        }
                                        if(dist(anker,spliting->end)>=minGroupSize){
                                            opening nO;
                                            nO.start=anker;
                                            nO.end=spliting->end;
                                            //oplist.push_back(nO);
                                        }
                                        break;
                                    }
                                    //If openings share one or two common wall check which side should be moved.
                                    else if(conection_lenght[0]<conection_lenght[1] && conected[0] || !conected[1]){
                                        sides=0;
                                    }
                                    int c=0;
                                    step_info.end=conect[sides];
                                    step_info.dir=conection_dir[sides];
                                    //Move one side of o until there is no overlap. 
                                    while(intersect_line(&oplist[h],&o) && c<inSearchLenght){
                                        c+=1;
                                        step_info=ant_step(step_info.end,conection_cw[sides],step_info.dir,topMap);
                                        if(sides==0){
                                            o.start.x=step_info.end.x;
                                            o.start.y=step_info.end.y;
                                        }else{
                                            o.end.x=step_info.end.x;
                                            o.end.y=step_info.end.y;
                                        }
                                        moveOpeningIntoCoridor(&o,topMap);
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
                                    if(intersect_line(&oplist[h],&o)){
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
                            }*/
                            //if(skip)o.label=16;//Removed because overlap couldn't be solved

                            if(dist(o.start,o.end)<minGroupSize){
                                continue;
                            }
                            moveOpeningIntoCoridor(&o,topMap);
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
        for(int y=0; y<mapSizeY;y++){
            for(int x=0;x<mapSizeX;x++){  
                int index=x+y*mapSizeX;            
                topoMapMsg.data[index]=topMap[x][y];
            }
        }
        for(int oi=0;oi<oplist.size();oi++){
            for(int i=0;i<oplist[oi].occupied_points.size();i++){
                dMap[oplist[oi].occupied_points[i].x][oplist[oi].occupied_points[i].y]=100;
            }
            dMap[oplist[oi].start.x][oplist[oi].start.y]=0;
            dMap[oplist[oi].end.x][oplist[oi].end.y]=50;
        }
        pubTopoMap.publish(topoMapMsg);
        for(int y=0; y<mapSizeY;y++){
            for(int x=0;x<mapSizeX;x++){  
                int index=x+y*mapSizeX;            
                topoMapMsg.data[index]=dMap[x][y];
            }
        }
        pubdMap.publish(topoMapMsg);
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "topology_gap_analysis");
    
    TopologyMapping topMapping;

    ROS_INFO("Topology Gap Analysis Started.");
    
    topMapping.spin();
    
    return 0;
}