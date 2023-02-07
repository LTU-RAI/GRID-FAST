#include "Utility.hh"      
#include "MapHandler.hh"
#include "MapTransform.hh"
#include "GapHandler.hh"
#include "MapFilter.hh"                          

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
        vector<vector<vector<scanGroup>>> travGaps;
        vector<vector<vector<scanGroup>>> nonTravGaps;
        vector<mapTransformMap> mapTransformList;

        MapHandler map, FilteredMap;
        MapTransform transform;
        GapHandler gaps;
        MapFilter filter;
        


    public:
        //setup
        TopologyMapping(){
            std::string config_file_path= ros::package::getPath("topology_mapping")+"/config/settings.conf";
            load_config_file(config_file_path);
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
        //Create travGaps, used for storing gaps in the gap analysis
        travGaps.resize(numberOfDir);
        nonTravGaps.resize(numberOfDir);
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
        
        //Create travGaps, used for storing gaps in the gap analysis
        travGaps.clear();
    }

    void spin(){
        ros::Rate rate(100); // Hz
        
        while (ros::ok()){
            /*ROS_INFO("g1");
            transform.updateTransform(&map);
            ROS_INFO("g2");
            gaps.analysis(&map,&transform);
            ROS_INFO("g3");
            FilteredMap.updateMap(&map);
            ROS_INFO("g4");
            filter.filterMap(&FilteredMap,&transform,&gaps);
            ROS_INFO("g5");*/

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
            for(int j=0; j<oplist.size() && false; j++){
                for(int b=0; b<oplist.size(); b++){
                    if(intersect_line(&oplist[j],&oplist[b]) && oplist[b].label<10 && oplist[j].label<10 && j!=b){
                        fixOverlap(&oplist[j],&oplist[b],searchLenghtFixOverlap);
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
            mapTransformMap tMap;
            tMap.resize(ySize);
            cosA=cos(rotation);
            sinA=sin(rotation);
            for(int y=0;y<ySize;y++){
                for(int x=0;x<xSize;x++){
                    int newX=int(std::round(((x-xSize/2)*cosA-(y-ySize/2)*sinA)+xSize/2));
                    int newY=int(std::round(((x-xSize/2)*sinA+(y-ySize/2)*cosA)+ySize/2));
                    if(newX<0 || newX>=mapSizeX) continue;
                    if(newY<0 || newY>=mapSizeY) continue;
                    mapTransformCell T;
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
        vector<int> data;
        data.resize(mapMsg.data.size());
        for(int index=0;index<mapMsg.data.size();index++){
            data[index]=mapMsg.data[index];
        }
        /*ROS_INFO("r1");
        map.updateMap(data,mapMsg.info.width,mapMsg.info.height,
                      mapMsg.info.resolution,mapMsg.info.origin.position.x,mapMsg.info.origin.position.y);
        ROS_INFO("r2");*/
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

    bool checkOpening(opening o){
        for(int sids=0;sids<2;sids++){
            bool check=false;
            point_int *p=sids?&o.end:&o.start;
            point_int dir={0,1};
            for(int d=0;d<4;d++){
                if(getMap(p->x+dir.x,p->y+dir.y,topMap)==100){
                    check=true;
                    break;
                }
                dir=rotate_dir(dir,true);
                dir=rotate_dir(dir,true);
            }
            for(int d=0;d<2 && !check;d++){
                ant_data step;
                step.end=*p;
                for(int s=0;s<10;s++){
                    step=ant_step(step.end,d,step.dir,topMap);
                    if(!step.emty_cell){
                        check=true;
                    }
                }
            }
            if(!check) return false;
        }
        return true;
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
    bool cleanOpenings(int index){
        vector<int> vo;
        vector<int> vint;
        bool selfDel=true;
        opening op=oplist[index];
        //Follow the wall at the start and end points of op in cw and ccw direction.
        for(int sids=0; sids<2; sids++){
            for(int cw=0; cw<2; cw++){
                ant_data step;
                point_int endP;
                step.dir={0,0};
                int empty_count=0;
                if(sids==0){
                    step.end=op.start;
                    endP=op.end;
                }else{
                    step.end=op.end;
                    endP=op.start;
                }
                //will follow the wall for at least searchLenghtClean steps
                for(int s=0; s<searchLenghtClean && empty_count<maxAntGap; s++){
                    
                    //Search will stop once a opening facing the other direction is found, 
                    //this is to stop cleaning of opening detection that does not belong to the current opening
                    //if(check_for_opening(step.end,sids==0?2:1))break;

                    if(step.emty_cell){
                        empty_count+=1;
                    }else if(empty_count>0){
                        empty_count-=1;
                    }
                    
                    if(sids==0){
                        vector<int> opIndex=check_and_get_all_opening(step.end,1,false,index);
                        for(int m=0;m<opIndex.size();m++){
                            vo.push_back(opIndex[m]);
                            int sepL=cw==1?s:-s;
                            vint.push_back(sepL);
                        }
                    }else{
                        vector<int> opIndex=check_and_get_all_opening(step.end,2,false,index);
                        for(int m=0;m<opIndex.size();m++){
                            //check if opening was found when the other side was searched
                            for(int index=0; index<vo.size(); index++){
                                if(vo[index]==opIndex[m]){
                                    vint[index]+=cw==0?s:-s;
                                    //remove the last fitting opening
                                    oplist[opIndex[m]].label=14;
                                    /*if(dist(op.start,op.end)+vint[index]/extendDevider<dist(oplist[opIndex].start,oplist[opIndex].end)){
                                        oplist[opIndex].label=14;
                                        return true;
                                    }else{
                                        selfDel = false;
                                        return false;
                                    }*/
                                    break;
                                }
                            }
                        }
                    }
                    
                    step=ant_step(step.end,(bool)cw, step.dir,topMap);
                    if(step.end==endP) break;
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
                    //ROS_INFO("%i, %i, %i, %i, %i, %i",step.end.x,step.end.y,o2->start.x,o2->start.y,o2->end.x,o2->end.y);
                    if(step.end==o2->start || step.end==o2->end){
                        if(conection_lenght[sides]==-1 ||
                            s<conection_lenght[sides]){
                            conection_lenght[sides]=s;
                            conection_cw[sides]=dir;
                            conections[sides]=step;
                            if(step.end==o2->start){
                                conection_cw[sides]=true;
                            }else conection_cw[sides]=false;
                            break;
                        }
                    }
                    step=ant_step(step.end,dir,step.dir,topMap);
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
        point_int *o1h,*o2h;
        if(!sides){
            o1h=&o1->start;
        }else o1h=&o1->end;

        if(conection_cw[sides]){
            o2h=&o2->start;
        }else o2h=&o2->end;

        point_int holder;
        holder=*o1h;
        *o1h=*o2h;
        *o2h=holder;
        if(checkForWall(*o1,1,topMap)||checkForWall(*o2,1,topMap)){
            holder=*o1h;
            *o1h=*o2h;
            *o2h=holder;
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
            gapG= travGaps[angelindex][row][gIndex].nextGroup;
        }else{
            gapG= travGaps[angelindex][row][gIndex].prevGroup;
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
            travGaps[angelindex][row][gIndex].nextGroup=gapG;
        }else{
            travGaps[angelindex][row][gIndex].prevGroup=gapG;
        }
        return gapG.size();
    }

    void gapAnalysis(int angleIndex){
        //Loop thru all map cells
        for(int i=0;i<mapTransformList[angleIndex].size();i++){
            scanGroup sg;
            vector<scanGroup> sgtList;
            vector<scanGroup> sgntList;
            int cfilter=0;
            int cGroupeSize=0;
            for(int j=0;j<mapTransformList[angleIndex][i].size();j++){
                mapTransformCell mt=mapTransformList[angleIndex][i][j];
                //Finde groups
                if(Map[mt.rpos.x][mt.rpos.y]==0){ //&& getMapTransform(Map,mt.tpos.x,i+1,angleIndex)==0){
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
                    sg.end=endpoint;
                    sg.row=i;
                    sg.prevGroup.clear();
                    sg.nextGroup.clear();
                    if(cGroupeSize>minGroupSize){
                        sgtList.push_back(sg);

                    }else if(cGroupeSize>0){
                        sgntList.push_back(sg);
                    }
                    cfilter=0;
                    cGroupeSize=0;
                }
            }
            travGaps[angleIndex][i]=sgtList;
            nonTravGaps[angleIndex][i]=sgntList;
            for(int index1=0;index1<travGaps[angleIndex][i].size() && i!=0;index1++){
                //find if there is tow or more gropse conecteing to a previus group
                for(int index2=0;index2<travGaps[angleIndex][i-1].size();index2++){
                    if(travGaps[angleIndex][i][index1].start<travGaps[angleIndex][i-1][index2].end &&
                        travGaps[angleIndex][i][index1].end>travGaps[angleIndex][i-1][index2].start){
                            int overlapStart=std::max(travGaps[angleIndex][i][index1].start,travGaps[angleIndex][i-1][index2].start);
                            int overlapEnd=std::min(travGaps[angleIndex][i][index1].end,travGaps[angleIndex][i-1][index2].end);
                            int overlapSize=overlapEnd-overlapStart;
                            
                            if(overlapSize<minGroupSize){
                                scanGroup sg;
                                sg.start=overlapStart;
                                sg.end=overlapEnd;
                                sg.row=i;
                                nonTravGaps[angleIndex][i].push_back(sg);
                                sg.row=i-1;
                                nonTravGaps[angleIndex][i-1].push_back(sg);
                                if(travGaps[angleIndex][i][index1].end
                                    -travGaps[angleIndex][i][index1].start
                                    -overlapSize<minGroupSize){
                                    travGaps[angleIndex][i][index1].end=-1;
                                    travGaps[angleIndex][i][index1].start=-1;
                                }else{
                                    if(travGaps[angleIndex][i][index1].start==overlapStart ||
                                        travGaps[angleIndex][i][index1].start==overlapEnd){
                                        travGaps[angleIndex][i][index1].start=overlapEnd+1;
                                    }else{
                                        travGaps[angleIndex][i][index1].end=overlapStart-1;
                                    }
                                }
                                if(travGaps[angleIndex][i-1][index2].end
                                    -travGaps[angleIndex][i-1][index2].start
                                    -overlapSize<minGroupSize){
                                    travGaps[angleIndex][i-1][index2].end=-1;
                                    travGaps[angleIndex][i-1][index2].start=-1;
                                }else{
                                    if(travGaps[angleIndex][i-1][index2].start==overlapStart ||
                                        travGaps[angleIndex][i-1][index2].start==overlapEnd){
                                        travGaps[angleIndex][i-1][index2].start=overlapEnd+1;
                                    }else{
                                        travGaps[angleIndex][i-1][index2].end=overlapStart-1;
                                    }
                                }
                                continue;
                            }
                            travGaps[angleIndex][i][index1].prevGroup.push_back(&travGaps[angleIndex][i-1][index2]);

                            travGaps[angleIndex][i-1][index2].nextGroup.push_back(&travGaps[angleIndex][i][index1]);
                    }
                }
            }
        }
    }

    void filterRemoveOpenings(int angleIndex){
        for(int row=0; row<travGaps[angleIndex].size(); row++){
            for(int index=0;index<travGaps[angleIndex][row].size();index++){
                scanGroup sg=travGaps[angleIndex][row][index];
                for(int m=sg.start;m<=sg.end; m++){
                    if(getMapTransform(topMap,m,row,angleIndex)==100) continue;
                    setMapTransform(topMap,m,row,angleIndex,0);
                }
            }
        }
        for(int row=0; row<nonTravGaps[angleIndex].size(); row++){
            for(int index=0;index<nonTravGaps[angleIndex][row].size();index++){
                scanGroup sg=nonTravGaps[angleIndex][row][index];
                if(//getMapTransform(Map,sg.start-1,row,angleIndex)==0 && getMapTransform(Map,sg.end+1,row,angleIndex)==0&&
                     getMapTransform(Map,sg.start-1,row,angleIndex)+getMapTransform(Map,sg.end+1,row,angleIndex)<99)
                        continue;
                for(int m=sg.start;m<=sg.end; m++){
                    setMapTransform(topMap,m,row,angleIndex,100);
                }
            }
        }
    }
    void filterRemoveObjects(int angleIndex){
        for(int i=1;i<travGaps[angleIndex].size();i++){
            for(int index= 0; index<travGaps[angleIndex][i].size();index++){
                for(int direction=0;direction<2;direction++){
                    if(travGaps[angleIndex][i][index].prevGroup.size()<2 &&direction==0||
                        travGaps[angleIndex][i][index].nextGroup.size()<2 &&direction==1) continue;
                    fitGapToMap(angleIndex,i,index,direction);
                    int gIndex=0;
                    while(true){
                        if(travGaps[angleIndex][i][index].prevGroup.size()<2 &&direction==0||
                        travGaps[angleIndex][i][index].nextGroup.size()<2 &&direction==1) break;
                        vector<scanGroup*> gapG;
                        if(direction){
                            gapG= travGaps[angleIndex][i][index].nextGroup;
                        }else{
                            gapG= travGaps[angleIndex][i][index].prevGroup;
                        }
                        ant_data step;
                        point_int startP=getMapIndexTransform(gapG[gIndex]->start,gapG[gIndex]->row,angleIndex);
                        step.end=getMapIndexTransform(gapG[gIndex]->end,gapG[gIndex]->row,angleIndex);
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
                                    if(fitGapToMap(angleIndex,i,index,direction)==pSize){
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

    void gapDetection(int angleIndex){
        for(int i=1;i<travGaps[angleIndex].size();i++){
            for(int index= 0; index<travGaps[angleIndex][i].size();index++){
                for(int direction=0;direction<2;direction++){
                    //If the gap has less than two connections then skip this gap.
                    if(travGaps[angleIndex][i][index].prevGroup.size()<2 &&direction==0||
                        travGaps[angleIndex][i][index].nextGroup.size()<2 &&direction==1) continue;
                    
                    //Count depth of original gap
                    scanGroup *p=&travGaps[angleIndex][i][index];
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
                    int loopAmount=travGaps[angleIndex][i][index].prevGroup.size();
                    if(direction==1){
                        loopAmount=travGaps[angleIndex][i][index].nextGroup.size();
                    }
                    for(int h=0;h<loopAmount;h++){
                        depthCount=0;
                        int endIndex=0;
                        if(direction==0){
                            p=travGaps[angleIndex][i][index].prevGroup[h];
                            endIndex=travGaps[angleIndex][i][index].prevGroup.size()-1;
                        }else{
                            p=travGaps[angleIndex][i][index].nextGroup[h];
                            endIndex=travGaps[angleIndex][i][index].nextGroup.size()-1;
                        }

                        while (true){
                            depthCount+=1;
                            if(depthCount==minCoridorSize){
                                int newStart, newEnd, row; 
                                if(direction==0){
                                    newStart=travGaps[angleIndex][i][index].prevGroup[h]->start;
                                    newEnd=travGaps[angleIndex][i][index].prevGroup[h]->end;
                                    row=travGaps[angleIndex][i][index].prevGroup[h]->row;
                                }else{
                                    newStart=travGaps[angleIndex][i][index].nextGroup[h]->start;
                                    newEnd=travGaps[angleIndex][i][index].nextGroup[h]->end;
                                    row=travGaps[angleIndex][i][index].nextGroup[h]->row;
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
                                newOp.angle=angleIndex;
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

                    //rotate point back to original rotation
                    newOpList=rotate_points(newOpList,angleIndex);
                    for(int k=0; k<newOpList.size();k++){
                        oplist.push_back(newOpList[k]);
                    } 
                }
            }
        }
    }

    void openingOptimization(vector<opening> newOplist){
        for(int index=0;index<newOplist.size();index++){
            if(newOplist[index].sideToMove!=3){
                vector<point_int> intersectP;
                vector<int> intersectIndex;
                for(int index2=0;index2<newOplist.size();index2++){
                    if(index==index2) continue;
                    if(newOplist[index2].sideToMove!=3) continue;
                    if(!intersect_line(&newOplist[index],&newOplist[index2])) continue;
                    intersectP.push_back(findIntersectionPoint(newOplist[index],newOplist[index2]));
                    intersectIndex.push_back(index2);
                }
                if(intersectP.size()>0){
                    point_int testP=newOplist[index].sideToMove==1?
                                    newOplist[index].end:newOplist[index].start;
                    double minLenght=dist(testP,intersectP[0]);
                    int bestIndex=intersectIndex[0];
                    for(int m=1;m<intersectP.size();m++){
                        double length=dist(testP,intersectP[m]);
                        if(length<minLenght){
                            minLenght=length;
                            bestIndex=intersectIndex[m];
                        }
                    }
                    point_int moveToP=newOplist[index].sideToMove==1?
                                      newOplist[bestIndex].end:newOplist[bestIndex].start;
                    if(newOplist[index].sideToMove==1){
                        newOplist[index].start=moveToP;
                    }else{
                        newOplist[index].end=moveToP;
                    }
                    if(dist(newOplist[index].start,newOplist[index].end)<minGroupSize ){
                        newOplist[index].label=14;
                    }
                    else if(checkForWall(newOplist[index],1,topMap)){
                        newOplist[index].label=12;
                    }
                }
            }
            //if(check_unnecessary_openings(newOplist[index],topMap)) newOplist[index].label=14;
        }
        oplist=newOplist;
        for(int i=0;i<oplist.size();i++){
            if(!checkOpening(oplist[i])) oplist[i].label=14;
        }
        for(int i=0;i<oplist.size();i++){
            for(int j=0;j<oplist.size();j++){
                if(j==i) continue;
                if(oplist[i].label>10) continue;
                if(oplist[j].label>10) continue;
                if(intersect_line(&oplist[i],&oplist[j]))
                    fixOverlap(&oplist[i],&oplist[j],50);
            }
        }
        for(int i=0;i<oplist.size();i++){
            if(check_unnecessary_openings(oplist[i],topMap)) oplist[i].label=14;
            if(checkForWall(oplist[i],1,topMap)) oplist[i].label=12;
        }
        for(int i=0;i<oplist.size();i++){
            if(oplist[i].label<10) cleanOpenings(i);
        }

        // move all points so they dont overlap
        for(int index=0; index<oplist.size();index++){
            for(int sids=0;sids<2;sids++){
                point_int targetP=sids?oplist[index].end:oplist[index].start;
                if(!check_for_opening(targetP,3,false,index)) continue;
                vector<point_int> cwPoints;
                vector<point_int> ccwPoints;
                bool cw=true;
                ant_data cwStep, ccwStep;
                cwStep.end=targetP;
                ccwStep.end=targetP;
                for(int c=0;c<30;c++){
                    if(cw){
                        cwStep=ant_step(cwStep.end,cw,cwStep.dir,topMap);
                        cwPoints.push_back(cwStep.end);
                        if(!check_for_opening(cwStep.end,3,false,index)){
                            for(int i=cwPoints.size()-2;i>=0;i--){
                                vector<point_int*> pp=check_and_get_all_opening_pointers(cwPoints[i],3);
                                for(int pindex=0; pindex<pp.size();pindex++){
                                    *pp[pindex]=cwPoints[i+1];
                                }
                            }
                            vector<int> pOpening=check_and_get_all_opening(targetP,3);
                            double minAng=-1;
                            point_int *moveP=NULL;
                            for(int i=0;i<pOpening.size();i++){
                                point_int *sP, *mP;   
                                if(oplist[pOpening[i]].start==targetP){
                                    mP=&oplist[pOpening[i]].start;
                                    sP=&oplist[pOpening[i]].end;
                                }else{
                                    mP=&oplist[pOpening[i]].end;
                                    sP=&oplist[pOpening[i]].start;
                                }
                                double ang=angleBetweenLines(*sP,*mP,cwStep.end,cwStep.dir);
                                if(minAng<0||ang<minAng){
                                    minAng=ang;
                                    moveP=mP;
                                }
                            }
                            if(moveP!=NULL)
                            *moveP=cwPoints[0];
                            break;
                        }
                    }else{
                        ccwStep=ant_step(ccwStep.end,cw,ccwStep.dir,topMap);
                        ccwPoints.push_back(ccwStep.end);
                        if(!check_for_opening(ccwStep.end,3,false,index)){
                            for(int i=ccwPoints.size()-2;i>=0;i--){
                                vector<point_int*> pp=check_and_get_all_opening_pointers(ccwPoints[i],3);
                                for(int pindex=0; pindex<pp.size();pindex++){
                                    *pp[pindex]=ccwPoints[i+1];
                                }
                            }
                            vector<int> pOpening=check_and_get_all_opening(targetP,3);
                            double minAng=-1;
                            point_int *moveP=NULL;
                            for(int i=0;i<pOpening.size();i++){
                                point_int *sP, *mP;   
                                if(oplist[pOpening[i]].start==targetP){
                                    mP=&oplist[pOpening[i]].start;
                                    sP=&oplist[pOpening[i]].end;
                                }else{
                                    mP=&oplist[pOpening[i]].end;
                                    sP=&oplist[pOpening[i]].start;
                                }
                                double ang=angleBetweenLines(*sP,*mP,ccwStep.end,ccwStep.dir);
                                if(minAng<0||ang<minAng){
                                    minAng=ang;
                                    moveP=mP;
                                }
                            }
                            if(moveP!=NULL)
                            *moveP=ccwPoints[0];
                            break;
                        }
                    }
                    cw=!cw;
                }

            }
        }
        
    }


    void topologyScan(){
        //copy map to topMap, clear other maps
        for(int x=0; x<mapSizeX;x++){
            for(int y=0;y<mapSizeY;y++){
                topMap[x][y]=Map[x][y];
                dMap[x][y]=-1;
                ObjectFilterLookup[x][y]=false;
            }
        }
        //_____GAP ANALYSIS_____
        for(int angleIndex=0; angleIndex<numberOfDir; angleIndex++){
            //Clear gaps from previus scan
            travGaps[angleIndex].clear();
            travGaps[angleIndex].resize(mapTransformList[angleIndex].size());
            nonTravGaps[angleIndex].clear();
            nonTravGaps[angleIndex].resize(mapTransformList[angleIndex].size());
            
            gapAnalysis(angleIndex);
        }
        //________FILTER________
        //First filter
        for(int angleIndex=0; angleIndex<numberOfDir; angleIndex++){
            if(removeOpeningsFirst){
                filterRemoveOpenings(angleIndex);
            }else{
                filterRemoveObjects(angleIndex);
            }
        }
        //Second filter
        for(int angleIndex=0; angleIndex<numberOfDir; angleIndex++){
            if(removeOpeningsFirst){
                filterRemoveObjects(angleIndex);
            }else{
                filterRemoveOpenings(angleIndex);
            }
        }
        //_____GAP DETECTION_____
        for(int angleIndex=0; angleIndex<numberOfDir; angleIndex++){
            gapDetection(angleIndex);
        }  
        for(int i=0;i<oplist.size();i++){
            correctOpening(&oplist[i],10);
        }

        //_____FIND OPENINGS_____
        vector<opening> newOplist;
        while(oplist.size()!=0){
            vector<int> ignorlist;
            if(!FindOpenings(0,&newOplist,&ignorlist)){
                //oplist[0].label=5;
                newOplist.push_back(oplist[0]);
                oplist.erase(oplist.begin());
            }
        }
        //oplist=newOplist;
        //__OPENING OPTIMIZATION_
        openingOptimization(newOplist);
        
    }

    point_int findIntersectionPoint(opening o1, opening o2) {
        // Line 1: y = m1*x + b1, where m1 = (y2 - y1)/(x2 - x1) and b1 = y1 - m1*x1
        double m1, b1;
        if (std::abs(o1.end.x - o1.start.x) < 1e-9) {  // Line 1 is vertical
            m1 = INFINITY;
            b1 = o1.start.x;
        } else {
            m1 = (o1.end.y - o1.start.y)/(o1.end.x - o1.start.x);
            b1 = o1.start.y - m1*o1.start.x;
        }

        // Line 2: y = m2*x + b2, where m2 = (y4 - y3)/(x4 - x3) and b2 = y3 - m2*x3
        double m2, b2;
        if (std::abs(o2.end.x - o2.start.x) < 1e-9) {  // Line 1 is vertical
            m2 = INFINITY;
            b2 = o2.start.x;
        } else {
            m2 = (o2.end.y - o2.start.y)/(o2.end.x - o2.start.x);
            b2 = o2.start.y - m2*o2.start.x;
        }

        // Find intersection
        point p;
        if (std::isinf(m1)) {  // Line 1 is vertical, so intersection point has x-coordinate of line 1
            p={b1, m2*b1 + b2};
        } else if (std::isinf(m2)) {  // Line 2 is vertical, so intersection point has x-coordinate of line 2
            p={b2, m1*b2 + b1};
        } else {
            // y = m1*x + b1 and y = m2*x + b2
            // m1*x + b1 = m2*x + b2
            // x = (b2 - b1)/(m1 - m2)
            double x = (b2 - b1)/(m1 - m2);
            double y = m1*x + b1;  // or y = m2*x + b2
            p={x, y};
            
        }
        point_int p_int={int(std::round(p.x)),int(std::round(p.y))};
        return p_int;
    }

    bool FindOpenings(int targetIndex, vector<opening> *newOplist, vector<int> *ignorList){
        while (true){
            if(oplist[targetIndex].sideToMove==3){
                return false;
            }
            vector<int> interOpIndex;
            vector<point_int> interPoint;
            for(int index=0;index<oplist.size();index++){
                if(targetIndex==index) continue;
                if(oplist[index].sideToMove==3) continue;
                if(oplist[targetIndex].sideToMove==oplist[index].sideToMove) continue;
                if(!intersect_line(&oplist[targetIndex],&oplist[index])) continue;
                interOpIndex.push_back(index);
                interPoint.push_back(findIntersectionPoint(oplist[targetIndex],oplist[index]));
            }
            
            if(interOpIndex.size()==0){
                return false;
            }

            opening bestNewOp;
            double bestScore=-1;
            int selectedIndex=-1;
            for(int i=0; i<interOpIndex.size();i++){
                opening newOp;
                if(oplist[targetIndex].sideToMove==1){
                    newOp.start=oplist[interOpIndex[i]].start;
                    newOp.end=oplist[targetIndex].end;
                }else{
                    newOp.start=oplist[targetIndex].start;
                    newOp.end=oplist[interOpIndex[i]].end;
                }
                if(dist(newOp.start,newOp.end)<minGroupSize) continue;

                //double d1=dist(interPoint[i],newOp.end);
                //double d2=dist(newOp.start,interPoint[i]);
                //double score=(d1-d2)*(d1-d2);
                //double score=d1*d1+d2*d2;
                double score=dist(newOp.start,newOp.end);
                
                //newOplist->push_back(newOp);
                if(bestScore<0||score<bestScore){
                    if((newOp.end.x-newOp.start.x)*(interPoint[i].y-newOp.start.y)-
                       (newOp.end.y-newOp.start.y)*(interPoint[i].x-newOp.start.x)>0) continue;
                    if(checkForWall(newOp,1,topMap)) continue;
                    if(check_unnecessary_openings(newOp,topMap)) continue;
                    selectedIndex=interOpIndex[i];
                    bestNewOp=newOp;
                    bestScore=score;
                }
            }
            
            if(selectedIndex==-1){
                return false;
            }
            //oplist.erase(oplist.begin()+targetIndex);
            //return true;
            for(int i=0; i<ignorList->size(); i++){
                if(ignorList->at(i)==selectedIndex) return false;
            }
            ignorList->push_back(targetIndex);
            bool check=FindOpenings(selectedIndex,newOplist,ignorList);
            ignorList->pop_back();
            if(check) continue;
            newOplist->push_back(bestNewOp);
            oplist.erase(oplist.begin()+std::max(targetIndex,selectedIndex));
            oplist.erase(oplist.begin()+std::min(targetIndex,selectedIndex));

            for(int i=0; i<ignorList->size(); i++){
                if(ignorList->at(i)>targetIndex) ignorList->at(i)--;
                if(ignorList->at(i)>selectedIndex) ignorList->at(i)--;
            }
            return true;
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
        pubTopoMap.publish(topoMapMsg);
        for(int oi=0;oi<oplist.size();oi++){
            for(int i=0;i<oplist[oi].occupied_points.size();i++){
                dMap[oplist[oi].occupied_points[i].x][oplist[oi].occupied_points[i].y]=100;
            }
            dMap[oplist[oi].start.x][oplist[oi].start.y]=0;
            dMap[oplist[oi].end.x][oplist[oi].end.y]=50;
        }
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