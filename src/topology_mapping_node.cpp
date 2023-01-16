#include "Utility.h"                                

class TopologyMapping{
    private:
        //ROS nh
        ros::NodeHandle nh;
        //subs
        ros::Subscriber subOccupancyMap;
        ros::Subscriber subOpeningList;
        //pub
        ros::Publisher pubMapDebug;
        ros::Publisher pubTopoPoly_debug;
        ros::Publisher pubTopoPoly;
        ros::Publisher pubTopometricMap;
        ros::Publisher pubRobotPath;
        ros::Publisher pubMarkDel;

        //msg
        nav_msgs::OccupancyGrid topoMapMsg;
        visualization_msgs::Marker markDel;
        topology_mapping::topometricMap topometricMapMsg;


        //Map
        int **topMap;
        int **debugMap;
        
        //Global var.
        bool rMap=false, rOpening=false;
        vector<vector<point_int>> robotPath;
        int oldNodCount=0;


    public:
        //Setup
        TopologyMapping(){
            subOccupancyMap= nh.subscribe("/topology_map_filterd",1,&TopologyMapping::updateMap, this);
            subOpeningList= nh.subscribe("/opening_list_int",1,&TopologyMapping::updateOplist, this);
            pubMapDebug=nh.advertise<nav_msgs::OccupancyGrid>("/topology_map_debug",5);
            pubTopoPoly_debug=nh.advertise<jsk_recognition_msgs::PolygonArray>("/topology_poly_opening",5);
            pubRobotPath=nh.advertise<visualization_msgs::MarkerArray>("/topology_robot_path",5);
            pubTopoPoly=nh.advertise<jsk_recognition_msgs::PolygonArray>("/topology_poly",5);
            pubTopometricMap=nh.advertise<topology_mapping::topometricMap>("/topometricMap",5);
            loadMemory();
        }
    
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
        topoMapMsg.info.origin.position.z = mapHight+.1; //for visualization

        topoMapMsg.data.resize(topoMapMsg.info.width * topoMapMsg.info.height);
        std::fill(topoMapMsg.data.begin(), topoMapMsg.data.end(), -1);
    }
        
    //Load all maps into the memory
    void loadMemory(){
        oplist.reserve(400);
        //alocate for Maps
        topMap=new int*[mapSizeX];
        debugMap=new int*[mapSizeX];
        for(int i=0;i<mapSizeX;i++){
            topMap[i]=new int[mapSizeY];
            debugMap[i]=new int[mapSizeY];
        }
        
        for (int i = 0; i < mapSizeX; ++i){
            for (int j = 0; j < mapSizeY; ++j){
                topMap[i][j] = -1;
                debugMap[i][j] = -1;
            }
        }
        initializeTopoMap();
    }
    void unloadMemory(){
        //alocate for Maps
        for(int i=0;i<mapSizeX;i++){
            delete[] topMap[i];
            delete[] debugMap[i];
        }
        delete[] topMap;
        delete[] debugMap;
    }


    void spin(){
        ros::Rate rate(100); // Hz
        
        while (ros::ok()){
            //waiting to receive new topic
            if(rMap && rOpening){
                robotPath.clear();
                if(oplist.size()>0){
                    vector<poligon> poly_list=creatPoligonList();
                    for(int b=0; b<oplist.size(); b++){
                        if(oplist[b].parent_poligon<0 && oplist[b].label<10){
                            oplist[b].label=24;
                        }
                    }
                    poly_list=optimize_intersection_openings(poly_list);
                    if(poly_list.size()>0){
                        poly_list=creatPathPoligons(poly_list);
                        poly_list=generat_robot_path(poly_list);
                    }
                    //remove unused openings
                    for(int b=0; b<oplist.size(); b++){
                        if(oplist[b].parent_poligon<0 && oplist[b].label<10){
                            oplist[b].label=24;
                        }
                    }
                    for(int b=0; b<oplist.size() && !show_removed_openings; b++){
                        if(oplist[b].label>10){
                            oplist.erase(oplist.begin()+b);
                            b-=1;
                        }
                    }
                    pubMap(poly_list);
                }
                
                rMap=false;
                rOpening=false;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

    //Copy openings from messages to opList.
    void updateOplist(const topology_mapping::opening_list& opMsg){
        oplist.resize(opMsg.list.size());
        for(int i=0; i<opMsg.list.size(); i++){
            opening op;
            op.start.x=opMsg.list[i].start.x;
            op.start.y=opMsg.list[i].start.y;
            op.end.x=opMsg.list[i].end.x;
            op.end.y=opMsg.list[i].end.y;
            op.label=opMsg.list[i].label;
            oplist[i]=op;
        }
        rOpening=true;
    }

    //Get new filtered occupancy map and move its value into Map.
    void updateMap(const nav_msgs::OccupancyGrid& mapMsg){
        int width=mapMsg.info.width;
        int height=mapMsg.info.height;
        resolution=mapMsg.info.resolution;
        mapOffsetX=mapMsg.info.origin.position.x;
        mapOffsetY=mapMsg.info.origin.position.y;
        mapHight=mapMsg.info.origin.position.z;
        initializeTopoMap();
        if(width!=mapSizeX || height!=mapSizeY){
            unloadMemory();
            mapSizeX=width;
            mapSizeY=height;
            loadMemory();
        }

        //int mapResolution=mapMsg.occupancy.info.resolution;
        for(int x=0; x<width;x++){
            for(int y=0; y<height;y++){
                int index= x+y*width;
                topMap[x][y]=mapMsg.data[index];
            }
        }
        rMap=true;
    }
    vector<point_int> fillPoints(point_int start, point_int end,double spacing,bool lastAndFirst=false){
        vector<point_int> out;
        point norm;
        double d=dist(start,end);
        norm.x=(end.x-start.x)/d*spacing;
        norm.y=(end.y-start.y)/d*spacing;

        for(int s=lastAndFirst?0:1;s<(int)(d/spacing);s++){
            point_int p;
            p.x=start.x+std::round(norm.x*s);
            p.y=start.y+std::round(norm.y*s);
            out.push_back(p);
        }
        if(lastAndFirst){
            out.push_back(end);
        }

        return out;
    }

    poligon creat_poligon_area(poligon poly){
        vector<point_int> points;
        vector<point_int> points_desplay;
        int oIndex=poly.sidesIndex[0];
        for(int sIndex=0;sIndex<poly.sidesIndex.size();sIndex++){
            ant_data step_info;
                    
            step_info.end=oplist[oIndex].start; 
            int rezCounter=poligonRez;
            step_info.dir={0,0};
            for(int s=0; s<=sercheLenthAntConect; s++){
                //don't save all point to save on resources
                points.push_back(step_info.end);
                if(rezCounter>=poligonRez){
                    rezCounter=0;
                    points_desplay.push_back(step_info.end);
                }else{
                    rezCounter+=1;
                }
                int newOIndex=-1;
                for(int i=0;i<poly.sidesIndex.size();i++){
                    if(step_info.end==oplist[poly.sidesIndex[i]].end){
                        newOIndex=poly.sidesIndex[i];
                    }
                }
                if(newOIndex!=-1){
                    points_desplay.push_back(step_info.end);
                    vector<point_int> np=fillPoints(oplist[newOIndex].end,oplist[newOIndex].start,0.7,true);
                    for(int i=0;i<np.size();i++) points.push_back(np[i]);
                    points_desplay.push_back(np[np.size()-1]);
                    oIndex=newOIndex;
                    break;
                }
                step_info=ant_step(step_info.end,true,step_info.dir,topMap);
            }                        
        }
        poly.poligon_points=points;
        poly.poligon_points_desplay=points_desplay;
        return poly;
    }


    point_int get_poligon_center(vector<point_int> sList){
        point_int center={0,0};
        for(int i=0; i<sList.size();i++){
            center.x+=sList[i].x;
            center.y+=sList[i].y;
        }
        center.x=center.x/(sList.size());
        center.y=center.y/(sList.size());

        return center;
    }

    bool testIntersection(vector<int> oIndexList, point_int center){
        bool t=false;
        for(int h=0; h<oIndexList.size() && !t;h++){
            opening o;
            o.start=center;
            o.end=oplist[oIndexList[h]].get_center();
            t=t?t:checkForWall(o,1,topMap);
        }
        return !t;
    }

    //Remove connection of all openings connected to the same poligon as opening with index Index, poligon is then tagged for removal.
    vector<poligon> remove_parent_poligon(vector<poligon> poly_list,int opIndex, bool remove_openign=false, bool remove_all_openings=false){
        if(opIndex>=0){
            if(oplist[opIndex].parent_poligon>=0){
                int pIndex=oplist[opIndex].parent_poligon;
                for(int i=0; i<poly_list[pIndex].sidesIndex.size();i++){
                    oplist[poly_list[pIndex].sidesIndex[i]].parent_poligon=-1;
                    if(remove_all_openings) oplist[poly_list[pIndex].sidesIndex[i]].label=26;
                    poly_list[pIndex].inactiv=true;
                }
            }
            if(remove_openign){
                oplist[opIndex].label=26;
            }
        }
        return poly_list;
    }

    //Connect all openings to create a list of polygons representing the intersections.
    vector<poligon> creatPoligonList(){
        vector<poligon> poly_list;
        for(int i=0; i<oplist.size(); i++){
            if(oplist[i].parent_poligon==-1 && oplist[i].label==1){
                //set init. val. for all var:s needed.
                //poly is a temp variable holding the information that will be appended to poligon_list 
                bool cw=true;
                poligon poly;
                poly.inactiv=false;
                poly.path=false;
                poly.label=1;
                poly.sidesIndex.push_back(i);
                oplist[i].parent_poligon=-2;//A temporary value used to indicate that the polygon is connected.
                ant_data step_info;      
                bool cheek=false, complet=false, moved=false;
                int firstIndex=i;
                int targetIndex=i;
                int lastIndex=-1;
                int IndexBadC=-1;
                int totalSlenght=0;
                int opIndex;
                while(!cheek){
                    int empty_cell_count=0, empty_cell_count_max=0;
                    if(cw){
                        step_info.end=oplist[targetIndex].start; 
                    }else{
                        step_info.end=oplist[targetIndex].end;
                    }
                    int rezCounter=0;
                    step_info.dir={0,0};
                    for(int s=0; s<=sercheLenthAntConect; s++){
                        opIndex=check_and_get_opening(step_info.end,cw?1:2,true);
                        int opIndexGood=check_and_get_opening(step_info.end,cw?2:1,true);
                        if(s==0 && opIndexGood==-1){
                            opIndex=-1;
                        }
                        if(opIndex!=-1&&opIndexGood!=-1){
                            if(!(oplist[opIndex].start==oplist[opIndexGood].end&&
                               oplist[opIndex].end==oplist[opIndexGood].start)){
                                point_int p1,p2;
                                if(cw){
                                    p1=oplist[opIndex].end;
                                    p2=oplist[opIndexGood].start;
                                }else{
                                    p2=oplist[opIndex].start;
                                    p1=oplist[opIndexGood].end;
                                }
                                p1.x-=step_info.end.x;
                                p1.y-=step_info.end.y;
                                p2.x-=step_info.end.x;
                                p2.y-=step_info.end.y;

                                double ang1=atan2(p1.y, p1.x);
                                double ang2=atan2(p2.y, p2.x);
                                double ang=ang1-ang2;
                                if(abs(ang)>M_PI){
                                    double newAng=abs(ang)-M_PI*2;
                                    if(ang<0){
                                        ang=-newAng;
                                    }else ang=newAng;
                                }
                                if(s==0 && ang<0){
                                    opIndex=-1;
                                }
                                else if(s==0){
                                    opIndex=-1;
                                    opIndexGood=-1;
                                }
                                else if(s!=0 && ang>0){
                                    opIndex=-1;
                                }
                            }else if(s==0){
                                opIndex=-1;
                                opIndexGood=-1;
                            }else opIndex=-1;
                        }
                        
                        // found bad conection
                        if(opIndex!=-1 || s==sercheLenthAntConect){
                                if(cw){
                                    //ROS_INFO("found no connection searching other direction");
                                    lastIndex=targetIndex;
                                    targetIndex=firstIndex;
                                    IndexBadC=opIndex;
                                    cw=false;
                                    break;
                                }else{
                                    if(IndexBadC!=opIndex){
                                        opening newOp;
                                        point_int d={0,0};
                                        newOp.label=3;
                                        newOp.start=oplist[IndexBadC].end;
                                        newOp.end=oplist[IndexBadC].start;
                                        oplist.push_back(newOp);
                                        newOp.start=oplist[opIndex].end;
                                        newOp.end=oplist[opIndex].start;
                                        oplist.push_back(newOp);
                                        i--;
                                    }
                                    else if(poly.sidesIndex.size()>1){
                                        //ROS_INFO("Create new opening");
                                        //creat mising openign
                                        opening newOp;
                                        newOp.start=oplist[targetIndex].end;
                                        newOp.end=oplist[lastIndex].start;
                                        //fitt new opening to coridor 
                                        newOp.label=2;
                                        newOp.parent_poligon=-2;
                                        poly.add_sideIndex(oplist.size(),cw);
                                        int newIndex=oplist.size();
                                        oplist.push_back(newOp);
                                        //check if new opening is good 
                                        if(!checkForWall(newOp,1,topMap) && dist(newOp.start,newOp.end)>=minGroupSize || true){
                                            complet=true;

                                        }else{//bad new opening, remove conflicting opening instead
                                            oplist[newIndex].parent_poligon=-1;
                                            oplist[newIndex].label=11;
                                            if(opIndex>=0){
                                                poly_list=remove_parent_poligon(poly_list,opIndex,true);
                                                i-=1;
                                            }        
                                        }
                                    }
                                    
                                    //ROS_INFO("Found no connection on searching");
                                    cheek=true;
                                    break;
                                }
                        }

                        opIndex=opIndexGood;

                        //Good conection
                        if(opIndex!=-1){
                                totalSlenght+=s;
                                //If an opening filip has caused a new connection possible with a polygon that have created a new opening remov that poligon
                                if(oplist[opIndex].parent_poligon!=-1 && oplist[opIndex].parent_poligon!=-2){
                                    poly_list=remove_parent_poligon(poly_list,opIndex);
                                }
                                if(oplist[opIndex].parent_poligon==-1){
                                    //ROS_INFO("Found connection");
                                    if(empty_cell_count_max>maxAntGap){
                                        opening newOp;
                                        newOp.label=2;
                                        if(cw){
                                            newOp.start=oplist[opIndex].end;
                                            newOp.end=oplist[targetIndex].start;
                                        }else{
                                            newOp.start=oplist[targetIndex].end;
                                            newOp.end=oplist[opIndex].start;
                                        }
                                        oplist.push_back(newOp);
                                        oplist.back().parent_poligon=-2;
                                        poly.add_sideIndex(oplist.size()-1,cw);
                                    }
                                    oplist[opIndex].parent_poligon=-2;
                                    poly.add_sideIndex(opIndex,cw);
                                    moved=moved?moved:oplist[opIndex].moved;
                                    targetIndex=opIndex;
                                    break;
                                }else if(poly.sidesIndex.size()==2){//Too small connection, flipping the openings
                                    //ROS_INFO("Small connection");
                                    oplist[targetIndex].parent_poligon=-1;
                                    if(oplist[i].fliped){//remove opening if flipped two times
                                        oplist[i].label=28;
                                    }else{
                                        oplist[i].parent_poligon=-1;
                                        oplist[targetIndex].flip();
                                        oplist[i].flip();
                                        oplist[i].fliped=true;
                                        if(check_and_get_opening(oplist[i].start,1,false,i)!=-1||
                                           check_and_get_opening(oplist[i].end,2,false,i)!=-1)
                                        oplist[i].label=12;
                                        if(check_and_get_opening(oplist[targetIndex].start,1,false,targetIndex)!=-1||
                                           check_and_get_opening(oplist[targetIndex].end,2,false,targetIndex)!=-1)
                                        oplist[targetIndex].label=12;
                                        
                                    }
                                    i-=1;
                                    cheek=true;
                                    break;
                                }else if(poly.sidesIndex.size()==1){
                                    cheek=true;
                                    break;
                                }else{
                                    //ROS_INFO("Found complete polygon");
                                    cheek=true;
                                    complet=true;
                                    break;
                                }
                        }

                        step_info=ant_step(step_info.end,cw,step_info.dir,topMap);

                        if(step_info.emty_cell){
                            empty_cell_count+=1;
                            if(empty_cell_count>empty_cell_count_max)
                                empty_cell_count_max=empty_cell_count;
                        }else if(empty_cell_count>0){
                            empty_cell_count-=1;
                        }
                    }
                    if(complet){
                        int label=0;
                        /*for(int p=0;p< poly.sidesIndex.size();p++){
                            ROS_INFO("%i, start: %i, %i, end: %i, %i",i,oplist[poly.sidesIndex[p]].start.x, oplist[poly.sidesIndex[p]].start.y,oplist[poly.sidesIndex[p]].end.x, oplist[poly.sidesIndex[p]].end.y);
                        }*/
                        if(poly.sidesIndex.size()==1){
                            label=41;
                        }else if(poly.sidesIndex.size()==2){
                            label=20;
                        }else{
                            label=30;
                        }
                        //set openings parent_poligon to the correct index 
                        for(int p=0;p<poly.sidesIndex.size();p++){
                            oplist[poly.sidesIndex[p]].parent_poligon=poly_list.size();
                        }
                        poly.label=label;
                        poly_list.push_back(poly);
                        poly_list[poly_list.size()-1]=creat_poligon_area(poly_list[poly_list.size()-1]);
                        poly_list[poly_list.size()-1].center=get_poligon_center(poly_list[poly_list.size()-1].poligon_points);

                    }else if(cheek){
                        for(int p=0;p<poly.sidesIndex.size();p++){
                            oplist[poly.sidesIndex[p]].parent_poligon=-1;
                        }
                    }
                }
            }
        }
        //remove polygons tagged for removal (polygons with inactive==true)
        for(int b=0; b<poly_list.size(); b++){
            if(poly_list[b].inactiv){
                for(int v=0;v<oplist.size();v++){
                    if(oplist[v].label>10) continue;
                    if(oplist[v].parent_poligon>b) oplist[v].parent_poligon--;
                }
                poly_list.erase(poly_list.begin()+b);
                b-=1;
            }
        }
        return poly_list;
    }

    vector<poligon> optimize_intersection_openings(vector<poligon> poly_list){
        int sLenght=6000;
        for(int pIndex=0;pIndex<poly_list.size();pIndex++){
            if(poly_list[pIndex].inactiv) continue;
            for(int sideIndex=0;sideIndex<poly_list[pIndex].sidesIndex.size();sideIndex++){
                int opIndex=poly_list[pIndex].sidesIndex[sideIndex];
                //if(oplist[opIndex].label!=2) continue;
                vector<point_int> startS, endS;
                for(int sids=0;sids<2;sids++){
                    for(int dir=0; dir<2;dir++){
                        ant_data step;
                        step.end=sids?oplist[opIndex].end:oplist[opIndex].start;
                        point_int p2=sids?oplist[opIndex].start:oplist[opIndex].end;
                        bool cw=!(dir==sids);
                        if(!dir){
                            if(!sids){
                                startS.push_back(step.end);
                            }else{
                                endS.push_back(step.end);
                            }
                        }
                        vector<int> indexListTpoint=check_and_get_all_opening(step.end,3,false,opIndex);
                        vector<int> indexListTp2=check_and_get_all_opening(p2,3,false,opIndex);
                        bool test=false;
                        for(int i1=0;i1<indexListTpoint.size() && !test;i1++){
                            for(int i2=0;i2<indexListTp2.size() && !test;i2++){
                                if(indexListTpoint[i1]==indexListTp2[i2]){
                                    test=true;
                                    break;
                                }
                            }
                        }
                        if(test && !dir) break;
                        if(!test && dir && indexListTpoint.size()!=0)break;

                        for(int s=0;s<sLenght;s++){
                            step=ant_step(step.end,cw,step.dir,topMap);
                            if(check_for_opening(step.end,3)) break;
                            if(step.emty_cell) continue;
                            if(!sids){
                                if(!dir){
                                    startS.push_back(step.end);
                                }else{
                                    startS.insert(startS.begin(),step.end);
                                }
                            }else{
                                if(!dir){
                                    endS.push_back(step.end);
                                }else{
                                    endS.insert(endS.begin(),step.end);
                                }
                            }
                        }
                    }
                }
                opening test, hbest, best;
                test=oplist[opIndex];
                best=test;
                hbest=test;
                double bestScore=0, bestLength=0;
                bool first=true, changed=false;
                int sIndex=0, eIndex=0, decrisCount=0;
                while(true){
                    //ROS_INFO("%i, %i, %i, %i",startS[sI].x,startS[sI].y,endS[eI].x,endS[eI].y);
                    if(sIndex+1<startS.size() && (eIndex+1>=endS.size() || 
                       dist(startS[sIndex+1],endS[eIndex])<dist(startS[sIndex],endS[eIndex+1]))){
                        sIndex+=1;
                    }else if(eIndex+1<endS.size()){
                        eIndex+=1;
                    }else{
                        best=hbest;
                        changed=true;
                        break;
                    }
                    test.start=startS[sIndex];
                    test.end=endS[eIndex];
                    double lenght=dist(test.start,test.end);
                    if(lenght<minGroupSize) break;
                    if(first || lenght<bestLength){
                        bestLength=lenght;
                    }else{
                        if(decrisCount>=4){
                            best=hbest;
                            changed=true;
                            decrisCount=0;
                        }else decrisCount+=1;
                    }
                    double score=lenght+dw*(sIndex+eIndex)/2; //dist(test.get_center(),poly_list[pIndex].center);
                    if(first || score<bestScore){
                        first=false;
                        bestScore=score;
                        hbest=test;
                    }
                    
                }
                //ROS_INFO("%i, %i, %i, %i",best.start.x,best.start.y,best.end.x,best.end.y);
                if(!checkForWall(best,1,topMap))
                oplist[opIndex]=best;
                if(!changed){
                    if(poly_list[pIndex].sidesIndex.size()<=3){
                        poly_list=remove_parent_poligon(poly_list,opIndex, true, true);
                    }else{
                        oplist[opIndex].parent_poligon=-1;
                        oplist[opIndex].label=26;
                        poly_list[pIndex].sidesIndex.erase(poly_list[pIndex].sidesIndex.begin()+sideIndex);
                        sideIndex=-1;
                    }
                }
            }
            poly_list[pIndex]=creat_poligon_area(poly_list[pIndex]);
            poly_list[pIndex].center=get_poligon_center(poly_list[pIndex].poligon_points);
        }
        //remove polygons tagged for removal (polygons with inactive==true)
        for(int b=0; b<poly_list.size(); b++){
            if(poly_list[b].inactiv){
                for(int v=0;v<oplist.size();v++){
                    if(oplist[v].label>10) continue;
                    if(oplist[v].parent_poligon>b) oplist[v].parent_poligon--;
                }
                poly_list.erase(poly_list.begin()+b);
                b-=1;
            }
        }
        return poly_list;
    }

    //Connects poligon connectingIndex to poligon that has opIndex as a child, conneting the connected poligon back to connectingIndex at posision connectOppeningPos
    vector<poligon>  connectPoligons(vector<poligon> poly_list,int opIndex,int connectingIndex, int connectOppeningPos){
        int opParentP=oplist[opIndex].parent_poligon;
        int opParentIndex=-1;
        for(int i=0; i<poly_list[opParentP].sidesIndex.size();i++){
            if(poly_list[opParentP].sidesIndex[i]==opIndex){
                opParentIndex=i;
                break;
            }                
        }
        if(poly_list[opParentP].connectedPoligons.size()<=opParentIndex){
            poly_list[opParentP].connectedPoligons.resize(opParentIndex+1);
        }
        poly_list[opParentP].connectedPoligons[opParentIndex]=connectingIndex;

        if(poly_list[connectingIndex].connectedPoligons.size()<=connectOppeningPos){
            poly_list[connectingIndex].connectedPoligons.resize(connectOppeningPos+1);
        }
        poly_list[connectingIndex].connectedPoligons[connectOppeningPos]=opParentP;
        return poly_list;
    }

    //Create path polygon between intersection
    vector<poligon> creatPathPoligons(vector<poligon> poly_list){
        for(int i=0; i<oplist.size(); i++){
            if(oplist[i].conected_to_path==-1 && oplist[i].parent_poligon>=0 && !poly_list[oplist[i].parent_poligon].inactiv){
                //set init. val. for all var:s needed.
                //poly is a temp variable holding the information that will be appended to poligon_list 
                poligon poly;
                poly.inactiv=false;
                poly.path=true;
                poly.label=1;
                poly.sidesIndex.push_back(i);
                oplist[i].conected_to_path=-2;//A temporary value used to indicate that the polygon is connected.
                ant_data step_info;      
                bool check=false;
                bool complet=false;
                int opIndex, pathType=0;
                vector<int> firstOpIndex;
                opening currentOpening=oplist[i];
                int countExtraPaths=0;
                poly.poligon_points=fillPoints(currentOpening.start,currentOpening.end,0.7,true);

                for(int sids=0; sids<2 && !check; sids++){
                    int max_emty_cells=0;
                    int empty_cell_count=0;
                    bool cw=sids==0;
                    if(cw){
                        step_info.end=currentOpening.end; 
                    }else{
                        step_info.end=currentOpening.start;
                    }
                    point_int firstPos=step_info.end;
                    poly.add_point_d(step_info.end,cw);
                    int rezCounter=0;
                    step_info.dir={0,0};
                    for(int s=0; s<=sercheLenthAntConect; s++){
                        poly.add_point(step_info.end,cw);
                        //don't save all point to save on resources
                        if(rezCounter>=poligonRezPath){
                            rezCounter=0;
                            poly.add_point_d(step_info.end,cw);
                        }else{
                            rezCounter+=1;
                        }
                        
                        /*if(step_info.end==firstPos){
                            check=true;
                            break;
                        }*/

                        vector<int> iList=check_and_get_all_opening(step_info.end,cw?1:2);
                        if(iList.size()>1 && s==0){
                            bool check=true;
                            for(int index=0;index<iList.size();index++){
                                if(cw && currentOpening.start==oplist[iList[index]].end||
                                  !cw && currentOpening.end==oplist[iList[index]].start){
                                    check=false;
                                    s++;
                                    opIndex=iList[index];
                                }
                            }
                            if(check){
                                opIndex=-1;
                            }
                        }else if(iList.size()>0){
                            opIndex=iList[0];
                        }else opIndex=-1;

                        if(opIndex!=-1&& s==0){
                            if(!(oplist[opIndex].start==currentOpening.end&&
                               oplist[opIndex].end==currentOpening.start)){
                                point_int p1,p2;
                                if(cw){
                                    p1=oplist[opIndex].end;
                                    p2=currentOpening.start;
                                }else{
                                    p2=oplist[opIndex].start;
                                    p1=currentOpening.end;
                                }
                                p1.x-=step_info.end.x;
                                p1.y-=step_info.end.y;
                                p2.x-=step_info.end.x;
                                p2.y-=step_info.end.y;

                                double ang1=atan2(p1.y, p1.x);
                                double ang2=atan2(p2.y, p2.x);
                                double ang=ang1-ang2;
                                if(abs(ang)>M_PI){
                                    double newAng=abs(ang)-M_PI*2;
                                    if(ang<0){
                                        ang=-newAng;
                                    }else ang=newAng;
                                }
                                if(s==0 && ang<0){
                                    opIndex=-1;
                                }
                            }
                        }
                        //Found connection
                        if(opIndex!=-1 || s==sercheLenthAntConect){
                            if(oplist[opIndex].parent_poligon>=0){
                                oplist[opIndex].conected_to_path=-2;
                                poly.add_point_d(step_info.end,cw);
                                if(cw){
                                    firstOpIndex=iList;
                                    if(opIndex==i){//opening found it self, no need to search other side 
                                        check=true;
                                        complet=true;
                                        if(max_emty_cells>maxAntGap){
                                            pathType=2;//opening led to an unexplored area
                                        }else{
                                            pathType=1;//dead end
                                        }
                                    }else if(max_emty_cells>maxAntGap && opIndex!=-1){
                                        pathType=5;
                                    }else if(s==sercheLenthAntConect){
                                        pathType=1;//dead end
                                    }
                                    break;
                                }else{
                                    if(opIndex!=-1){
                                        poly.sidesIndex.push_back(opIndex);
                                        vector<point_int> p=fillPoints(oplist[opIndex].end,oplist[opIndex].start,0.7,true);
                                        for(int n=0; n<p.size();n++) poly.add_point(p[n],cw);
                                    }
                                    bool test=false;
                                    for(int m=0; m<firstOpIndex.size();m++){
                                        if(firstOpIndex[m]==opIndex) test=true;
                                    }
                                    if(!test){
                                        //ROS_INFO("Warning, path with more than two connection!");
                                        pathType=4;
                                        if(opIndex!=-1){//loop is rerun to connect additional openings
                                            currentOpening=oplist[opIndex];
                                            if(countExtraPaths>4){
                                                complet=true;
                                            }else{
                                                countExtraPaths+=1;
                                                sids-=1;
                                            }
                                        }else{
                                            complet=true;
                                        }

                                    }else if(max_emty_cells>maxAntGap && opIndex!=-1 && pathType!=4){
                                        pathType=5;
                                        complet=true;
                                    }else if(pathType==0){
                                        pathType=3;
                                        complet=true;
                                    }else{
                                        complet=true;
                                    }
                                    break;
                                }
                            }
                        }
                        step_info=ant_step(step_info.end,cw,step_info.dir,topMap);
                        if(step_info.emty_cell){
                            empty_cell_count+=1;
                            if(empty_cell_count>max_emty_cells) max_emty_cells=empty_cell_count;
                        }else if(empty_cell_count>0){
                            empty_cell_count-=1;
                        }
                    }
                }
                if(complet){
                    int label=0;
                    switch (pathType)
                    {
                    case 1://room or dead end 
                        label=41;
                        break;

                    case 2://path leading to an unexplored area
                        label=52;
                        break;
                    
                    case 3://normal path
                        label=64;
                        break;

                    case 4://path connected to multiple intersection
                        label=76;
                        poly.center=get_poligon_center(poly.poligon_points);
                        break;

                    case 5://path with potensial unexplord area 
                        label=28;
                        break;

                    default:
                        break;
                    }
                    if(oplist[i].parent_poligon==0)
                    for(int p=0;p<poly.sidesIndex.size();p++){
                        oplist[poly.sidesIndex[p]].conected_to_path=poly_list.size();
                    }
                    poly.label=label;
                    poly_list.push_back(poly);
                    for(int m=0;m<poly.sidesIndex.size();m++){
                        //poly_list=connectPoligons(poly_list,poly.sidesIndex[m],poly_list.size()-1,m);
                    }
                }else if(check){
                    for(int p=0;p<poly.sidesIndex.size();p++){
                        oplist[poly.sidesIndex[p]].conected_to_path=-1;
                    }
                }
            }
        }
        return poly_list;
    }

    vector<point_int> generate_voronoi(poligon poly, point_int start, point_int end={-1,-1}){
        vector<point_int> PL=poly.poligon_points;
        vector<point_int> filledPoints=fillPoly(PL);
        for(int i=0;i<filledPoints.size();i++) 
            setMap(filledPoints[i].x,filledPoints[i].y,100,debugMap);
        point_int maxP={-1,-1},minP={-1,-1};
        for(int n=0;n<PL.size();n++){
            //setMap(PL[n].x,PL[n].y,0,debugMap);
            if(PL[n].x>maxP.x) maxP.x=PL[n].x;
            if(PL[n].y>maxP.y) maxP.y=PL[n].y;
            if(PL[n].x<minP.x || minP.x==-1) minP.x=PL[n].x;
            if(PL[n].y<minP.y || minP.y==-1) minP.y=PL[n].y;
        }
        setMap(start.x,start.y,100,debugMap);
        setMap(end.x,end.y,100,debugMap);
        maxP={maxP.x+1,maxP.y+1};
        minP={minP.x-1,minP.y-1};
        point_int P[]={{0,0},{-1,0},{-1,1},{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0}};
        bool check=false;
        while(!check){
            for(int sids=0;sids<2;sids++){
                vector<point_int> M;
                for(int i=minP.x;i<=maxP.x;i++){
                    for(int j=minP.y;j<=maxP.y;j++){
                        point_int rp={i,j};
                        if(getMap(i,j,debugMap)!=100 ||
                           rp==start || rp==end)continue;
                        int B=0;
                        for(int pIndex=1; pIndex<9;pIndex++){
                            if(getMap(i+P[pIndex].x,j+P[pIndex].y,debugMap)==100) B+=1;
                        }
                        if(B<2 || B>6) continue;

                        int A=0;
                        for(int pIndex=1; pIndex<9;pIndex++){
                            if(getMap(i+P[pIndex].x,j+P[pIndex].y,debugMap)!=100 &&
                               getMap(i+P[pIndex+1].x,j+P[pIndex+1].y,debugMap)==100) A+=1;
                        }
                        if(A!=1) continue;
                        int c[]={1,3,5};
                        int d[]={3,4,7};
                        if(sids==1){
                            c[0]=1; c[1]=3; c[2]=7;
                            d[0]=1; d[1]=5; d[2]=7;
                        }
                        bool t=true;
                        for(int cIndex=0;cIndex<3;cIndex++){
                            if(getMap(i+P[c[cIndex]].x,j+P[c[cIndex]].y,debugMap)!=100){
                                t=false;
                                break;
                            }
                        }
                        if(t) continue;

                        t=true;
                        for(int dIndex=0;dIndex<3;dIndex++){
                            if(getMap(i+P[d[dIndex]].x,j+P[d[dIndex]].y,debugMap)!=100){
                                t=false;
                                break;
                            }
                        }
                        if(t) continue;

                        M.push_back(rp);
                    }
                }
                if(M.size()==0){
                    check=true;
                    break;
                }else{
                    for(int i=0;i<M.size();i++){
                        setMap(M[i].x,M[i].y,-1,debugMap);
                    }
                }
            }
        }

        //Get voronoi path
        point_int P2[]={{-1,0},{0,1},{1,0},{0,-1},{-1,1},{1,1},{1,-1},{-1,-1}};
        vector<vector<point_int>> paths;
        paths.resize(1);
        int cPathIndex=0;
        point_int curentPoint=start, newPoint=start;
        paths[cPathIndex].push_back(start);
        check=false;
        while (!check){
            setMap(curentPoint.x,curentPoint.y,0,debugMap);
            int c=0;
            bool flip=true;
            for(int pIndex=0;pIndex<8;pIndex++){
                if(curentPoint.x+P2[pIndex].x==end.x && curentPoint.y+P2[pIndex].y==end.y){
                    newPoint={curentPoint.x+P2[pIndex].x,curentPoint.y+P2[pIndex].y};
                    paths[cPathIndex].push_back(newPoint);
                    check=true;
                    break;
                }
                if(getMap(curentPoint.x+P2[pIndex].x,curentPoint.y+P2[pIndex].y,topMap)==100) continue;
                if(getMap(curentPoint.x+P2[pIndex].x,curentPoint.y+P2[pIndex].y,debugMap)==100){
                    if(pIndex>=4){
                        if(getMap(curentPoint.x+P2[(pIndex+4)%8].x,curentPoint.y+P2[(pIndex+4)%8].y,topMap)==100 &&
                           getMap(curentPoint.x+P2[(pIndex+5)%8].x,curentPoint.y+P2[(pIndex+5)%8].y,topMap)==100) continue;
                    }
                    c+=1;
                    if(flip) newPoint={curentPoint.x+P2[pIndex].x,curentPoint.y+P2[pIndex].y};
                    flip=false;
                }
            }
            if(check) break;

            if(c==0){
                if(cPathIndex-1<0){
                    int maxS=-1;
                    for (int p=0; p<paths.size(); p++){
                        if(int(paths[p].size())>maxS){
                            maxS=paths[p].size();
                            cPathIndex=p;
                        }
                    }
                    check=true;
                    break;
                    
                }else{
                    cPathIndex-=1;
                    curentPoint=paths[cPathIndex][paths[cPathIndex].size()-1];
                    continue;
                }
            }else if(c==1){
                paths[cPathIndex].push_back(newPoint);
            }else{
                paths.push_back(paths[cPathIndex]);
                cPathIndex=paths.size()-1;
                paths[cPathIndex].push_back(newPoint);
            }
            curentPoint=newPoint;
        }
        for(int x=minP.x;x<=maxP.x;x++){
            for(int y=minP.y;y<=maxP.y;y++){
                if(getMap(x,y,debugMap)==100){
                    setMap(x,y,0,debugMap);
                }
            }
        }
        return paths[cPathIndex];
    }

    vector<poligon> generat_robot_path(vector<poligon> poly_list){
        for(int x=0; x<mapSizeX;x++){
            for(int y=0;y<mapSizeY;y++){
                debugMap[x][y]=-1;
            }
        }
        for(int polyIndex=0; polyIndex<poly_list.size(); polyIndex++){
            if(poly_list[polyIndex].inactiv) continue;
            for(int sideIndex=0; sideIndex<poly_list[polyIndex].sidesIndex.size();sideIndex++){
                int label=poly_list[polyIndex].label;
                int opIndex=poly_list[polyIndex].sidesIndex[sideIndex];
                point_int center=poly_list[polyIndex].center;
                point_int oCenter=oplist[opIndex].get_center();
                opening o, o2;
                o.start=oCenter;
                o.end=center;
                o2=o;
                if(label==64||label==28){
                    o2.end=oplist[poly_list[polyIndex].sidesIndex[sideIndex+1]].get_center();
                }
                vector<point_int> np;
                if((label==30||label==76)&&!checkForWall(o,1,topMap)){
                    np.push_back(oCenter);
                    np.push_back(center);
                }else if((label==64||label==28)&&!checkForWall(o2,1,topMap)){
                    np.push_back(o2.start);
                    np.push_back(o2.end);
                    sideIndex+=1;
                }else{
                    vector<point_int> vp;
                    if(label==30||label==76){
                        vp=generate_voronoi(poly_list[polyIndex],oCenter,center);
                    }else if(label==41||label==52){
                        vp=generate_voronoi(poly_list[polyIndex],oCenter);
                    }else{
                        vp=generate_voronoi(poly_list[polyIndex],oCenter,oplist[poly_list[polyIndex].sidesIndex[sideIndex+1]].get_center());
                        sideIndex+=1;
                    }
                    bool toC=false;
                    for(int n=0;n<vp.size();n+=voronoiRez){
                        np.push_back(vp[n]);
                        o.start=vp[n];
                        if((label==30||label==76) && !checkForWall(o,1,topMap)){
                            toC=true;
                            np.push_back(center);
                            break;
                        }
                    }
                    if(!toC) np.push_back(vp[vp.size()-1]);
                    
                }
                poly_list[polyIndex].connectedPaths.push_back(np);
                robotPath.push_back(np);
            }
        }
        return poly_list;
    }

    topology_mapping::point2D_intList ConvertPointListToMsg(vector<point_int> list){
        topology_mapping::point2D_intList newList;
        int size=list.size();
        newList.list.resize(size);
        for(int i=0;i<size;i++){
            newList.list[i].x=list[i].x;
            newList.list[i].y=list[i].y;
        }
        return newList;
    }

    //Function to publish all topics. 
    void pubMap(vector<poligon> poly_list){
        jsk_recognition_msgs::PolygonArray pubPolyArray_old;
        pubPolyArray_old.header.frame_id = "map";
        pubPolyArray_old.header.stamp = ros::Time::now();
        pubPolyArray_old.polygons.resize(oplist.size());
        pubPolyArray_old.labels.resize(oplist.size());
        pubPolyArray_old.likelihood.resize(oplist.size());
        int MapOrigenX=-mapOffsetX/resolution;
        int MapOrigenY=-mapOffsetY/resolution;
        for(int i=0; i<oplist.size(); i++){
            //convert the line of an opening to a rectangular polygon
            opening op=oplist[i];
            point side={(double)(op.end.x-op.start.x)*resolution,
                        (double)(op.end.y-op.start.y)*resolution};
            
            double l=sqrt(abs(side.x*side.x+side.y*side.y));
            if(l<0.01) continue;
            point nSide={side.x/(16*l),side.y/(16*l)};
            point nNorm={-nSide.y, nSide.x};
            geometry_msgs::PolygonStamped p;
            p.header.frame_id = "map";
            p.header.stamp = ros::Time::now();
            p.polygon.points.resize(7);
            topMap[op.start.x][op.start.y]=100;
            topMap[op.end.x][op.end.y]=50;
            p.polygon.points[0].x=(op.start.x-MapOrigenX)*resolution;
            p.polygon.points[0].y=(op.start.y-MapOrigenY)*resolution;
            p.polygon.points[0].z=0.2;

            p.polygon.points[1].x=(op.start.x-MapOrigenX)*resolution+side.x/2-nSide.x;
            p.polygon.points[1].y=(op.start.y-MapOrigenY)*resolution+side.y/2-nSide.y;
            p.polygon.points[1].z=0.2;

            p.polygon.points[2].x=(op.start.x-MapOrigenX)*resolution+side.x/2+2*nNorm.x;
            p.polygon.points[2].y=(op.start.y-MapOrigenY)*resolution+side.y/2+2*nNorm.y;
            p.polygon.points[2].z=0.2;

            p.polygon.points[3].x=(op.start.x-MapOrigenX)*resolution+side.x/2+nSide.x;
            p.polygon.points[3].y=(op.start.y-MapOrigenY)*resolution+side.y/2+nSide.y;
            p.polygon.points[3].z=0.2;

            p.polygon.points[4].x=(op.end.x-MapOrigenX)*resolution;
            p.polygon.points[4].y=(op.end.y-MapOrigenY)*resolution;
            p.polygon.points[4].z=0.2;
            
            p.polygon.points[5].x=(op.end.x-MapOrigenX)*resolution-nNorm.x;
            p.polygon.points[5].y=(op.end.y-MapOrigenY)*resolution-nNorm.y;
            p.polygon.points[5].z=0.2;

            p.polygon.points[6].x=(op.start.x-MapOrigenX)*resolution-nNorm.x;
            p.polygon.points[6].y=(op.start.y-MapOrigenY)*resolution-nNorm.y;
            p.polygon.points[6].z=0.2;
            
            pubPolyArray_old.polygons[i]=p;
            pubPolyArray_old.labels[i]=op.label;
            pubPolyArray_old.likelihood[i]=op.parent_poligon;
        }
        pubTopoPoly_debug.publish(pubPolyArray_old);
        
        jsk_recognition_msgs::PolygonArray pubPolyArray;
        pubPolyArray.header.frame_id = "map";
        pubPolyArray.header.stamp = ros::Time::now();
        pubPolyArray.polygons.resize(poly_list.size());
        pubPolyArray.labels.resize(poly_list.size());
        pubPolyArray.likelihood.resize(poly_list.size());
        int c=0;
        for(int i=0; i<poly_list.size(); i++){
            pubPolyArray.polygons[i]=get_polygon(poly_list[i],resolution);
            pubPolyArray.labels[i]=poly_list[i].label;
            if(poly_list[i].label==30 || poly_list[i].label==41 || poly_list[i].label==76 || poly_list[i].label==52) c++;
            pubPolyArray.likelihood[i]=1;
        }
        if(c!=oldNodCount){
            oldNodCount=c;
            ROS_INFO("Node count: %i",c);
        }
        pubTopoPoly.publish(pubPolyArray);
        for(int i=0; i<robotPath.size();i++){
            if(robotPath[i].size()<2){
                robotPath.erase(robotPath.begin()+i);
                i-=1;
            }
        }
        visualization_msgs::MarkerArray msgRobotPath;
        msgRobotPath.markers.resize(1+robotPath.size());
        msgRobotPath.markers[0].header.frame_id = "map";
        msgRobotPath.markers[0].header.stamp = ros::Time::now();
        msgRobotPath.markers[0].action=msgRobotPath.markers[0].DELETEALL;
        for(int i=1; i<robotPath.size()+1;i++){
            msgRobotPath.markers[i].header.frame_id = "map";
            msgRobotPath.markers[i].header.stamp = ros::Time::now();
            msgRobotPath.markers[i].ns="robotPath";
            msgRobotPath.markers[i].id=i;
            msgRobotPath.markers[i].type=msgRobotPath.markers[i].LINE_STRIP;
            msgRobotPath.markers[i].action=msgRobotPath.markers[i].ADD;

            msgRobotPath.markers[i].pose.position.x=0;
            msgRobotPath.markers[i].pose.position.y=0;
            msgRobotPath.markers[i].pose.position.z=mapHight+0.05;
            msgRobotPath.markers[i].pose.orientation.w=1.0;
            msgRobotPath.markers[i].pose.orientation.x=0.0;
            msgRobotPath.markers[i].pose.orientation.y=0.0;
            msgRobotPath.markers[i].pose.orientation.z=0.0;
            msgRobotPath.markers[i].scale.x=0.15;
            msgRobotPath.markers[i].scale.y=0.1;
            msgRobotPath.markers[i].scale.z=0.1;
            msgRobotPath.markers[i].color.a=1.0;
            msgRobotPath.markers[i].color.r=1.0;
            msgRobotPath.markers[i].color.b=0.0;
            msgRobotPath.markers[i].color.g=0.0;
            msgRobotPath.markers[i].points.resize(robotPath[i-1].size());
            msgRobotPath.markers[i].lifetime=ros::Duration(0);
            for(int m=0; m<robotPath[i-1].size();m++){
                msgRobotPath.markers[i].points[m].x=(robotPath[i-1][m].x-MapOrigenX)*resolution;
                msgRobotPath.markers[i].points[m].y=(robotPath[i-1][m].y-MapOrigenY)*resolution;
                msgRobotPath.markers[i].points[m].z=0;
            }
        }
        
        pubRobotPath.publish(msgRobotPath);
        
        topoMapMsg.header.stamp = ros::Time::now();
        topometricMapMsg.header=topoMapMsg.header;
        topometricMapMsg.info=topoMapMsg.info;
        topometricMapMsg.polygonType.resize(topoMapMsg.data.size());
        topometricMapMsg.polygonId.resize(topoMapMsg.data.size());
        topometricMapMsg.polygons.resize(poly_list.size());
        for(int i=0;i<topoMapMsg.data.size();i++){
            topometricMapMsg.polygonType[i]=-1;
            topometricMapMsg.polygonId[i]=-1;
            topoMapMsg.data[i]=-1;
        }
        for(int i=0; i<poly_list.size();i++){
            vector<point_int> filedP;//=fillPoly(poly_list[i].poligon_points);
            for(int j=0;j<filedP.size();j++){
                int index=filedP[j].x+filedP[j].y*mapSizeX;
                topometricMapMsg.polygonType[index]=poly_list[i].label;
                topometricMapMsg.polygonId[index]=i;
                topoMapMsg.data[index]=poly_list[i].label;
            }
            topometricMapMsg.polygons[i].polygonPoints=ConvertPointListToMsg(poly_list[i].poligon_points);
            int size=poly_list[i].connectedPaths.size();
            topometricMapMsg.polygons[i].connectedPaths.resize(size);
            for(int n=0;n<size;n++){
                topometricMapMsg.polygons[i].connectedPaths[n]=ConvertPointListToMsg(poly_list[i].connectedPaths[n]);
            }
            topometricMapMsg.polygons[i].id=i;
            topometricMapMsg.polygons[i].type=poly_list[i].label;
            topometricMapMsg.polygons[i].connectedPolygons=poly_list[i].connectedPoligons;

        }
        pubMapDebug.publish(topoMapMsg);
        pubTopometricMap.publish(topometricMapMsg);
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "topology_mapping");
    
    TopologyMapping topMapping;

    ROS_INFO("Topology Mapping Started.");
    bool done=false;
    while(!done){
        try{
            topMapping.spin();
            done=true;
        }
        catch(...){
            ROS_INFO("rutin stopt restaring...");
        }

    }
    
    
    return 0;
}