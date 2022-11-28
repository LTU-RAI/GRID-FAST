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
        ros::Publisher pubRobotPath;
        ros::Publisher pubMarkDel;

        //msg
        nav_msgs::OccupancyGrid topoMapMsg;
        visualization_msgs::Marker markDel;


        //Map
        int **topMap;
        int **debugMap;
        Poligon_list *poly_list=NULL;
        poligon poly;
        
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
        topoMapMsg.info.origin.position.z = mapHight+1; //for visualization

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
                //Clear poly_list
                if(poly_list!=NULL){
                    delete poly_list;
                }
                poly_list=new Poligon_list;
                robotPath.clear();
                if(oplist.size()>0){
                    creatPoligonList();

                    if(poly_list->size()>0){;;
                        creatPathPoligons();
                        generat_robot_path();
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
                    //remove polygons tagged for removal (polygons with inactive==true)
                    for(int b=0; b<poly_list->poly.size(); b++){
                        if(poly_list->poly[b].inactiv){
                            poly_list->poly.erase(poly_list->poly.begin()+b);
                            poly_list->label.erase(poly_list->label.begin()+b);
                            b-=1;
                        }
                    }
                }
                pubMap();
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

    void creat_poligon_area(int polyIndex){
        vector<point_int> points;
        vector<point_int> points_desplay;
        int oIndex=poly_list->poly[polyIndex].sidesIndex[0];
        for(int sIndex=0;sIndex<poly_list->poly[polyIndex].sidesIndex.size();sIndex++){
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
                for(int i=0;i<poly_list->poly[polyIndex].sidesIndex.size();i++){
                    if(step_info.end==oplist[poly_list->poly[polyIndex].sidesIndex[i]].end){
                        newOIndex=poly_list->poly[polyIndex].sidesIndex[i];
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
        poly_list->poly[polyIndex].poligon_points=points;
        poly_list->poly[polyIndex].poligon_points_desplay=points_desplay;
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
    void remove_parent_poligon(int opIndex, bool remove_openign=false){
        if(opIndex>=0){
            if(oplist[opIndex].parent_poligon>=0){
                int pIndex=oplist[opIndex].parent_poligon;
                for(int i=0; i<poly_list->poly[pIndex].sidesIndex.size();i++){
                    oplist[poly_list->poly[pIndex].sidesIndex[i]].parent_poligon=-1;
                    poly_list->poly[pIndex].inactiv=true;
                }
            }
            if(remove_openign){
                oplist[opIndex].label=26;
            }
        }
    }

    //Connect all openings to create a list of polygons representing the intersections.
    void creatPoligonList(){
        for(int i=0; i<oplist.size(); i++){
            if(oplist[i].parent_poligon==-1 && oplist[i].label==1){
                //set init. val. for all var:s needed.
                //poly is a temp variable holding the information that will be appended to poligon_list 
                bool cw=true;
                poly.poligon_points.clear();
                poly.poligon_points_desplay.clear();
                poly.sidesIndex.clear();
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
                int indexOpeningBlocking=-1;
                int totalSlenght=0;
                int opIndex;

                while(!cheek){
                    int empty_cell_count=0;
                    if(cw){
                        step_info.end=oplist[targetIndex].start; 
                    }else{
                        step_info.end=oplist[targetIndex].end;
                    }
                    int rezCounter=0;
                    step_info.dir={0,0};
                    for(int s=0; s<=sercheLenthAntConect; s++){
                        
                        opIndex=check_and_get_opening(step_info.end,cw?1:2,true);
                        
                        // found bad conection
                        if((opIndex!=-1 && opIndex!=targetIndex) ||
                            empty_cell_count>maxAntGap || s==sercheLenthAntConect){
                                if(cw){
                                    //ROS_INFO("found no connection searching other direction");
                                    lastIndex=targetIndex;
                                    targetIndex=firstIndex;
                                    indexOpeningBlocking=opIndex;
                                    cw=false;
                                    break;
                                }else{
                                    if(poly.sidesIndex.size()>1){
                                        if(opIndex!=-1 && opIndex==indexOpeningBlocking && false){
                                            if(!moved && oplist[opIndex].parent_poligon<0){
                                                for(int h=0; h<poly.sidesIndex.size();h++){
                                                    oplist[poly.sidesIndex[h]].moved=true;
                                                    oplist[poly.sidesIndex[h]].parent_poligon=-1;
                                                    oplist.push_back(oplist[poly.sidesIndex[h]]);
                                                    oplist[poly.sidesIndex[h]].label=14;
                                                }
                                                i-=1;
                                                cheek=true;
                                                break;
                                            }else if(oplist[opIndex].parent_poligon>=0){
                                                vector<int> oIndexList;
                                                vector<point_int> oPointList;
                                                for(int h=0; h<poly.sidesIndex.size();h++){
                                                    oIndexList.push_back(poly.sidesIndex[h]);
                                                    oPointList.push_back(oplist[poly.sidesIndex[h]].start);
                                                    oPointList.push_back(oplist[poly.sidesIndex[h]].end);
                                                }
                                                for(int h=0; h<poly_list->poly[oplist[opIndex].parent_poligon].sidesIndex.size();h++){
                                                    if(poly_list->poly[oplist[opIndex].parent_poligon].sidesIndex[h]!=opIndex){
                                                        oIndexList.push_back(poly_list->poly[oplist[opIndex].parent_poligon].sidesIndex[h]);
                                                        oPointList.push_back(oplist[poly_list->poly[oplist[opIndex].parent_poligon].sidesIndex[h]].start);
                                                        oPointList.push_back(oplist[poly_list->poly[oplist[opIndex].parent_poligon].sidesIndex[h]].end);
                                                    }
                                                }
                                                point_int center=get_poligon_center(oPointList);
                                                bool t=!testIntersection(oIndexList,center);
                                                if(!t){
                                                    remove_parent_poligon(opIndex,true);
                                                    i-=1;
                                                    cheek=true;
                                                    break;
                                                }
                                            }
                                        }

                                        //ROS_INFO("Create new opening");
                                        //creat mising openign
                                        opening newOp;
                                        newOp.start=oplist[targetIndex].end;
                                        newOp.end=oplist[lastIndex].start;
                                        //fitt new opening to coridor get s1 and s2 that is the points that the opening was moved 
                                        fitToCorridor2(&newOp,40,topMap,true,true);
                                        newOp.label=2;
                                        newOp.parent_poligon=-2;
                                        poly.add_sideIndex(oplist.size(),cw);
                                        int newIndex=oplist.size();
                                        oplist.push_back(newOp);
                                        //check if new opening is good 
                                        if(!checkForWall(newOp,1,topMap) && dist(newOp.start,newOp.end)>=minGroupSize){
                                            complet=true;

                                        }else{//bad new opening, remove conflicting opening instead
                                            oplist[newIndex].parent_poligon=-1;
                                            oplist[newIndex].label=11;
                                            if(opIndex>=0){
                                                remove_parent_poligon(opIndex,true);
                                                i-=1;
                                            }        
                                        }
                                    }
                                    
                                    //ROS_INFO("Found no connection on searching");
                                    cheek=true;
                                    break;
                                }
                        }
                        
                        opIndex=check_and_get_opening(step_info.end,cw?2:1,true);

                        //Good conection
                        if(opIndex!=-1){
                                totalSlenght+=s;
                                //If an opening filip has caused a new connection possible with a polygon that have created a new opening remov that poligon
                                if(oplist[opIndex].parent_poligon!=-1 && oplist[opIndex].parent_poligon!=-2){
                                    remove_parent_poligon(opIndex);
                                }
                                if(oplist[opIndex].parent_poligon==-1){
                                    //ROS_INFO("Found connection");
                                    oplist[opIndex].parent_poligon=-2;
                                    poly.add_sideIndex(opIndex,cw);
                                    moved=moved?moved:oplist[opIndex].moved;
                                    targetIndex=opIndex;
                                    break;
                                }else if(poly.sidesIndex.size()==2){//&& totalSlenght<=minimumSercheLenght){//Too small connection, flipping the openings
                                    //ROS_INFO("Small connection");
                                    oplist[targetIndex].parent_poligon=-1;
                                    if(oplist[i].fliped){//remove opening if flipped two times
                                        oplist[i].label=28;
                                    }else{
                                        oplist[i].parent_poligon=-1;
                                        oplist[targetIndex].flip();
                                        oplist[i].flip();
                                        oplist[i].fliped=true;
                                        
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
                                    if(poly.sidesIndex.size()==1 && s<dist(oplist[i].start,oplist[i].end)*2) break;
                                    complet=true;
                                    break;
                                }
                        }

                        step_info=ant_step(step_info.end,cw,step_info.dir,topMap);

                        if(step_info.emty_cell){
                            empty_cell_count+=1;
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
                            oplist[poly.sidesIndex[p]].parent_poligon=poly_list->poly.size();
                        }
                        poly_list->add(poly,label);
                        creat_poligon_area(poly_list->size()-1);
                        poly_list->poly[poly_list->size()-1].center=get_poligon_center(poly_list->poly[poly_list->size()-1].poligon_points);

                    }else if(cheek){
                        for(int p=0;p<poly.sidesIndex.size();p++){
                            oplist[poly.sidesIndex[p]].parent_poligon=-1;
                        }
                    }
                }
            }
        }
    }

    //Create path polygon between intersection
    void creatPathPoligons(){
        for(int i=0; i<oplist.size(); i++){
            if(oplist[i].conected_to_path==-1 && oplist[i].parent_poligon>=0 && !poly_list->poly[oplist[i].parent_poligon].inactiv){
                //set init. val. for all var:s needed.
                //poly is a temp variable holding the information that will be appended to poligon_list 
                poly.poligon_points.clear();
                poly.poligon_points_desplay.clear();
                poly.sidesIndex.clear();
                poly.inactiv=false;
                poly.path=true;
                poly.label=1;
                poly.sidesIndex.push_back(i);
                oplist[i].conected_to_path=-2;//A temporary value used to indicate that the polygon is connected.
                ant_data step_info;      
                bool check=false;
                bool complet=false;
                int opIndex, firstOpIndex=-2, pathType=0;
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
                        step_info=ant_step(step_info.end,cw,step_info.dir,topMap);

                        if(step_info.end==firstPos){
                            check=true;
                            break;
                        }

                        opIndex=check_and_get_opening(step_info.end,cw?1:2);
                        
                        //Found connection
                        if(opIndex!=-1 || s==sercheLenthAntConect){
                            if(oplist[opIndex].parent_poligon>=0){
                                oplist[opIndex].conected_to_path=-2;
                                poly.add_point_d(step_info.end,cw);

                                if(cw){
                                    firstOpIndex=opIndex;
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
                                    if(opIndex!=firstOpIndex){
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
                    for(int p=0;p<poly.sidesIndex.size();p++){
                        oplist[poly.sidesIndex[p]].conected_to_path=poly_list->poly.size();
                    }
                    poly_list->add(poly,label);
                }else if(check){
                    for(int p=0;p<poly.sidesIndex.size();p++){
                        oplist[poly.sidesIndex[p]].conected_to_path=-1;
                    }
                }
            }
        }
    }

    vector<point_int> generate_voronoi(int pIndex, point_int start, point_int end={-1,-1}){
        vector<point_int> PL=poly_list->poly[pIndex].poligon_points;
        fillPoly(PL,100,debugMap);
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

    void generat_robot_path(){
        for(int x=0; x<mapSizeX;x++){
            for(int y=0;y<mapSizeY;y++){
                debugMap[x][y]=-1;
            }
        }
        for(int polyIndex=0; polyIndex<poly_list->size(); polyIndex++){
            if(poly_list->poly[polyIndex].inactiv) continue;
            for(int sideIndex=0; sideIndex<poly_list->poly[polyIndex].sidesIndex.size();sideIndex++){
                int label=poly_list->label[polyIndex];
                int opIndex=poly_list->poly[polyIndex].sidesIndex[sideIndex];
                point_int center=poly_list->poly[polyIndex].center;
                point_int oCenter=oplist[opIndex].get_center();
                opening o, o2;
                o.start=oCenter;
                o.end=center;
                o2=o;
                if(label==64||label==28){
                    o2.end=oplist[poly_list->poly[polyIndex].sidesIndex[sideIndex+1]].get_center();
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
                        vp=generate_voronoi(polyIndex,oCenter,center);
                    }else if(label==41||label==52){
                        vp=generate_voronoi(polyIndex,oCenter);
                    }else{
                        vp=generate_voronoi(polyIndex,oCenter,oplist[poly_list->poly[polyIndex].sidesIndex[sideIndex+1]].get_center());
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
                robotPath.push_back(np);
            }
        }
    }


    //Function to publish all topics. 
    void pubMap(){
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
        pubPolyArray.polygons.resize(poly_list->size());
        pubPolyArray.labels.resize(poly_list->size());
        pubPolyArray.likelihood.resize(poly_list->size());
        int c=0;
        for(int i=0; i<poly_list->size(); i++){
            pubPolyArray.polygons[i]=poly_list->get_polygon(i,resolution);
            pubPolyArray.labels[i]=poly_list->label[i];
            if(poly_list->label[i]==30 || poly_list->label[i]==41 || poly_list->label[i]==76 || poly_list->label[i]==52) c++;
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
        for(int y=0; y<mapSizeY;y++){
            for(int x=0;x<mapSizeX;x++){  
                int index=x+y*mapSizeX;            
                topoMapMsg.data[index]=debugMap[x][y];
            }
        }
        pubMapDebug.publish(topoMapMsg);
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "topology_mapping");
    
    TopologyMapping topMapping;

    ROS_INFO("Topology Mapping Started.");
    
    topMapping.spin();
    
    return 0;
}