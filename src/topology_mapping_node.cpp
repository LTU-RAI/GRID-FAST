#include "Utility.h"                                

class TopologyMapping{
    private:
        //ROS nh
        ros::NodeHandle nh;
        //subs
        ros::Subscriber subOccupancyMap;
        ros::Subscriber subOpeningList;
        //pub
        ros::Publisher pubTopoPoly_debug;
        ros::Publisher pubTopoPoly;


        //Map
        int **topMap;
        Poligon_list *poly_list=NULL;
        poligon poly;
        
        //Global var.
        bool rMap=false, rOpening=false;


    public:
        //Setup
        TopologyMapping(){
            subOccupancyMap= nh.subscribe("/topology_map_filterd",1,&TopologyMapping::updateMap, this);
            subOpeningList= nh.subscribe("/opening_list_int",1,&TopologyMapping::updateOplist, this);
            pubTopoPoly_debug=nh.advertise<jsk_recognition_msgs::PolygonArray>("/topology_poly_opening",5);
            pubTopoPoly=nh.advertise<jsk_recognition_msgs::PolygonArray>("/topology_poly",5);
            loadMemory();
        }
        
    //Load all maps into the memory
    void loadMemory(){
        oplist.reserve(400);
        //alocate for Maps
        topMap=new int*[mapSize];
        for(int i=0;i<mapSize;i++){
            topMap[i]=new int[mapSize];
        }
        
        for (int i = 0; i < mapSize; ++i){
            for (int j = 0; j < mapSize; ++j){
                topMap[i][j] = -1;
            }
        }
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
                
                if(oplist.size()>0){
                    creatPoligonList();

                    if(poly_list->size()>0){
                        creatPathPoligons();
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
        //int mapResolution=mapMsg.occupancy.info.resolution;
        for(int x=0; x<width;x++){
            for(int y=0; y<height;y++){
                int index= x+y*width;
                topMap[x][y]=mapMsg.data[index];
            }
        }
        rMap=true;
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
                poly.sidesIndex.clear();
                poly.inactiv=false;
                poly.label=1;
                poly.sidesIndex.push_back(i);
                oplist[i].parent_poligon=-2;//A temporary value used to indicate that the polygon is connected.
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
                        
                        //don't save all point to save on resources
                        if(rezCounter>=poligonRez){
                            rezCounter=0;
                            poly_points.push_back(step_info.end);
                        }else{
                            rezCounter+=1;
                        }
                        
                        opIndex=check_and_get_opening(step_info.end,cw?1:2,true);
                        
                        // found bad conection
                        if((opIndex!=-1 && opIndex!=targetIndex) ||
                            empty_cell_count>maxAntGap || s==sercheLenthAntConect){
                                if(cw){
                                    wait=s;

                                    ROS_INFO("found no connection searching other direction");
                                    lastIndex=targetIndex;
                                    targetIndex=firstIndex;
                                    cw=false;
                                    break;
                                }else{
                                    if(poly.sidesIndex.size()>1){
                                        ROS_INFO("Create new opening");
                                        //creat mising openign
                                        opening newOp;
                                        newOp.start=oplist[targetIndex].end;
                                        newOp.end=oplist[lastIndex].start;
                                        //fitt new opening to coridor get s1 and s2 that is the points that the opening was moved 
                                        vector<point_int> s1, s2;
                                        fitToCorridor(&newOp,40,topMap,true,true,&s1,&s2);
                                        newOp.label=2;
                                        newOp.parent_poligon=-2;
                                        poly.sidesIndex.push_back(oplist.size());
                                        int newIndex=oplist.size();
                                        oplist.push_back(newOp);
                                        //check if new opening is good 
                                        if(!checkForWall(newOp,1,topMap) && dist(newOp.start,newOp.end)>=minGroupSize){
                                            for(int k=0; k<s2.size();k+=poligonRez){
                                                poly.add_point(s2[k],cw);
                                            }
                                            if(s2.size()>0) poly.add_point(newOp.end,cw);
                                            if(s1.size()>0) poly.add_point(newOp.start,cw);
                                            for(int k=s1.size()-1; k>0;k-=poligonRez){
                                                poly.add_point(s1[k],cw);
                                            }
                                    
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
                                    
                                    ROS_INFO("Found no connection on searching");
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
                                //If an opening filip has caused a new connection possible with a polygon that have created a new opening remov that poligon
                                if(oplist[opIndex].parent_poligon!=-1 && oplist[opIndex].parent_poligon!=-2){
                                    remove_parent_poligon(opIndex);
                                }
                                if(oplist[opIndex].parent_poligon==-1){
                                    ROS_INFO("Found connection");
                                    oplist[opIndex].parent_poligon=-2;
                                    poly.sidesIndex.push_back(opIndex);
                                    targetIndex=opIndex;
                                    break;
                                }else if(poly.sidesIndex.size()==2 ){//&& totalSlenght<=minimumSercheLenght){//Too small connection, flipping the openings
                                    ROS_INFO("Small connection");
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
                                }else{
                                    ROS_INFO("Found complete polygon");
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
                        for(int p=0;p< poly.sidesIndex.size();p++){
                            ROS_INFO("%i, start: %i, %i, end: %i, %i",i,oplist[poly.sidesIndex[p]].start.x, oplist[poly.sidesIndex[p]].start.y,oplist[poly.sidesIndex[p]].end.x, oplist[poly.sidesIndex[p]].end.y);
                        }
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
                poly.sidesIndex.clear();
                poly.inactiv=false;
                poly.label=1;
                poly.sidesIndex.push_back(i);
                oplist[i].conected_to_path=-2;//A temporary value used to indicate that the polygon is connected.
                ant_data step_info;      
                bool check=false;
                bool complet=false;
                int opIndex, firstOpIndex=-2, pathType=0;
                opening currentOpening=oplist[i];
                int countExtraPaths=0;

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
                    poly.add_point(step_info.end,cw);
                    int rezCounter=0;
                    
                    step_info.dir={0,0};
                    for(int s=0; s<=sercheLenthAntConect; s++){
                        //don't save all point to save on resources
                        if(rezCounter>=poligonRezPath){
                            rezCounter=0;
                            poly.add_point(step_info.end,cw);
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
                                poly.sidesIndex.push_back(opIndex);
                                poly.add_point(step_info.end,cw);

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
                                    if(opIndex!=firstOpIndex){
                                        ROS_INFO("Warning, path with more than two connection!");
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

                                    }else if(max_emty_cells>maxAntGap && opIndex!=-1){
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
            point nSide={side.x/(8*l),side.y/(8*l)};
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
        for(int i=0; i<poly_list->size(); i++){
            pubPolyArray.polygons[i]=poly_list->get_polygon(i,mapSize,resolution);
            pubPolyArray.labels[i]=poly_list->label[i];
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