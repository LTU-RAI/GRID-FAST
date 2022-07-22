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
        Poligon_list poly_list;
        poligon poly;
        
        bool rMap=false, rOpening=false;


    public:
        TopologyMapping(){
            subOccupancyMap= nh.subscribe("/topology_map_filterd",1,&TopologyMapping::updateMap, this);
            subOpeningList= nh.subscribe("/opening_list_int",1,&TopologyMapping::updateOplist, this);
            pubTopoPoly_debug=nh.advertise<jsk_recognition_msgs::PolygonArray>("/topology_poly_opening",5);
            pubTopoPoly=nh.advertise<jsk_recognition_msgs::PolygonArray>("/topology_poly",5);
            loadMemory();
        }
        

    void loadMemory(){
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
            
            if(rMap && rOpening){
                poly_list.clear();
                
                if(oplist.size()>0){
                    creatPoligonList();
                    if(poly_list.size()>0){
                        creatPathPoligons();
                    }
                
                    
                    for(int b=0; b<oplist.size(); b++){
                        if(oplist[b].parent_poligon<0 || oplist[b].label<0){
                            oplist.erase(oplist.begin()+b);
                            b-=1;
                        }
                    }
                    
                    for(int b=0; b<poly_list.poly.size(); b++){
                        if(poly_list.poly[b].inactiv){
                            poly_list.poly.erase(poly_list.poly.begin()+b);
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
                        step_info=ant_step(step_info.end,cw,step_info.dir,topMap);

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
                                        fitToCoridor(&newOp,40,topMap,true,true,&s1,&s2);
                                        newOp.label=2;
                                        newOp.parent_poligon=-2;
                                        poly.sidesIndex.push_back(oplist.size());
                                        int newIndex=oplist.size();
                                        oplist.push_back(newOp);
                                        if(!checkForWall(newOp,2,topMap) && dist(newOp.start,newOp.end)>=minGroupSize){
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
                                            //oplist[newIndex].label=-1;
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
                        step_info=ant_step(step_info.end,cw,step_info.dir,topMap);

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
            pubPolyArray_old.likelihood[i]=op.parent_poligon;
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

    ros::init(argc, argv, "topology_gap_analysis");
    
    TopologyMapping topMapping;

    ROS_INFO("Topology Gap Analysis Started.");
    
    topMapping.spin();
    
    return 0;
}