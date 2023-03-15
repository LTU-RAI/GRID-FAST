#include "Utility.hh"      
#include "MapHandler.hh"
#include "MapTransform.hh"
#include "GapHandler.hh"
#include "MapFilter.hh"                  
#include "OpeningHandler.hh"     
#include "PolygonHandler.hh"   

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
        ros::Publisher pubMapDebug;
        ros::Publisher pubTopoPoly_debug;
        ros::Publisher pubTopoPoly;
        ros::Publisher pubTopometricMap;
        ros::Publisher pubRobotPath;
        ros::Publisher pubMarkDel;

        //msg
        nav_msgs::OccupancyGrid topoMapMsg;
        nav_msgs::OccupancyGrid topoMapMsgD;

        MapHandler* map, *mapDebug;
        MapTransform* transform;
        GapHandler* gaps;
        MapFilter* filter;
        OpeningHandler* openingList;
        PolygonHandler* polygonList; 

        int oldNodCount=0;
        vector<vector<double>> timeVector;


    public:
        //setup
        TopologyMapping(){
            std::string config_file_path= ros::package::getPath("grid_fast")+"/config/settings.conf";
            load_config_file(config_file_path);
            subOccupancyMap= nh.subscribe("/map",1,&TopologyMapping::updateMap, this);
            pubTopoMap=nh.advertise<nav_msgs::OccupancyGrid>("/topology_map_filterd",5);
            pubdMap=nh.advertise<nav_msgs::OccupancyGrid>("/topology_map_d",5);
            pubOpeningList=nh.advertise<grid_fast::opening_list>("/opening_list_int",5);
            pubMapDebug=nh.advertise<nav_msgs::OccupancyGrid>("/topology_map_debug",5);
            pubTopoPoly_debug=nh.advertise<jsk_recognition_msgs::PolygonArray>("/topology_poly_opening",5);
            pubRobotPath=nh.advertise<visualization_msgs::MarkerArray>("/topology_robot_path",5);
            pubTopoPoly=nh.advertise<jsk_recognition_msgs::PolygonArray>("/topology_poly",5);
            pubTopometricMap=nh.advertise<grid_fast::topometricMap>("/topometricMap",5);
            loadMemory();
        }
        
    //Load all maps, and map transforms into the memory
    void loadMemory(){
        timeVector.resize(4);
        map = new MapHandler;
        mapDebug = new MapHandler;
        transform = new MapTransform;
        gaps = new GapHandler;
        filter = new MapFilter;
        openingList = new OpeningHandler;
        polygonList = new PolygonHandler;

    }

    // initialization of map message
    void initializeTopoMap(){
        topoMapMsg.header.frame_id = "map";
        topoMapMsg.info.origin.orientation.x = 0.0;
        topoMapMsg.info.origin.orientation.y = 0.0;
        topoMapMsg.info.origin.orientation.z = 0.0;
        topoMapMsg.info.origin.orientation.w = 1.0;
    }

    //Get new occupancy map and move its value into Map.
    void updateMap(const nav_msgs::OccupancyGrid& mapMsg){
        vector<int> data;
        vector<int> bData;
        data.resize(mapMsg.data.size());
        bData.resize(mapMsg.data.size());
        for(int index=0;index<mapMsg.data.size();index++){
            data[index]=mapMsg.data[index];
            bData[index]=-1;
        }
        map->updateMap(data,mapMsg.info.width,mapMsg.info.height,
                      mapMsg.info.resolution,mapMsg.info.origin.position.x,mapMsg.info.origin.position.y,mapMsg.info.origin.position.z);
        
        
        mapDebug->updateMap(bData,mapMsg.info.width,mapMsg.info.height,
                      mapMsg.info.resolution,mapMsg.info.origin.position.x,mapMsg.info.origin.position.y,mapMsg.info.origin.position.z);
        Time T;
        //ROS_INFO("g0");
        gaps->clear();
        openingList->clear();
        polygonList->clear();
        //ROS_INFO("g1");
        transform->updateTransform(map,true);
        timeVector[0].insert(timeVector[0].begin(),T.get_since());
        //ROS_INFO("g2");
        gaps->analysis(map,transform);
        //ROS_INFO("g3");
        openingList->updateDetections(map,transform,gaps);
        //ROS_INFO("g4");
        openingList->update(map);
        //ROS_INFO("g5");
        polygonList->updateIntersections(openingList,map);
        //ROS_INFO("g6");
        polygonList->generatePolygonArea(openingList);
        timeVector[2].insert(timeVector[2].begin(),T.get_since());
        ///ROS_INFO("g7");
        polygonList->generateRobotPath(openingList,map,mapDebug);
        //ROS_INFO("g8");
        
        timeVector[3].insert(timeVector[3].begin(),T.get_since());
        vector<double> Td;
        Td.resize(timeVector.size());
        for(int i1=0;i1<timeVector.size();i1++){
            Td[i1]=0;
            if(timeVector[i1].size()>10) timeVector[i1].pop_back();
            for(int i2=0;i2<timeVector[i1].size();i2++){
                Td[i1]+=timeVector[i1][i2];
            }
            Td[i1]=Td[i1]/timeVector[i1].size();
        }
        
        ROS_INFO("Time: %f, %f", Td[2], Td[3]);
        pubMap();
    }
    
    //Function to publish all topics. 
    void pubMap(){
        grid_fast::opening_list OpeningListMsg;

        topoMapMsg.info.width = map->getMapSizeX();
        topoMapMsg.info.height = map->getMapSizeY();
        topoMapMsg.info.resolution = map->getMapResolution();
        topoMapMsg.info.origin.position.x = map->getMapOffsetX();
        topoMapMsg.info.origin.position.y = map->getMapOffsetY();
        topoMapMsg.info.origin.position.z = map->getMapHight()+0.05; //for visualization
        topoMapMsg.data.resize(topoMapMsg.info.width * topoMapMsg.info.height);

        topoMapMsg.header.stamp = ros::Time::now();
        for(int y=0; y<map->getMapSizeY();y++){
            for(int x=0;x<map->getMapSizeX();x++){ 
                int index=x+y*map->getMapSizeX();           
                topoMapMsg.data[index]=map->getMap(x,y);
            }
        }
        pubTopoMap.publish(topoMapMsg);
        vector<opening> pubOpList;
        for(int i=0; i<openingList->size(); i++){
            opening op=openingList->get(i)->getOpening();
            pubOpList.push_back(op);
        }
        /*for(int i=0; i<openingList.gapDetectionsSize(); i++){
            opening op=*openingList.getDetection(i);
            pubOpList.push_back(op);
        }*/
        vector<vector<int>> dMap;
        dMap.resize(mapDebug->getMapSizeX());

        for(int x=0; x<mapDebug->getMapSizeX();x++){
            dMap[x].resize(mapDebug->getMapSizeY());
            for(int y=0;y<mapDebug->getMapSizeY();y++){  
                dMap[x][y]=mapDebug->getMap(x,y);
            }
        }
        jsk_recognition_msgs::PolygonArray pubPolyArray_old;
        pubPolyArray_old.header.frame_id = "map";
        pubPolyArray_old.header.stamp = ros::Time::now();
        pubPolyArray_old.polygons.resize(pubOpList.size());
        pubPolyArray_old.labels.resize(pubOpList.size());
        pubPolyArray_old.likelihood.resize(pubOpList.size());
        float resolution=map->getMapResolution();
        int MapOrigenX=-map->getMapOffsetX()/resolution;
        int MapOrigenY=-map->getMapOffsetY()/resolution;
        for(int i=0; i<pubOpList.size(); i++){
            //convert the line of an opening to a rectangular polygon
            opening op=pubOpList[i];
            dMap[op.start.x][op.start.y]=100;
            dMap[op.end.x][op.end.y]=50;
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
        }
        for(int y=0; y<map->getMapSizeY();y++){
            for(int x=0;x<map->getMapSizeX();x++){  
                int index=x+y*map->getMapSizeX();            
                topoMapMsg.data[index]=dMap[x][y];
            }
        }
        pubdMap.publish(topoMapMsg);
        pubTopoPoly_debug.publish(pubPolyArray_old);
        
        jsk_recognition_msgs::PolygonArray pubPolyArray;
        pubPolyArray.header.frame_id = "map";
        pubPolyArray.header.stamp = ros::Time::now();
        pubPolyArray.polygons.resize(polygonList->size());
        pubPolyArray.labels.resize(polygonList->size());
        pubPolyArray.likelihood.resize(polygonList->size());
        int c=0;
        for(int i=0; i<polygonList->size(); i++){
            polygon p=*polygonList->get(i);
            pubPolyArray.polygons[i]=get_polygon(p,resolution,map->getMapOffsetX(),map->getMapOffsetY());
            pubPolyArray.labels[i]=p.label;
            if(p.label==30 || p.label==41 || p.label==76 || p.label==52) c++;
            pubPolyArray.likelihood[i]=1;
        }
        if(c!=oldNodCount){
            oldNodCount=c;
            ROS_INFO("Node count: %i",c);
        }
        pubTopoPoly.publish(pubPolyArray);
        
        vector<robotPath> robotPathList;
        for(int i=0; i<polygonList->size(); i++){
            polygon* p=polygonList->get(i);
            for(int pathI=0;pathI<p->pathList.size();pathI++){
                robotPathList.push_back(p->pathList[pathI]);
            }
        }
        visualization_msgs::MarkerArray msgRobotPath;
        msgRobotPath.markers.resize(1+robotPathList.size());
        msgRobotPath.markers[0].header.frame_id = "map";
        msgRobotPath.markers[0].header.stamp = ros::Time::now();
        msgRobotPath.markers[0].action=msgRobotPath.markers[0].DELETEALL;
        for(int i=1; i<robotPathList.size()+1;i++){
            msgRobotPath.markers[i].header.frame_id = "map";
            msgRobotPath.markers[i].header.stamp = ros::Time::now();
            msgRobotPath.markers[i].ns="robotPathList";
            msgRobotPath.markers[i].id=i;
            msgRobotPath.markers[i].type=msgRobotPath.markers[i].LINE_STRIP;
            msgRobotPath.markers[i].action=msgRobotPath.markers[i].ADD;

            msgRobotPath.markers[i].pose.position.x=0;
            msgRobotPath.markers[i].pose.position.y=0;
            msgRobotPath.markers[i].pose.position.z=map->getMapHight()+0.05;
            msgRobotPath.markers[i].pose.orientation.w=1.0;
            msgRobotPath.markers[i].pose.orientation.x=0.0;
            msgRobotPath.markers[i].pose.orientation.y=0.0;
            msgRobotPath.markers[i].pose.orientation.z=0.0;
            msgRobotPath.markers[i].scale.x=0.1;
            msgRobotPath.markers[i].scale.y=0.1;
            msgRobotPath.markers[i].scale.z=0.1;
            msgRobotPath.markers[i].color.a=1.0;
            msgRobotPath.markers[i].color.r=1.0;
            msgRobotPath.markers[i].color.b=0.0;
            msgRobotPath.markers[i].color.g=0.0;
            msgRobotPath.markers[i].points.resize(robotPathList[i-1].size());
            msgRobotPath.markers[i].lifetime=ros::Duration(0);
            for(int m=0; m<robotPathList[i-1].size();m++){
                msgRobotPath.markers[i].points[m].x=(robotPathList[i-1][m].x-MapOrigenX)*resolution;
                msgRobotPath.markers[i].points[m].y=(robotPathList[i-1][m].y-MapOrigenY)*resolution;
                msgRobotPath.markers[i].points[m].z=0;
            }
        }
        
        pubRobotPath.publish(msgRobotPath);
        
        /*topoMapMsg.header.stamp = ros::Time::now();
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
            vector<point_int> filedP;//=fillPoly(poly_list[i].polygon_points);
            for(int j=0;j<filedP.size();j++){
                int index=filedP[j].x+filedP[j].y*mapSizeX;
                topometricMapMsg.polygonType[index]=poly_list[i].label;
                topometricMapMsg.polygonId[index]=i;
                topoMapMsg.data[index]=poly_list[i].label;
            }
            //topometricMapMsg.polygons[i].polygonPoints=ConvertPointListToMsg(poly_list[i].polygon_points);
            int size=poly_list[i].connectedPaths.size();
            topometricMapMsg.polygons[i].connectedPaths.resize(size);
            for(int n=0;n<size;n++){
                //topometricMapMsg.polygons[i].connectedPaths[n]=ConvertPointListToMsg(poly_list[i].connectedPaths[n]);
            }
            topometricMapMsg.polygons[i].id=i;
            topometricMapMsg.polygons[i].type=poly_list[i].label;
            //topometricMapMsg.polygons[i].connectedPolygons=poly_list[i].connectedpolygons;

        }
        pubMapDebug.publish(topoMapMsg);
        pubTopometricMap.publish(topometricMapMsg);
    */
    }

};


int main(int argc, char** argv){

    ros::init(argc, argv, "topology_gap_analysis");
    
    TopologyMapping topMapping;

    ROS_INFO("Topology Gap Analysis Started.");
    
    ros::spin();
    
    return 0;
}