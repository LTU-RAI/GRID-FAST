#include "Utility.h"                       
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
class TopologyMapping{
    private:
        //ROS nh
        ros::NodeHandle nh;
        //subs
        ros::Subscriber subTopometricMap;
        ros::Subscriber subLaserScan;
        ros::Subscriber subOdom;
        //pub
        ros::Publisher pubCmdVel;
        ros::Publisher pubPathMarker;

        //msg
        nav_msgs::OccupancyGrid topoMapMsg;
        visualization_msgs::Marker markDel;
        topology_mapping::topometricMap topometricMapMsg;


        //Map
        topology_mapping::topometricMap topoMap;
        
        //Global var.
        vector<point> NavigationPath;
        vector<double> laserRange;
        double laserAngInc;
        double explorationIntergrator=0;
        double rPosX, rPosY, aPosZ;
        int sameCount=0;
        int standingLabel=-1;
        int standingId=-1;



        int state=0; //0:Exploring, 1:Topometric pathplaning, 2:Topometric navigation, 3:Stop


    public:
        //Setup
        TopologyMapping(){
            subTopometricMap= nh.subscribe("/topometricMap",1,&TopologyMapping::updateMap, this);
            subLaserScan= nh.subscribe("/scan",1,&TopologyMapping::updateLaser, this);
            subOdom= nh.subscribe("/odom",1,&TopologyMapping::updatePos, this);
            pubCmdVel=nh.advertise<geometry_msgs::Twist>("/cmd_vel",5);
            pubPathMarker=nh.advertise<visualization_msgs::Marker>("/pathMarker",5);
            
        }
    
    


    void spin(){
        ros::Rate rate(10); // Hz
        
        while (ros::ok()){
            switch (state)
            {
            case 0:
                explore();
                break;
            case 1:
                creatPath();
                break;
            case 2:
                followPath();
            default:
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

    void updateLaser(const sensor_msgs::LaserScan laserMsg){
        /*int c=0;
        for(int i=laserMsg.ranges.size()-41;i<laserMsg.ranges.size();i++){
            if(isinf(laserMsg.ranges[i])){
                laserRange[c]=4;
            }else
            laserRange[c]= laserMsg.ranges[i];
            c++;

        }
        for(int i=0;i<40;i++){
            if(isinf(laserMsg.ranges[i])){
                laserRange[i+40]=4;
            }else
            laserRange[i+40]= laserMsg.ranges[i];
            
        }*/
        laserRange.resize(laserMsg.ranges.size());
        for(int i=0; i<laserMsg.ranges.size();i++){
            if(isinf(laserMsg.ranges[i])){
                laserRange[i]=4;
            }else
            laserRange[i]= laserMsg.ranges[i];
        }
        laserAngInc=laserMsg.angle_increment;
    }
    void updatePos(nav_msgs::Odometry odomMsg){
        rPosX=odomMsg.pose.pose.position.x;
        rPosY=odomMsg.pose.pose.position.y;
        double w=odomMsg.pose.pose.orientation.w,
               z=odomMsg.pose.pose.orientation.z,
               x=odomMsg.pose.pose.orientation.x,
               y=odomMsg.pose.pose.orientation.y;
        double siny = 2.0 * (w * z + x * y);
        double cosy = 1.0 - 2.0 * (y * y + z * z);

        aPosZ=std::atan2(siny, cosy);
    }

    void updateMap(const topology_mapping::topometricMap topoMsg){
        topoMap=topoMsg;
    }

    void UpdateTopoMapStanding(){
        int gridX = int((rPosX - topoMap.info.origin.position.x) / topoMap.info.resolution);
        int gridY = int((rPosY - topoMap.info.origin.position.y) / topoMap.info.resolution);
        int index=gridX+gridY*topoMap.info.width;
        if(index>=topoMap.polygonType.size()) return;

        standingId=topoMap.polygonId[index];
        
        if(standingLabel==topoMap.polygonType[index]){
            sameCount++;
        }else sameCount=0;
        
        standingLabel=topoMap.polygonType[index];
    }


    void explore(){
        point force={50,0};
        for(int i=0;i<laserRange.size();i++){
            double ang=laserAngInc*i;
            force.x-=cos(ang)*(1/(laserRange[i]*laserRange[i]))*.12;
            force.y-=sin(ang)*(1/(laserRange[i]*laserRange[i]))*.12;
        }
        double tDir=atan2(force.y,force.x);
        double p=1.0/6.0;
        double I=0.1;
        double control=tDir*p;
        geometry_msgs::Twist command;
        UpdateTopoMapStanding();
        if(sameCount>0&&standingLabel==41){
            state=1;
            ROS_INFO("done");

        }else{
            command.angular.z=control;
            if(abs(control)<0.35){
                command.linear.x=.1;
            }
        }

        pubCmdVel.publish(command);
    }

    vector<point_int> addToList(vector<point_int> list, topology_mapping::point2D_intList addision, bool revers, int startAt=-1){
        int s=startAt==-1?0:startAt;
        int e=addision.list.size();
        int change=1;
        if(revers){
            s=startAt==-1?addision.list.size()-1:startAt;
            e=-1;
            change=-1;
        }
        for(int i=s;i!=e;i+=change){
            point_int p={addision.list[i].x,
                         addision.list[i].y};
            list.push_back(p);
        }
        return list;
    }

    void creatPath(){
        UpdateTopoMapStanding();
        //chech if ther is any unexplord areas 
        bool check=true;
        for(int i=0; i<topoMap.polygons.size();i++){
            if(topoMap.polygons[i].type==52){
                check=false;
                break;
            }
        }
        if(check){
            if(topoMap.polygons.size()==0) return;
            ROS_INFO("completed");
            state=4;
            return;
        }
        //build navigation tree
        vector<vector<int>> tree;
        vector<int> path;
        tree.resize(1);
        tree[0].push_back(standingId);
        check=false;
        while(!check){
            int s=tree.size();
            for(int i=0;i<s;i++){
                if(topoMap.polygons[tree[i][tree[i].size()-1]].type==52){
                    path=tree[i];
                    check=true;
                    break;
                }

                int tIndex=0;
                if(tree[i].size()>1 && 
                   topoMap.polygons[tree[i][tree[i].size()-1]].connectedPolygons[0] == tree[i][tree[i].size()-2]){
                    tIndex=1;
                    if(topoMap.polygons[tree[i][tree[i].size()-1]].connectedPolygons.size()<2) continue;

                }

                for(int j=tIndex;j<topoMap.polygons[tree[i][tree[i].size()-1]].connectedPolygons.size();j++){
                    if(tree[i].size()>1&&
                       topoMap.polygons[tree[i][tree[i].size()-1]].connectedPolygons[j]== tree[i][tree[i].size()-2]) continue;
                    tree.push_back(tree[i]);
                    tree[tree.size()-1].push_back(topoMap.polygons[tree[i][tree[i].size()-1]].connectedPolygons[j]);
                }
                tree[i].push_back(topoMap.polygons[tree[i][tree[i].size()-1]].connectedPolygons[tIndex]);
            }
        }

        int closesIndex=-1;
        double closesDis=-1;
        point_int grid = {int((rPosX - topoMap.info.origin.position.x) / topoMap.info.resolution),
                          int((rPosY - topoMap.info.origin.position.y) / topoMap.info.resolution)};
        for(int i=0;i<topoMap.polygons[path[0]].connectedPaths[0].list.size();i++){
            point_int p={topoMap.polygons[path[0]].connectedPaths[0].list[i].x,
                         topoMap.polygons[path[0]].connectedPaths[0].list[i].y};
            double d=dist(grid,p);
            if(closesDis<0 || d<closesDis){
                closesIndex=i;
                closesDis=d;
            }
        }
        vector<point_int> list;
        for(int i=0;i<path.size()-1;i++){
            if(topoMap.polygons[path[i]].connectedPaths.size()==1){
                bool revers=false;
                int size=topoMap.polygons[path[i]].connectedPaths[0].list.size();
                for(int j=0;j<topoMap.polygons[path[i+1]].connectedPolygons.size();j++){
                    if(topoMap.polygons[path[i+1]].connectedPaths[j].list[0]
                       == topoMap.polygons[path[i]].connectedPaths[0].list[0]){
                        revers=true;
                        break;
                    }
                }
                list=addToList(list,topoMap.polygons[path[i]].connectedPaths[0],revers,closesIndex);
                closesIndex=-1;
            }else{
                int prevIndexIndex=-1;
                for(int j=0;j<topoMap.polygons[path[i]].connectedPolygons.size();j++){
                    if(topoMap.polygons[path[i]].connectedPolygons[j]==path[i-1]){
                        prevIndexIndex=j;
                        break;
                    }
                }
                int nextIndexIndex=-1;
                list=addToList(list,topoMap.polygons[path[i]].connectedPaths[prevIndexIndex],false);
                for(int j=0;j<topoMap.polygons[path[i]].connectedPolygons.size();j++){
                    if(topoMap.polygons[path[i]].connectedPolygons[j]==path[i+1]){
                        nextIndexIndex=j;
                        break;
                    }
                }
                list=addToList(list,topoMap.polygons[path[i]].connectedPaths[nextIndexIndex],true);
            }
        }
        NavigationPath.clear();
        double rez=topoMap.info.resolution;
        for(int i=0;i<list.size();i++){
            point p={(double(list[i].x))*rez+topoMap.info.origin.position.x,
                     (double(list[i].y))*rez+topoMap.info.origin.position.y};
            NavigationPath.push_back(p);
        }
        drawPath();
        state=2;
        
    }

    void followPath(){
        point p={NavigationPath[0].x-rPosX,
                 NavigationPath[0].y-rPosY};
        double lenght=sqrt(p.x*p.x+p.y*p.y);
        if(lenght<0.1){
            NavigationPath.erase(NavigationPath.begin());
            if(NavigationPath.size()==0){
                state=0;
                removePath();
                return;
            }
        }
        double tAng=atan2(p.y,p.x);
        double controlAng=tAng-aPosZ;
        if(abs(controlAng)>M_PI){
            int sign=controlAng<0?1:-1;
            controlAng=2*M_PI-abs(controlAng);
            controlAng*=sign;
        }
        
        double control=controlAng*0.5;
        geometry_msgs::Twist command;
        command.angular.z=control;
        if(abs(control)<0.2){
            command.linear.x=.1;
        }

        pubCmdVel.publish(command);
        drawPath();
    }

    void drawPath(){
        visualization_msgs::Marker msgRobotPath;

        msgRobotPath.header.frame_id = "map";
        msgRobotPath.header.stamp = ros::Time::now();
        msgRobotPath.ns="path";
        msgRobotPath.id=0;
        msgRobotPath.type=msgRobotPath.LINE_STRIP;
        msgRobotPath.action=msgRobotPath.ADD;

        msgRobotPath.pose.position.x=0;
        msgRobotPath.pose.position.y=0;
        msgRobotPath.pose.position.z=mapHight+0.2;
        msgRobotPath.pose.orientation.w=1.0;
        msgRobotPath.pose.orientation.x=0.0;
        msgRobotPath.pose.orientation.y=0.0;
        msgRobotPath.pose.orientation.z=0.0;
        msgRobotPath.scale.x=0.2;
        msgRobotPath.scale.y=0.1;
        msgRobotPath.scale.z=0.1;
        msgRobotPath.color.a=1.0;
        msgRobotPath.color.r=0.0;
        msgRobotPath.color.b=1.0;
        msgRobotPath.color.g=0.0;
        msgRobotPath.points.resize(NavigationPath.size()+1);
        msgRobotPath.lifetime=ros::Duration(0);
        msgRobotPath.points[0].x=rPosX;
        msgRobotPath.points[0].y=rPosY;
        msgRobotPath.points[0].z=0;
        for(int m=0; m<NavigationPath.size();m++){
            msgRobotPath.points[m+1].x=NavigationPath[m].x;
            msgRobotPath.points[m+1].y=NavigationPath[m].y;
            msgRobotPath.points[m+1].z=0;
        }
        pubPathMarker.publish(msgRobotPath);
    }

    void removePath(){
        visualization_msgs::Marker msgRobotPath;
        msgRobotPath.header.frame_id = "map";
        msgRobotPath.header.stamp = ros::Time::now();
        msgRobotPath.ns="path";
        msgRobotPath.id=0;
        msgRobotPath.action=msgRobotPath.DELETEALL;
        pubPathMarker.publish(msgRobotPath);
        pubPathMarker.publish(msgRobotPath);
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "topology_navigation");
    
    TopologyMapping topMapping;

    ROS_INFO("Topology Mapping Started.");
    
    topMapping.spin();
    
    return 0;
}