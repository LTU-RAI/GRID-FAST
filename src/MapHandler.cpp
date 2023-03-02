#include "Utility.hh"
#include "MapHandler.hh"


MapHandler::MapHandler(){
    MapHandler::mapSizeX=0;
    MapHandler::mapSizeY=0;
    MapHandler::mapOffsetX=0;
    MapHandler::mapOffsetY=0;
    MapHandler::resulution=1;
}

MapHandler::~MapHandler(){
}

void MapHandler::updateMap(vector<int> newMap,int newSizeX,int newSizeY,int newResulution,int newMapOffsetX=0,int newMapOffsetY=0){
    MapHandler::resulution=resulution;
    MapHandler::mapOffsetX=newMapOffsetX;
    MapHandler::mapOffsetY=newMapOffsetY;
    MapHandler::mapSizeX=newSizeX;
    MapHandler::mapSizeY=newSizeY;

    MapHandler::map=newMap;
}

void MapHandler::updateMap(MapHandler* newMap){
    MapHandler::resulution=newMap->getMapResolution();
    MapHandler::mapOffsetX=newMap->getMapOffsetX();
    MapHandler::mapOffsetY=newMap->getMapOffsetY();
    MapHandler::mapSizeX=newMap->getMapSizeX();
    MapHandler::mapSizeY=newMap->getMapSizeY();

    MapHandler::map=newMap->getMapComplete();
}

int MapHandler::getMapSizeX(){
    return MapHandler::mapSizeX;
}

int MapHandler::getMapSizeY(){
    return MapHandler::mapSizeY;
}

int MapHandler::getMapOffsetX(){
    return MapHandler::mapOffsetX;
}

int MapHandler::getMapOffsetY(){
    return MapHandler::mapOffsetY;
}

int MapHandler::getMapResolution(){
    return MapHandler::resulution;
}

int MapHandler::getMap(int x,int y){
    if(x<0||x>=MapHandler::mapSizeX) return -1;
    if(y<0||y>=MapHandler::mapSizeY) return -1;
    int index=x+y*MapHandler::mapSizeX;
    return MapHandler::map[index];
}

void MapHandler::setMap(int x,int y,int value){
    if(x<0||x>=MapHandler::mapSizeX) return;
    if(y<0||y>=MapHandler::mapSizeY) return;
    int index=x+y*MapHandler::mapSizeX;
    MapHandler::map[index]=value;
}

//Do not have any protection for out of array access
int MapHandler::getMapUnsafe(int x,int y){
    int index=x+y*MapHandler::mapSizeX;
    return MapHandler::map[index];
}

vector<int> MapHandler::getMapComplete(){
    return map;
}

//Function to take one step using an ant algorithm 
ant_data MapHandler::ant_step(ant_data oldStep, bool clockwise=true){
    int newDirx, newDiry;
    ant_data step;
    point_int start=oldStep.end;
    
    step.end=oldStep.end;

    if(oldStep.dir.x==0 && oldStep.dir.y==0){
        step.dir={1,0};
        int c=0;
        int l=1;
        while(MapHandler::getMap(start.x+step.dir.x*l,start.y+step.dir.y*l)==MAP_UNOCCUPIED){
            newDirx=clockwise?step.dir.y:-step.dir.y;
            newDiry=clockwise?-step.dir.x:step.dir.x;
            step.dir.x=newDirx;
            step.dir.y=newDiry;
            if(c>4){
                l++;
                c=0;
            }
            c++;
        }if(l>2){
            start.x+=step.dir.x*(l-1);
            start.y+=step.dir.y*(l-1);
        }
    }else{
        step.dir=rotate_dir(oldStep.dir,!clockwise);
        step.dir=rotate_dir(step.dir,!clockwise);
    }

    for(int d=0;d<4;d++){
        bool check=false;
        if(MapHandler::getMap(start.x+step.dir.x,start.y+step.dir.y)==MAP_UNOCCUPIED){
            point_int dir=step.dir;
            if(dir.x*dir.y!=0 && false){
                dir=MapHandler::rotate_dir(dir,clockwise);
                if(MapHandler::getMap(start.x+dir.x,start.y+dir.y)==MAP_UNOCCUPIED)
                    check=true;
            }else{check=true;}
        }
        if(check){
            step.end.x=start.x+step.dir.x;
            step.end.y=start.y+step.dir.y;
            step.emty_cell=false;
            point_int dir={0,1};
            step.emty_cell=false;
            for(int e=0;e<8;e++){
                    
                if(MapHandler::getMap(step.end.x+dir.x,step.end.y+dir.y)==MAP_UNKNOWN){
                    step.emty_cell=true;
                    break;
                }
                dir=rotate_dir(dir,clockwise);
            }
            return step;
        }

        step.dir=rotate_dir(step.dir,clockwise);
        step.dir=rotate_dir(step.dir,clockwise);
    }
    return step;
} 

point_int MapHandler::rotate_dir(point_int dir, bool cw){
    point_int newDir[]={{-1,1},{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{0,0}};
    for(int i=0; i<8; i++){
        if(newDir[i]==dir){
            int next_index=cw?i+1:i-1;
            if(next_index==-1)next_index=7;
            next_index=next_index%8;
            return newDir[next_index];
        }
    }
    return newDir[8];
}

// Retrun number of cells is wall overlapt by a ray between p1 and p2
int MapHandler::checkForWallRay(point_int p1, point_int p2){
    int wallcount=0;
    double l=0.1;
    double opLenght=dist(p1,p2);
    point normdVop={((p2.x-p1.x)/(opLenght))*l,((p2.y-p1.y)/(opLenght))*l};
    for(int wallScan=0;wallScan<(int)(opLenght/l);wallScan++){
        point_int pp={p1.x+(int)(std::round(normdVop.x*wallScan)), p1.y+(int)(std::round(normdVop.y*wallScan))};
        if(MapHandler::getMap(pp.x,pp.y)!=MAP_UNOCCUPIED){
            wallcount+=1;
        }
    }
    return wallcount;
}





