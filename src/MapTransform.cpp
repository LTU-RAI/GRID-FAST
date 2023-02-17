#include "Utility.hh"
#include "MapTransform.hh"

MapTransform::MapTransform(){
    MapTransform::previusMapSizeX=0;
    MapTransform::previusMapSizeY=0;
    MapTransform::mapTransformList.resize(numberOfDir);
}

MapTransform::~MapTransform(){
}

void MapTransform::updateTransform(MapHandler* map){
    if(MapTransform::previusMapSizeX==map->getMapSizeX() &&
       MapTransform::previusMapSizeY==map->getMapSizeY()) return;
    ROS_INFO("ghgd");
    
    for(int angleIndex=0;angleIndex<MapTransform::mapTransformList.size();angleIndex++){
        for(int y=0;y<MapTransform::mapTransformList[angleIndex].size();y++){
            MapTransform::mapTransformList[angleIndex][y].clear();
        }
        MapTransform::mapTransformList[angleIndex].clear();
    }
    MapTransform::mapTransformList.resize(numberOfDir);
    ROS_INFO("f3");
    MapTransform::generateMapTransform(map->getMapSizeX(),map->getMapSizeY());

}

int MapTransform::getMaptransformSizeY(int angleIndex){
    if(angleIndex>=mapTransformList.size()) return 0;
    return MapTransform::mapTransformList[angleIndex].size();
}

int MapTransform::getMaptransformSizeX(int angleIndex, int y){
    if(angleIndex>=mapTransformList.size()) return 0;
    if(y>=mapTransformList[angleIndex].size()) return 0;
    return MapTransform::mapTransformList[angleIndex][y].size();
}

mapTransformCell MapTransform::getMapTransformCell(int angleIndex, int y, int x){
    return MapTransform::mapTransformList[angleIndex][y][x];
}

//Get value at original map, at coordinates x and y. using transformation matrix with index angIndex
int MapTransform::getMapAtTransform(int x, int y, int angIndex, MapHandler* map){
    int indexY=y;
    if(indexY<0||indexY>=MapTransform::mapTransformList[angIndex].size()) return -1;
    int indexX=x-MapTransform::mapTransformList[angIndex][indexY][0].tpos.x;
    if(indexX<0||indexX>=MapTransform::mapTransformList[angIndex][indexY].size()) return -1;
    return map->getMap(MapTransform::mapTransformList[angIndex][indexY][indexX].rpos.x,
                       MapTransform::mapTransformList[angIndex][indexY][indexX].rpos.y);
}

//set value at original map, at coordinates x and y. using transformation matrix with index angIndex
void MapTransform::setMapAtTransform(int x, int y, int angIndex, int value,MapHandler* map){
    int indexY=y;
    if(indexY<0||indexY>=MapTransform::mapTransformList[angIndex].size()) return;
    int indexX=x-MapTransform::mapTransformList[angIndex][indexY][0].tpos.x;
    if(indexX<0||indexX>=MapTransform::mapTransformList[angIndex][indexY].size()) return;
    map->setMap(MapTransform::mapTransformList[angIndex][indexY][indexX].rpos.x,
                MapTransform::mapTransformList[angIndex][indexY][indexX].rpos.y,
                value);
}

//Givs index to the orignal map
point_int MapTransform::getMapIndexTransform(int x, int y, int angIndex){
    point_int mIndex={-1,-1};
    int indexY=y;
    if(indexY<0||indexY>=MapTransform::mapTransformList[angIndex].size()) return mIndex;
    int indexX=x-MapTransform::mapTransformList[angIndex][indexY][0].tpos.x;
    if(indexX<0||indexX>=MapTransform::mapTransformList[angIndex][indexY].size()) return mIndex;
    mIndex={MapTransform::mapTransformList[angIndex][indexY][indexX].rpos.x,
            MapTransform::mapTransformList[angIndex][indexY][indexX].rpos.y};
    return mIndex;
}

//pre calculates the rotation transform for the map, this saves on performance.
void MapTransform::generateMapTransform(int mapSizeX,int mapSizeY){
    MapTransform::previusMapSizeX=mapSizeX;
    MapTransform::previusMapSizeY=mapSizeY;
    MapTransform::mapTransformList.resize(numberOfDir);

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
        MapTransform::mapTransformList[angle]=tMap;
    }
}