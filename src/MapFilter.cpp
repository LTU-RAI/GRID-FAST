#include "Utility.hh"
#include "MapFilter.hh"

MapFilter::MapFilter(){
    MapFilter::previusSizeX=0;
    MapFilter::previusSizeY=0;
}

MapFilter::~MapFilter(){
}

void MapFilter::filterMap(MapHandler* map,MapTransform* transform,GapHandler* gaps){
    //prepare wallFollowLookUp
    if(MapFilter::previusSizeX!=map->getMapSizeX() ||
       MapFilter::previusSizeY!=map->getMapSizeY()){
        wallFollowLookUp.resize(map->getMapSizeX()*map->getMapSizeY());
    }
    for(int index=0;index<wallFollowLookUp.size();index++){
        wallFollowLookUp[index]=false;
    }

    if(removeOpeningsFirst){
        MapFilter::removeOpenings(map,transform,gaps);
        MapFilter::removeObjects(map,transform,gaps);
    }else{
        MapFilter::removeObjects(map,transform,gaps);
        MapFilter::removeOpenings(map,transform,gaps);
    }
}

bool MapFilter::getLookUp(int x, int y){
    int index= x+y*MapFilter::previusSizeX;
    return MapFilter::wallFollowLookUp[index];
}

void MapFilter::setLookUp(int x, int y, bool value){
    int index= x+y*MapFilter::previusSizeX;
    MapFilter::wallFollowLookUp[index]=value;
}

void MapFilter::removeOpenings(MapHandler* map,MapTransform* transform,GapHandler* gaps){
    for(int angleIndex=0;angleIndex<numberOfDir;angleIndex++){
        for(int row=0;row<gaps->getSizeRows(angleIndex);row++){
            for(int index=0;index<gaps->getSizeGaps(angleIndex,row);index++){
                MapFilter::filterGapOverlapSize(gaps,gaps->get(angleIndex,row,index));
            }
        }
    }

    for(int angleIndex=0;angleIndex<numberOfDir;angleIndex++){
        for(int row=0;row<gaps->getSizeRows(angleIndex);row++){
            for(int index=0;index<gaps->getSizeGaps(angleIndex,row);index++){
                scanGroup* targetGap=gaps->get(angleIndex,row,index);
                if(!targetGap->traversable){
                    MapFilter::fillGapAtMap(map,transform,angleIndex,targetGap);
                    gaps->cleanConnections(targetGap);
                }
            }
        }
    }
}


void MapFilter::filterGapOverlapSize(GapHandler* gapH,scanGroup* gap1){
    if(!gap1->traversable) return;
    for(int nextIndex=0;nextIndex<gap1->nextGroup.size();nextIndex++){
        scanGroup* gap2=gap1->nextGroup[nextIndex];
        if(!gap2->traversable) continue;

        int startGap1=gap1->start;
        int endGap1=gap1->end;
        int startGap2=gap2->start;
        int endGap2=gap2->end;
        
        int overlapStart=std::max(startGap1,startGap2);
        int overlapEnd=std::min(endGap1,endGap2);
        int overlapSize=overlapEnd-overlapStart;
        if(overlapSize<minGroupSize){
            if(startGap1==overlapStart){
                gap1->start=overlapEnd+1;
                gap2->end=overlapStart-1;
            }else{
                gap2->start=overlapEnd+1;
                gap1->end=overlapStart-1;
            }
            scanGroup sg;
            sg.start=overlapStart;
            sg.end=overlapEnd;
            gapH->add(sg,gap1->angle,gap1->row,false);
            gapH->add(sg,gap2->angle,gap2->row,false);
            gapH->updateGap(gap2);
            if(!gapH->updateGap(gap1)) return;
            
            
            MapFilter::filterGapOverlapSize(gapH, gap1);
            return;
        }
    }
}

void MapFilter::fillGapAtMap(MapHandler* map,MapTransform* transform,int angleIndex, scanGroup* gap){
    for(int x=gap->start;x<gap->end;x++){
        transform->setMapAtTransform(x,gap->row,angleIndex,MAP_OCCUPIED,map);
    }
}

void MapFilter::removeObjects(MapHandler* map, MapTransform* transform, GapHandler* gaps){
    for(int row=0;row<gaps->getSizeRows(0);row++){
        for(int index=0;index<gaps->getSizeGaps(0,row);index++){
            int numberOfConections=0;
            scanGroup* targetGap=gaps->get(0,row,index);
            if(!removeOpeningsFirst || !targetGap->traversable) continue;
            for(int nextIndex=0;nextIndex<targetGap->nextGroup.size();nextIndex++){
                if(!removeOpeningsFirst || !targetGap->nextGroup[nextIndex]->traversable)
                numberOfConections++;
            }
            if(numberOfConections<2) continue;
            
            MapFilter::checkAndRemoveObject(map,targetGap,numberOfConections);
        }
    }
    
    for(int angleIndex=0;angleIndex<numberOfDir;angleIndex++){
        for(int row=0;row<gaps->getSizeRows(angleIndex);row++){
            for(int index=0;index<gaps->getSizeGaps(angleIndex,row);index++){
                fitGapToMap(map,transform,gaps,angleIndex,row,index);
            }
        }
    }
}

void MapFilter::checkAndRemoveObject(MapHandler* map,scanGroup* gap, int numberOfConections){
    int numberGapsChecked=0;
    for(int nextIndex=0;nextIndex<gap->nextGroup.size();nextIndex++){
        scanGroup* targetGap=gap->nextGroup[nextIndex];
        if(!removeOpeningsFirst || !targetGap->traversable) continue;
        numberGapsChecked++;
        if(numberGapsChecked==numberOfConections) return;

        ant_data step;
        point_int startP={targetGap->start,targetGap->row};
        step.end={targetGap->end,targetGap->row};;
        vector<point_int> pointList;
        for(int s=0;s<=objectFilterMaxStep;s++){
            step=map->ant_step(step,false);
            if(MapFilter::getLookUp(step.end.x,step.end.y) || step.end==startP){
                break;
            }
            if(pointList.size()>0){
                if(step.end==pointList[0]){
                    vector<point_int> filledPoints=fillPoly(pointList);
                    for(int e=0;e<filledPoints.size();e++) 
                        map->setMap(filledPoints[e].x,filledPoints[e].y,MAP_UNOCCUPIED);
                    break;
                }
            }
            if(s==objectFilterMaxStep){
                for(int m=0;m<pointList.size();m++){
                    MapFilter::setLookUp(pointList[m].x,pointList[m].y,true);
                }
                break;
            }
            pointList.push_back(step.end);
        }
    }
}

void MapFilter::fitGapToMap(MapHandler* map, MapTransform* transform, GapHandler* gaps,int angleIndex, int row, int index){
    scanGroup* targetGap=gaps->get(angleIndex,row,index);
    if(!removeOpeningsFirst || !targetGap->traversable) return;
    int newEnd=targetGap->end;
    while(transform->getMapAtTransform(newEnd+1,targetGap->row,targetGap->angle,map)==MAP_UNOCCUPIED){
        newEnd++;
    }
    targetGap->end=newEnd;
    while(true){
        scanGroup* testGap=gaps->get(angleIndex,row,index+1);
        if(testGap==NULL) break;
        if(testGap->start>newEnd) break;
        gaps->remove(testGap);
    }
    
}