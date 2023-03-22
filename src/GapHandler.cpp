#include "Utility.hh"
#include "GapHandler.hh"

GapHandler::GapHandler(){    
}

GapHandler::~GapHandler(){
}

void GapHandler::analysis(MapHandler* map,MapTransform* transform){
    //setup
    GapHandler::toBeFilterdPoints.clear();
    GapHandler::toBeFilterdValues.clear();
    GapHandler::gaps.resize(numberOfDir);
    for(int ang=0;ang<numberOfDir;ang++){
        int ySize=transform->getMaptransformSizeY(ang);
        GapHandler::gaps[ang].resize(ySize);
    }
    //analysis
    for(int angleIndex=0;angleIndex<numberOfDir;angleIndex++){
        GapHandler::analysisAtAngle(angleIndex,map,transform);
    }
    /*std::vector<std::thread> threads;
    for (int angleIndex = 0; angleIndex < numberOfDir; angleIndex++) {
        threads.push_back(std::thread(&GapHandler::analysisAtAngle, this, angleIndex, map, transform));
    }

    for (std::thread &t : threads) {
        if (t.joinable()) {
            t.join();
        }
    }*/

    for(int angleIndex=0;angleIndex<numberOfDir;angleIndex++){
        for(int row=0;row<GapHandler::getSizeRows(angleIndex);row++){
            for(int index=0;index<GapHandler::getSizeGaps(angleIndex,row);index++){
                scanGroup* targetGap=GapHandler::get(angleIndex,row,index);
                if(targetGap->traversable) continue;
                GapHandler::fillGapAtMap(map,transform,angleIndex,targetGap);
            }
        }
    }

    //Apply filter
    for(int index=0;index<GapHandler::toBeFilterdPoints.size();index++){
        if(map->getMapUnsafe(GapHandler::toBeFilterdPoints[index].x,
                             GapHandler::toBeFilterdPoints[index].y)==MAP_OCCUPIED) continue;
        map->setMap(GapHandler::toBeFilterdPoints[index].x,
                    GapHandler::toBeFilterdPoints[index].y,
                    GapHandler::toBeFilterdValues[index]);
    }
}

void GapHandler::analysisAtAngle(int angleIndex, MapHandler* map, MapTransform* transform){
    for(int row=0;row<transform->getMaptransformSizeY(angleIndex);row++){
        analysisAtRow(angleIndex,row,map,transform);
    }
}

void GapHandler::analysisAtRow(int angleIndex, int row,MapHandler* map, MapTransform* transform){
    scanGroup sg;
    int cfilter=0;
    int cGroupeSize=0;
    vector<point_int> scanPoints;
    for(int index=0;index<transform->getMaptransformSizeX(angleIndex,row);index++){
        mapTransformCell mt=transform->getMapTransformCell(angleIndex,row,index);
        //Finde groups
        int mapValue=map->getMapUnsafe(mt.rpos.x,mt.rpos.y);
        if(mapValue==0){ 
            if(cGroupeSize==0){
                sg.start=mt.tpos.x;
            }
            cGroupeSize+=1;
            cfilter=0;
            scanPoints.push_back(mt.rpos);
        //filter out unoccupied cells
        }else if (mapValue==-1 && cfilter<cfilterSize && cGroupeSize!=0){
            cfilter+=1;
            scanPoints.push_back(mt.rpos);
        
        //if findeing ostacals biger then filter end serche
        }else if(cGroupeSize!=0){
            int endpoint=mt.tpos.x-1-cfilter;
            //if found groupe is larger then minGroupSize add it as gap
            sg.end=endpoint;
            int value=MAP_UNOCCUPIED;
            if(sg.end-sg.start>=minGroupSize){
                GapHandler::add(sg,angleIndex,row);
            }else{
                if(transform->getMapAtTransform(sg.start-1,row,angleIndex,map)==MAP_OCCUPIED||
                   transform->getMapAtTransform(sg.end+1,row,angleIndex,map)==MAP_OCCUPIED){
                    value=MAP_OCCUPIED;
                }else{
                    value=MAP_UNKNOWN;
                }
                for(int i=0; i<scanPoints.size()-cfilter;i++){
                    GapHandler::toBeFilterdPoints.push_back(scanPoints[i]);
                    GapHandler::toBeFilterdValues.push_back(value);
                }
            }
            
            
            scanPoints.clear();
            cfilter=0;
            cGroupeSize=0;
        }
    }
}

void GapHandler::fillGapAtMap(MapHandler* map,MapTransform* transform,int angleIndex, scanGroup* gap){
    int value=MAP_UNOCCUPIED;
    if(!gap->traversable){
        if(transform->getMapAtTransform(gap->start-1,gap->row,angleIndex,map)==MAP_OCCUPIED||
            transform->getMapAtTransform(gap->end+1,gap->row,angleIndex,map)==MAP_OCCUPIED){
            value=MAP_OCCUPIED;
        }else{
            value=MAP_UNKNOWN;
        }
    }
    for(int x=gap->start;x<=gap->end;x++){
        GapHandler::toBeFilterdPoints.push_back(transform->getMapIndexTransform(x,gap->row,angleIndex));
        GapHandler::toBeFilterdValues.push_back(value);
    }
}

int GapHandler::getSizeRows(int angleIndex){
    return GapHandler::gaps[angleIndex].size();
}

int GapHandler::getSizeGaps(int angleIndex, int row){
    return GapHandler::gaps[angleIndex][row].size();
}

//Retruns pointer to target gap, returns NULL if gap dos not exist
scanGroup* GapHandler::get(int angleIndex, int row, int index){
    if(angleIndex>numberOfDir) return NULL;
    if(row>=GapHandler::gaps[angleIndex].size()) return NULL;
    if(index>=GapHandler::gaps[angleIndex][row].size()) return NULL;
    return GapHandler::gaps[angleIndex][row][index];
}

void GapHandler::add(scanGroup newGap, int angleIndex, int row){
    newGap.row=row;
    newGap.angle=angleIndex;
    scanGroup* gapToAdd= new scanGroup(newGap);
    GapHandler::gaps[angleIndex][row].push_back(gapToAdd);
    
    GapHandler::updateGap(gapToAdd);
}

bool GapHandler::updateGap(scanGroup* gap){
    if(gap->end-gap->start<0){
        GapHandler::remove(gap);
        return false;
    } 

    GapHandler::cleanConnections(gap);

    gap->traversable=gap->end-gap->start>=minGroupSize;
    if(!gap->traversable) return true;

    for(int sides=0;sides<2;sides++){
        int row=sides?gap->row+1:gap->row-1;
        if(row<0||row>=GapHandler::gaps[gap->angle].size()) continue;

        for(int index=0;index<GapHandler::gaps[gap->angle][row].size();index++){
            //Skip nontraversable gaps
            if(!GapHandler::gaps[gap->angle][row][index]->traversable) continue;
            if(!GapHandler::checkForOverlap( GapHandler::gaps[gap->angle][row][index], gap )) break;
        }
    }
    return true;
}

void GapHandler::remove(scanGroup* gap){
    GapHandler::cleanConnections(gap);
    for(int i=0;i<GapHandler::gaps[gap->angle][gap->row].size();i++){
        if(GapHandler::gaps[gap->angle][gap->row][i]!=gap) continue;
        GapHandler::gaps[gap->angle][gap->row].erase(
          GapHandler::gaps[gap->angle][gap->row].begin() +i);
        break;    
    }
    delete gap;
}

void GapHandler::clear(){
    for(int angleIndex=0;angleIndex<GapHandler::gaps.size();angleIndex++){
        for(int row=0;row<GapHandler::gaps[angleIndex].size();row++){
            for(int index=0;index<GapHandler::gaps[angleIndex][row].size();index++){
                delete GapHandler::gaps[angleIndex][row][index];
            }
            GapHandler::gaps[angleIndex][row].clear();
        }
        GapHandler::gaps[angleIndex].clear();
    }
    GapHandler::gaps.clear();
}

void GapHandler::cleanConnections(scanGroup* gap){
    for(int i1=0;i1<gap->prevGroup.size();i1++){
        scanGroup* conectedGap= gap->prevGroup[i1];
        for(int i2=0; i2<conectedGap->nextGroup.size();i2++){
            if(conectedGap->nextGroup[i2]!=gap) continue;
            conectedGap->nextGroup.erase(conectedGap->nextGroup.begin()+i2);
            break; 
        }
    }
    for(int i1=0;i1<gap->nextGroup.size();i1++){
        scanGroup* conectedGap= gap->nextGroup[i1];
        for(int i2=0; i2<conectedGap->prevGroup.size();i2++){
            if(conectedGap->prevGroup[i2]!=gap) continue;
            conectedGap->prevGroup.erase(conectedGap->prevGroup.begin()+i2);
            break; 
        }
    }
    gap->nextGroup.clear();
    gap->prevGroup.clear();
}


//retruns false if gap1 or gap2 is changed
bool GapHandler::checkForOverlap(scanGroup* gap1, scanGroup* gap2){
    int startGap1=gap1->start;
    int endGap1=gap1->end;
    int startGap2=gap2->start;
    int endGap2=gap2->end;
    //If gaps do not overlap, continue 
    if(startGap1>endGap2 || endGap1<startGap2) return true;

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
        GapHandler::add(sg,gap1->angle,gap1->row);
        GapHandler::add(sg,gap2->angle,gap2->row);
        GapHandler::updateGap(gap2);
        GapHandler::updateGap(gap1);
        return false;
    }
    if(gap1->row<gap2->row){
        gap1->nextGroup.push_back(gap2);
        gap2->prevGroup.push_back(gap1);
    }else{
        gap2->nextGroup.push_back(gap1);
        gap1->prevGroup.push_back(gap2);
    }

    return true;
}