#include "Utility.hh"
#include "GapHandler.hh"

GapHandler::GapHandler(){    
}

GapHandler::~GapHandler(){
}

void GapHandler::analysis(MapHandler* map,MapTransform* transform){
    //setup
    GapHandler::gaps.resize(numberOfDir);
    for(int ang=0;ang<numberOfDir;ang++){
        int ySize=transform->getMaptransformSizeY(ang);
        GapHandler::gaps[ang].resize(ySize);
    }
    //analysis
    for(int angleIndex=0;angleIndex<numberOfDir;angleIndex++){
        GapHandler::analysisAtAngle(angleIndex,map,transform);
    }

}

void GapHandler::analysisAtAngle(int angleIndex, MapHandler* map, MapTransform* transform){
    //Loop thru all map cells
    for(int i=0;i<transform->getMaptransformSizeY(angleIndex);i++){
        scanGroup sg;
        int cfilter=0;
        int cGroupeSize=0;
        for(int j=0;j<transform->getMaptransformSizeX(angleIndex,i);j++){
            mapTransformCell mt=transform->getMapTransformCell(angleIndex,i,j);
            //Finde groups
            int mapValue=map->getMapUnsafe(mt.rpos.x,mt.rpos.y);
            if(mapValue==0){ 
                if(cGroupeSize==0){
                    sg.start=mt.tpos.x;
                }
                cGroupeSize+=1;
                cfilter=0;
            //filter out unoccupied cells
            }else if (mapValue==-1 && cfilter<cfilterSize && cGroupeSize!=0){
                cfilter+=1;
            
            //if findeing ostacals biger then filter end serche
            }else if(cGroupeSize!=0){
                int endpoint=mt.tpos.x-1-cfilter;
                //if found groupe is larger then minGroupSize add it as gap
                sg.end=endpoint;
                GapHandler::add(sg,angleIndex,i,true);
                cfilter=0;
                cGroupeSize=0;
            }
        }
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

void GapHandler::add(scanGroup newGap, int angleIndex, int row, bool updateConnections=true){
    newGap.row=row;
    newGap.angle=angleIndex;
    scanGroup* gapToAdd= new scanGroup(newGap);
    GapHandler::gaps[angleIndex][row].push_back(gapToAdd);
    
    if(updateConnections)
      GapHandler::updateGap(gapToAdd);
}

bool GapHandler::updateGap(scanGroup* gap){
    if(gap->end-gap->start<0){
        GapHandler::remove(gap);
        return false;
    } 

    GapHandler::cleanConnections(gap);

    gap->traversable=gap->end-gap->start>=minGroupSize;

    for(int sides=0;sides<2;sides++){
        int row=sides?gap->row+1:gap->row-1;
        if(row<0||row>=GapHandler::gaps[gap->angle].size()) continue;

        for(int index=0;index<GapHandler::gaps[gap->angle][row].size();index++){
            //Skip nontraversable gaps
            if(!GapHandler::gaps[gap->angle][row][index]->traversable) continue;
            GapHandler::checkForOverlap( GapHandler::gaps[gap->angle][row][index], gap );
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
void GapHandler::checkForOverlap(scanGroup* gap1, scanGroup* gap2){
    int startGap1=gap1->start;
    int endGap1=gap1->end;
    int startGap2=gap2->start;
    int endGap2=gap2->end;
    //If gaps do not overlap, continue 
    if(startGap1>endGap2 || endGap1<startGap2) return;

    if(gap1->row<gap2->row){
        gap1->nextGroup.push_back(gap2);
        gap2->prevGroup.push_back(gap1);
    }else{
        gap2->nextGroup.push_back(gap1);
        gap1->prevGroup.push_back(gap2);
    }

    return;
}