#include "Utility.hh"
#include "OpeningHandler.hh"

OpeningHandler::OpeningHandler(){

}

OpeningHandler::~OpeningHandler(){
    
}

void OpeningHandler::updateDetections(MapHandler* map, MapTransform* transform, GapHandler* gaps){
    OpeningHandler::detectionMap.resize(map->getMapSizeX());
    for(int i=0;i<OpeningHandler::detectionMap.size();i++){
        OpeningHandler::detectionMap[i].resize(map->getMapSizeY());
    }
    for(int angleIndex=0;angleIndex<numberOfDir;angleIndex++){
        for(int row=0;row<gaps->getSizeRows(angleIndex);row++){
            for(int index=0;index<gaps->getSizeGaps(angleIndex,row);index++){
                OpeningHandler::checkForDetection(angleIndex,row,index,map,transform,gaps);
            }
        }
    }
    OpeningHandler::getWalls(map);
    for(int i=0; i<OpeningHandler::detectionList.size();i++){
        if(OpeningHandler::detectionList[i].connectedWallStart!=NULL&&
           OpeningHandler::detectionList[i].connectedWallEnd!=NULL) continue;
        OpeningHandler::detectionList.erase(OpeningHandler::detectionList.begin()+i);
        i--;
    }
}

void OpeningHandler::checkForDetection(int angleIndex, int row, int index, MapHandler* map ,MapTransform* transform, GapHandler* gaps){
    scanGroup* gap =gaps->get(angleIndex,row,index);
    for(int direction=0;direction<2;direction++){
        vector<scanGroup*> connectedGaps=direction?gap->prevGroup:gap->nextGroup;
        if(connectedGaps.size()<2) continue;
        
        for(int i=0;i<connectedGaps.size();i++){
            if(!OpeningHandler::checkDepth(connectedGaps[i],direction,0)){
                connectedGaps.erase(connectedGaps.begin()+i);
                i--;
            }
        }
        if(connectedGaps.size()<2) continue;

        for(int i=0;i<connectedGaps.size();i++){
            opening newOp;
            if(!direction){
                newOp.start={connectedGaps[i]->end,connectedGaps[i]->row};
                newOp.end={connectedGaps[i]->start,connectedGaps[i]->row};
                if(i==0){
                    newOp.sideToMove=2;
                }else if(i==connectedGaps.size()){
                    newOp.sideToMove=1;
                }
            }else{
                newOp.start={connectedGaps[i]->start,connectedGaps[i]->row};
                newOp.end={connectedGaps[i]->end,connectedGaps[i]->row};
                if(i==0){
                    newOp.sideToMove=1;
                }else if(i==connectedGaps.size()-1){
                    newOp.sideToMove=2;
                }
            }
            newOp.angle=angleIndex;
            newOp=OpeningHandler::rotateOpening(newOp,angleIndex,transform);
            OpeningHandler::correctOpening(&newOp,map);
            OpeningHandler::detectionMap[newOp.start.x][newOp.start.y].push_back(OpeningHandler::detectionList.size());
            OpeningHandler::detectionMap[newOp.end.x][newOp.end.y].push_back(OpeningHandler::detectionList.size());
            if(dist(newOp.start,newOp.end)<minGroupSize) continue;
            OpeningHandler::detectionList.push_back(newOp);
        }

    }
}

bool OpeningHandler::checkDepth(scanGroup* gap, bool direction, int currentDepth){
    if(currentDepth>=minCoridorSize) return true;
    currentDepth++;

    vector<scanGroup*>* connectedGaps=direction?&gap->prevGroup:&gap->nextGroup;

    for(int i=0;i<connectedGaps->size();i++){
        if(OpeningHandler::checkDepth(connectedGaps->at(i),direction,currentDepth))
            return true;
    }

    return false;
}

opening OpeningHandler::rotateOpening(opening op, int angleIndex, MapTransform* transform){
    opening newOp;
    newOp=op;
    for(int sids=0; sids<2;sids++){
        point_int p=op.start;
        if(sids==1){
            p=op.end;
        }
        p=transform->getMapIndexTransform(p.x,p.y,angleIndex);
        if(sids==0){
            newOp.start=p;
        }else{
            newOp.end=p;
        }
    }
    
    return newOp; 
}

void OpeningHandler::correctOpening(opening *op, MapHandler* map){
    point_int *p1, *p2;
    for(int k=0; k<2; k++){
        if(k==0){
            p1=&op->start;
            p2=&op->end;
        }else{
            p1=&op->end;
            p2=&op->start;
        }
        
        //move points outside a wall 
        while (map->getMap(p1->x,p1->y)!=0||map->getMap(p1->x+1,p1->y)!=0 && map->getMap(p1->x-1,p1->y)!=0 &&
                map->getMap(p1->x,p1->y+1)!=0 && map->getMap(p1->x,p1->y-1)!=0){
            if(abs(p2->x-p1->x)>abs(p2->y-p1->y)&&abs(p2->x-p1->x)>2){
                p1->x+=(p2->x-p1->x)<0?-1:1;
            }
            else if(abs(p2->y-p1->y)>2){
                p1->y+=(p2->y-p1->y)<0?-1:1;
            }else{
                break;
            }
        }
        //moves point so they are next to a wall 
        point_int dir1={1,0},dir2={1,0};
        bool test=false;
        for(int d1=0;d1<4;d1++){
            if(map->getMap(p1->x+dir1.x,p1->y+dir1.y)!=MAP_UNOCCUPIED){
                test=true;
                break;
            }
            dir1={-dir1.y,dir1.x};
        }
        if(test) continue;
        bool done=false;
        for(int d1=0;d1<8 && !done;d1++){
            if(map->getMap(p1->x+dir1.x,p1->y+dir1.y)!=MAP_UNOCCUPIED) continue;
            for(int d2=0;d2<4;d2++){
                if(map->getMap(p1->x+dir1.x+dir2.x,p1->y+dir1.y+dir2.y)!=MAP_UNOCCUPIED){
                    *p1={p1->x+dir1.x,p1->y+dir1.y};
                    done=true;
                    break;
                }
                dir2={-dir2.y,dir2.x};
            }
            dir1=rotate_dir(dir1,true);
            if(d1==7 && !done) *p2=*p1;//make the opening smaller then minsize so it will be removed
        }
    }
}

void OpeningHandler::getWalls(MapHandler* map){
    vector<point_int> fillPoints;
    for(int detectIndex=0;detectIndex<detectionList.size();detectIndex++){
        for(int side=0;side<2;side++){
            if(OpeningHandler::detectionList[detectIndex].label==10) continue;
            if(side && OpeningHandler::detectionList[detectIndex].connectedWallIndexStart!=-1 ||
               !side && OpeningHandler::detectionList[detectIndex].connectedWallIndexEnd!=-1) continue;
            point_int p=side?detectionList[detectIndex].start:detectionList[detectIndex].end;
            OpeningHandler::getAndFilterWall(map, p, &fillPoints);
        }
    }
    for(int i=0;i<fillPoints.size();i++){
        map->setMap(fillPoints[i].x,fillPoints[i].y,MAP_UNOCCUPIED);
    }
    for(int i=OpeningHandler::detectionList.size()-1;i>=0;i--){
        if(OpeningHandler::detectionList[i].label!=10) continue;

        OpeningHandler::detectionList.erase(OpeningHandler::detectionList.begin()+i);
    }
}

void OpeningHandler::getAndFilterWall(MapHandler* map, point_int start, vector<point_int>* fillPoints){
    ant_data step;
    step.end=start;
    wall* newWall=new wall;
    vector<int> connectedDetections;
    vector<point_int> wallPoints;
    for(int s=0;s<2000000;s++){
        step=map->ant_step(step,true);
        wallCell w;
        w.emptyNeighbour=step.emty_cell;
        w.position=step.end;
        wallPoints.push_back(step.end);
        newWall->wall.push_back(w);
        for(int i=0;i<detectionMap[step.end.x][step.end.y].size();i++){
            int detectIndex=detectionMap[step.end.x][step.end.y][i];
            if(detectionList[detectIndex].start==step.end){
                detectionList[detectIndex].connectedWallStart=newWall;
                detectionList[detectIndex].connectedWallIndexStart=newWall->wall.size()-1;
            }
            if(detectionList[detectIndex].end==step.end){
                detectionList[detectIndex].connectedWallEnd=newWall;
                detectionList[detectIndex].connectedWallIndexEnd=newWall->wall.size()-1;
            }
            connectedDetections.push_back(detectIndex);
        }
        if(step.end==start) break;
    }
    if(newWall->wall.size()==2000000){
        ROS_INFO("----");
        ROS_INFO("%i, %i",start.x,start.y);
        ROS_INFO("%i",map->getMap(start.x,start.y));
        for(int i=0;i<20;i++){
            ROS_INFO("%i, %i",wallPoints[i].x,wallPoints[i].y);
        }
    }
    if(newWall->wall.size()>objectFilterMaxStep){
        OpeningHandler::wallList.push_back(newWall);
        return;
    } 

    for(int i=0; i<connectedDetections.size();i++){
        OpeningHandler::detectionList[connectedDetections[i]].label=10;
    }
    vector<point_int> fp=fillPoly(wallPoints);
    int fillPointsSize=fillPoints->size();
    fillPoints->resize(fillPointsSize+fp.size());
    for(int i=0;i<fp.size();i++){
        fillPoints->at(fillPointsSize+i)=fp[i];
    }
    delete newWall;
}

void OpeningHandler::update(MapHandler* map){
    vector<opening> dList=OpeningHandler::detectionList;
    //Find Openings
    while(dList.size()!=0){
        vector<int> ignorlist;
        if(!OpeningHandler::findOpenings(map,0,&dList,&ignorlist)){
            //oplist[0].label=5;
            OpeningHandler::add(dList[0]);
            dList.erase(dList.begin());
        }
    }
    //Opening Optimization
    //OpeningHandler::fitNonFixedOpenings(map);
    OpeningHandler::fixOverlapingPoints(map);
    //Fix overlap
    for(int i=0;i<OpeningHandler::size();i++){
        if(OpeningHandler::openingList[i]->label>10) continue;
        for(int j=0;j<OpeningHandler::size();j++){
            if(j==i) continue;
            if(OpeningHandler::openingList[j]->label>10) continue;
            if(!OpeningHandler::intersectOpenings(OpeningHandler::openingList[i],OpeningHandler::openingList[j])) continue;
            
            OpeningHandler::fixOverlap(OpeningHandler::openingList[i],OpeningHandler::openingList[j],map);
        }
    }
    //Remove openings that still overlap
    for(int i=0;i<OpeningHandler::size();i++){
        if(OpeningHandler::openingList[i]->label>10) continue;
        if(dist(OpeningHandler::openingList[i]->start,OpeningHandler::openingList[i]->end)<minGroupSize){
            OpeningHandler::openingList[i]->label=14;
            continue;
        }
        for(int j=0;j<OpeningHandler::size();j++){
            if(j==i) continue;
            if(OpeningHandler::openingList[j]->label>10) continue;
            if(!OpeningHandler::intersectOpenings(OpeningHandler::openingList[i],OpeningHandler::openingList[j])) continue;
            if(dist(OpeningHandler::openingList[i]->start,OpeningHandler::openingList[i]->end)<
               dist(OpeningHandler::openingList[j]->start,OpeningHandler::openingList[j]->end)){
                OpeningHandler::openingList[j]->label=18;
            }else{
                OpeningHandler::openingList[i]->label=18;
            }
        }
    }
}

bool OpeningHandler::findOpenings(MapHandler* map,int targetIndex, vector<opening> *dList, vector<int> *ignorList){
    while (true){
        if(dList->at(targetIndex).sideToMove==3){
            return false;
        }
        vector<int> interOpIndex;
        vector<point_int> interPoint;
        for(int index=0;index<dList->size();index++){
            if(targetIndex==index) continue;
            if(dList->at(index).sideToMove==3) continue;
            if(dList->at(targetIndex).sideToMove==dList->at(index).sideToMove) continue;
            if(!OpeningHandler::intersectOpenings(&dList->at(targetIndex),&dList->at(index))) continue;
            interOpIndex.push_back(index);
            interPoint.push_back(OpeningHandler::findIntersectionPoint(dList->at(targetIndex),dList->at(index)));
        }
        if(interOpIndex.size()==0){
            return false;
        }
        opening bestNewOp;
        double bestScore=-1;
        int selectedIndex=-1;
        for(int i=0; i<interOpIndex.size();i++){
            opening newOp;
            if(dList->at(targetIndex).sideToMove==1){
                newOp.start=dList->at(interOpIndex[i]).start;
                newOp.connectedWallIndexStart=dList->at(interOpIndex[i]).connectedWallIndexStart;
                newOp.connectedWallStart=dList->at(interOpIndex[i]).connectedWallStart;
                newOp.end=dList->at(targetIndex).end;
                newOp.connectedWallIndexEnd=dList->at(targetIndex).connectedWallIndexEnd;
                newOp.connectedWallEnd=dList->at(targetIndex).connectedWallEnd;
            }else{
                newOp.start=dList->at(targetIndex).start;
                newOp.connectedWallIndexStart=dList->at(targetIndex).connectedWallIndexStart;
                newOp.connectedWallStart=dList->at(targetIndex).connectedWallStart;
                newOp.end=dList->at(interOpIndex[i]).end;
                newOp.connectedWallIndexEnd=dList->at(interOpIndex[i]).connectedWallIndexEnd;
                newOp.connectedWallEnd=dList->at(interOpIndex[i]).connectedWallEnd;
            }
            if(dist(newOp.start,newOp.end)<minGroupSize) continue;
            double score=dist(newOp.start,newOp.end);
            
            //newOplist->push_back(newOp);
            if(bestScore<0||score<bestScore){
                if((newOp.end.x-newOp.start.x)*(interPoint[i].y-newOp.start.y)-
                    (newOp.end.y-newOp.start.y)*(interPoint[i].x-newOp.start.x)>0) continue;
                if(OpeningHandler::checkForWall(newOp,map)) continue;
                //if(OpeningHandler::check_unnecessary_openings(newOp,map)) continue;
                selectedIndex=interOpIndex[i];
                bestNewOp=newOp;
                bestScore=score;
            }
        }
        if(selectedIndex==-1){
            return false;
        }
        for(int i=0; i<ignorList->size(); i++){
            if(ignorList->at(i)==selectedIndex) return false;
        }
        ignorList->push_back(targetIndex);
        bool check=findOpenings(map,selectedIndex,dList,ignorList);
        ignorList->pop_back();
        if(check) continue;
        OpeningHandler::add(bestNewOp);
        dList->erase(dList->begin()+std::max(targetIndex,selectedIndex));
        dList->erase(dList->begin()+std::min(targetIndex,selectedIndex));
        for(int i=0; i<ignorList->size(); i++){
            if(ignorList->at(i)>targetIndex) ignorList->at(i)--;
            if(ignorList->at(i)>selectedIndex) ignorList->at(i)--;
        }
        return true;
    }
}

point_int OpeningHandler::findIntersectionPoint(opening o1, opening o2) {
    // Line 1: y = m1*x + b1, where m1 = (y2 - y1)/(x2 - x1) and b1 = y1 - m1*x1
    double m1, b1;
    if(std::abs(o1.end.x-o1.start.x)<1e-9){  // Line 1 is vertical
        m1=INFINITY;
        b1=o1.start.x;
    } else {
        m1=(o1.end.y-o1.start.y)/(o1.end.x-o1.start.x);
        b1=o1.start.y-m1*o1.start.x;
    }
    // Line 2: y = m2*x + b2, where m2 = (y4 - y3)/(x4 - x3) and b2 = y3 - m2*x3
    double m2, b2;
    if (std::abs(o2.end.x - o2.start.x) < 1e-9){  // Line 1 is vertical
        m2 = INFINITY;
        b2 = o2.start.x;
    } else {
        m2=(o2.end.y-o2.start.y)/(o2.end.x-o2.start.x);
        b2=o2.start.y-m2*o2.start.x;
    }
    // Find intersection
    point p;
    if(std::isinf(m1)){  // Line 1 is vertical, so intersection point has x-coordinate of line 1
        p={b1,m2*b1+b2};
    }else if(std::isinf(m2)){  // Line 2 is vertical, so intersection point has x-coordinate of line 2
        p={b2,m1*b2+b1};
    }else {
        double x = (b2-b1)/(m1-m2);
        double y = m1*x + b1;  // or y = m2*x + b2
        p={x, y};
        
    }
    point_int p_int={int(std::round(p.x)),int(std::round(p.y))};
    return p_int;
}

bool OpeningHandler::check_unnecessary_openings(opening o, MapHandler* map){
    if(o.connectedWallEnd!=o.connectedWallStart) return false;

    int maxSteps=(dist(o.start,o.end)*openingRemoveScale);
    if(double(o.connectedWallEnd->getDistans(o.connectedWallIndexStart,o.connectedWallIndexStart))<maxSteps) return true;

    return false;
}

void OpeningHandler::fitNonFixedOpenings(MapHandler* map){
    for(int index=0;index<OpeningHandler::size();index++){
        if(OpeningHandler::openingList[index]->sideToMove!=3){
            vector<point_int> intersectP;
            vector<int> intersectIndex;
            for(int index2=0;index2<OpeningHandler::size();index2++){
                if(index==index2) continue;
                if(OpeningHandler::openingList[index2]->sideToMove!=3) continue;
                if(!OpeningHandler::intersectOpenings(OpeningHandler::openingList[index],OpeningHandler::openingList[index2])) continue;
                intersectP.push_back(findIntersectionPoint(*OpeningHandler::openingList[index],*OpeningHandler::openingList[index2]));
                intersectIndex.push_back(index2);
            }
            if(intersectP.size()>0){
                point_int testP=OpeningHandler::openingList[index]->sideToMove==1?
                                OpeningHandler::openingList[index]->end:
                                OpeningHandler::openingList[index]->start;
                double minLenght=dist(testP,intersectP[0]);
                int bestIndex=intersectIndex[0];
                for(int m=1;m<intersectP.size();m++){
                    double length=dist(testP,intersectP[m]);
                    if(length<minLenght){
                        minLenght=length;
                        bestIndex=intersectIndex[m];
                    }
                }
                point_int moveToP=OpeningHandler::openingList[index]->sideToMove==1?
                                    OpeningHandler::openingList[bestIndex]->end:
                                    OpeningHandler::openingList[bestIndex]->start;
                wall* moveToWall=OpeningHandler::openingList[index]->sideToMove==1?
                                                OpeningHandler::openingList[bestIndex]->connectedWallEnd:
                                                OpeningHandler::openingList[bestIndex]->connectedWallStart;
                int moveToWallIndex=OpeningHandler::openingList[index]->sideToMove==1?
                                                OpeningHandler::openingList[bestIndex]->connectedWallIndexEnd:
                                                OpeningHandler::openingList[bestIndex]->connectedWallIndexStart;
                opening newOp=*OpeningHandler::openingList[index];
                if(newOp.sideToMove==1){
                    newOp.start=moveToP;
                    newOp.connectedWallStart=moveToWall;
                    newOp.connectedWallIndexStart=moveToWallIndex;
                }else{
                    newOp.end=moveToP;
                    newOp.connectedWallEnd=moveToWall;
                    newOp.connectedWallIndexEnd=moveToWallIndex;
                }
                OpeningHandler::updateOpening(OpeningHandler::openingList[index],newOp);
                if(dist(OpeningHandler::openingList[index]->start,
                        OpeningHandler::openingList[index]->end)<minGroupSize ){
                    OpeningHandler::openingList[index]->label=14;
                }
                else if(OpeningHandler::checkForWall(*OpeningHandler::openingList[index],map)){
                    OpeningHandler::openingList[index]->label=12;
                }
            }
        }
    }
}

//move all opening point so non are on the same map cell
void OpeningHandler::fixOverlapingPoints(MapHandler* map){
    // move all points so they dont overlap
    for(int index=0; index<OpeningHandler::size();index++){
        for(int sids=0;sids<2;sids++){
            opening* targetOp=OpeningHandler::openingList[index];
            point_int targetP=sids? targetOp->end:targetOp->start;
            wall* targetWall=sids? targetOp->connectedWallEnd: targetOp->connectedWallStart;
            int targetWallIndex=sids? targetOp->connectedWallIndexEnd: targetOp->connectedWallIndexStart;
            if(OpeningHandler::checkForOpenings(&targetWall->wall[targetWallIndex])<=1) continue;
            bool cw=true;
            int wMove=1;
            for(int s=0;s<objectFilterMaxStep;s++){
                int dir=cw?1:-1;
                int tIndex=targetWallIndex+wMove*dir;
                wallCell* w=targetWall->getCell(&tIndex);
                if(OpeningHandler::checkForOpenings(w)==0){
                    point_int moveToP;
                    int moveToIndex;
                    for(int m=1;m<wMove;m++){
                        moveToP=w->position;
                        moveToIndex=tIndex;
                        tIndex-=dir;
                        w=targetWall->getCell(&tIndex);
                        for(int i=0;i<w->connectedtOpeningStart.size();i++){
                            opening newOp=*w->connectedtOpeningStart[i];
                            newOp.start=moveToP;
                            newOp.connectedWallIndexStart=moveToIndex;
                            OpeningHandler::updateOpening(w->connectedtOpeningStart[i],newOp);
                        }
                        for(int i=0;i<w->connectedtOpeningEnd.size();i++){
                            opening newOp=*w->connectedtOpeningEnd[i];
                            newOp.end=moveToP;
                            newOp.connectedWallIndexEnd=moveToIndex;
                            OpeningHandler::updateOpening(w->connectedtOpeningEnd[i],newOp);
                        }
                    }
                    wallCell* w2=targetWall->getCell(&targetWallIndex);
                    opening* opToMove=NULL;
                    bool moveStart=true;
                    double bestAngle=-1;
                    for(int i=0;i<w2->connectedtOpeningStart.size();i++){
                        opening* op=w2->connectedtOpeningStart[i];
                        double ang=angleBetweenLines(op->end,op->start,w->position,w2->position);
                        if(ang<bestAngle || bestAngle<0){
                            opToMove=op;
                            moveStart=true;
                            bestAngle=ang;
                        }
                    }
                    for(int i=0;i<w2->connectedtOpeningEnd.size();i++){
                        opening* op=w2->connectedtOpeningEnd[i];
                        double ang=angleBetweenLines(op->start,op->end,w->position,w2->position);
                        if(ang<bestAngle || bestAngle<0){
                            opToMove=op;
                            moveStart=false;
                            bestAngle=ang;
                        }
                    }
                    opening newOp=*opToMove;
                    if(moveStart){
                        newOp.start=w->position;
                        newOp.connectedWallIndexStart=tIndex;
                    }else{
                        newOp.end=w->position;
                        newOp.connectedWallIndexEnd=tIndex;
                    }
                    OpeningHandler::updateOpening(opToMove,newOp);
                    break;
                }
                cw=!cw;
                if(cw) wMove++;
            }
        }
    }

}

int OpeningHandler::checkForOpenings(wallCell* cell){
    return cell->connectedtOpeningEnd.size()+cell->connectedtOpeningStart.size();
}

bool OpeningHandler::checkForWall(opening o, MapHandler* map){
    for(int sids=0;sids<2;sids++){
        point_int dir={1,0};
        point_int testP=sids?o.end:o.start;
        point_int endP=sids?o.start:o.end;
        for(int d=0; d<4; d++){
            if(map->getMap(testP.x+dir.x,testP.y+dir.y)==MAP_OCCUPIED){
                point_int v={endP.x-testP.x,endP.y-testP.y};
                double ang=v.x*dir.x+v.y*dir.y;
                ang=ang/dist(testP,endP);
                ang=acos(ang);
                if(ang<(M_PI_4)) return true;
            }
            dir={-dir.y,dir.x};
        }
    }
    
    int wallcount=map->checkForWallRay(o.start,o.end);
    if(wallcount<1){
        return false;
    }else{
        return true;
    }
}

int OpeningHandler::size(){
    return OpeningHandler::openingList.size();
}

int OpeningHandler::gapDetectionsSize(){
    return OpeningHandler::detectionList.size();
}

opening* OpeningHandler::get(int index){
    if(index>OpeningHandler::openingList.size()) return NULL;

    return OpeningHandler::openingList[index];
}

opening* OpeningHandler::getDetection(int index){
    if(index>OpeningHandler::detectionList.size()) return NULL;

    return &OpeningHandler::detectionList[index];
}

void OpeningHandler::add(opening newOp){
    opening* opToAdd= new opening(newOp);
    OpeningHandler::openingList.push_back(opToAdd);
    OpeningHandler::connectToWall(opToAdd);
}

void OpeningHandler::remove(opening* op){
    for(int i=0;i<OpeningHandler::openingList.size();i++){
        if(OpeningHandler::openingList[i]!=op) continue;
        OpeningHandler::openingList.erase(OpeningHandler::openingList.begin()+i);
        break;    
    }
    delete op;
}

void OpeningHandler::updateOpening(opening* op, opening newOp){
    OpeningHandler::disconnectFromWall(op);
    *op=newOp;
    OpeningHandler::connectToWall(op);
}

void OpeningHandler::connectToWall(opening* op){
    op->connectedWallStart->wall[op->connectedWallIndexStart].connectedtOpeningStart.push_back(op);
    op->connectedWallEnd->wall[op->connectedWallIndexEnd].connectedtOpeningEnd.push_back(op);
}

void OpeningHandler::disconnectFromWall(opening* op){
    vector<opening*>* olist=&op->connectedWallStart->wall[op->connectedWallIndexStart].connectedtOpeningStart;
    for(int i=0;i<olist->size();i++){
        if(olist->at(i)!=op) continue;
        olist->erase(olist->begin()+i);
        break;
    }
    olist=&op->connectedWallEnd->wall[op->connectedWallIndexEnd].connectedtOpeningEnd;
    for(int i=0;i<olist->size();i++){
        if(olist->at(i)!=op) continue;
        olist->erase(olist->begin()+i);
        break;
    }
}

void OpeningHandler::clear(){
    OpeningHandler::detectionList.clear();
    for(int i=0;i<OpeningHandler::wallList.size();i++){
        delete OpeningHandler::wallList[i];
    }
    OpeningHandler::wallList.clear();

    for(int i1=0;i1<OpeningHandler::detectionMap.size();i1++){
        for(int i2=0;i2<OpeningHandler::detectionMap[i1].size();i2++){
            OpeningHandler::detectionMap[i1][i2].clear();
        }
    }

    for(int i=0; i<OpeningHandler::size();i++){
        delete OpeningHandler::openingList[i];
    }
    OpeningHandler::openingList.clear();
}

bool OpeningHandler::intersectOpenings(opening *o1,opening *o2){
    point_int p1Max, p1Min, p2Max,p2Min;
    double l=0.8;
    p1Max.x=std::max(o1->start.x,o1->end.x);
    p1Max.y=std::max(o1->start.y,o1->end.y);
    p1Min.x=std::min(o1->start.x,o1->end.x);
    p1Min.y=std::min(o1->start.y,o1->end.y);
    p2Max.x=std::max(o2->start.x,o2->end.x);
    p2Max.y=std::max(o2->start.y,o2->end.y);
    p2Min.x=std::min(o2->start.x,o2->end.x);
    p2Min.y=std::min(o2->start.y,o2->end.y);
    
    if(!(p1Min.x<=p2Max.x && p1Max.x>=p2Min.x &&
         p1Min.y<=p2Max.y && p1Max.y>=p2Min.y)) return false;
    if(o1->end==o2->end) return true;
    if(o1->start==o2->start) return true;
    if(o1->start==o2->end) return true;
    if(o1->end==o2->start) return true;
    double l1 = dist(o1->start,o1->end);
    double l2 = dist(o2->start,o2->end);
    point n1={((o1->end.x-o1->start.x)/l1)*l, ((o1->end.y-o1->start.y)/l1)*l};
    point n2={((o2->end.x-o2->start.x)/l2)*l, ((o2->end.y-o2->start.y)/l2)*l};
    point A={o1->start.x-n1.x,o1->start.y-n1.y}, B={o1->end.x+n1.x,o1->end.y+n1.y};
    point C={o2->start.x-n2.x,o2->start.y-n2.y}, D={o2->end.x+n2.x,o2->end.y+n2.y}; 
    return OpeningHandler::ccw(A,C,D)!=OpeningHandler::ccw(B,C,D) &&
           OpeningHandler::ccw(A,B,C)!=OpeningHandler::ccw(A,B,D);
}

bool OpeningHandler::ccw(point A,point B,point C){
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
}

void OpeningHandler::fixOverlap(opening *o1,opening *o2, MapHandler* map){
    int conection_lenght[]={-1,-1};//direction and distans to conection
    bool moveEndO2[2];
    for(int sides=0;sides<2;sides++){
        for(int dir=0;dir<2;dir++){
            wall* connectedWall1=sides?o1->connectedWallEnd:o1->connectedWallStart;
            wall* connectedWall2=dir?o2->connectedWallEnd:o2->connectedWallStart;
            if(connectedWall1!=connectedWall2) continue;
            int connectedIndex1=sides?o1->connectedWallIndexEnd:o1->connectedWallIndexStart;
            int connectedIndex2=dir?o2->connectedWallIndexEnd:o2->connectedWallIndexStart;
            int connectL=std::abs(connectedIndex1-connectedIndex2);
            if(connectL>searchLenghtOverlap) continue;
            if(conection_lenght[sides]!=-1 && connectL>=conection_lenght[sides]) continue;
            conection_lenght[sides]=connectL;
            moveEndO2[sides]=dir;
        }
    }
    //If overlapping opening doesnt share a wall with o then keep the shortest opening.
    
    if(conection_lenght[0]==-1 && conection_lenght[1]==-1){
        if(dist(o1->start,o1->end)<dist(o2->start,o2->end)){
            o2->label=16;
        }else{
            o1->label=16;
        }
        return;
    }
    //If openings share one or two common wall check which side should be moved.
    int sides=1;
    if(conection_lenght[1]==-1 || 
        conection_lenght[0]!=-1 && conection_lenght[0]<conection_lenght[1]){
        sides=0;
    }
    int c=0;
    opening newO1=*o1;
    opening newO2=*o2;
    point_int holder;
    int indexHolder;

    if(sides && moveEndO2[sides]){
        holder=newO1.end;
        indexHolder=newO1.connectedWallIndexEnd;
        newO1.end=newO2.end;
        newO1.connectedWallIndexEnd=newO2.connectedWallIndexEnd;
        newO2.end=holder;
        newO2.connectedWallIndexEnd=indexHolder;
    }else if(!sides && !moveEndO2[sides]){
        holder=newO1.start;
        indexHolder=newO1.connectedWallIndexStart;
        newO1.start=newO2.start;
        newO1.connectedWallIndexStart=newO2.connectedWallIndexStart;
        newO2.start=holder;
        newO2.connectedWallIndexStart=indexHolder;
    }else if(sides && !moveEndO2[sides]){
        holder=newO1.end;
        indexHolder=newO1.connectedWallIndexEnd;
        newO1.end=newO2.start;
        newO1.connectedWallIndexEnd=newO2.connectedWallIndexStart;
        newO2.start=holder;
        newO2.connectedWallIndexStart=indexHolder;
    }else if(!sides && moveEndO2[sides]){
        holder=newO1.start;
        indexHolder=newO1.connectedWallIndexStart;
        newO1.start=newO2.end;
        newO1.connectedWallIndexStart=newO2.connectedWallIndexEnd;
        newO2.end=holder;
        newO2.connectedWallIndexEnd=indexHolder;
    }

    if(!OpeningHandler::checkForWall(newO1,map)&&!OpeningHandler::checkForWall(newO2,map)){
        OpeningHandler::updateOpening(o1,newO1);
        OpeningHandler::updateOpening(o2,newO2);
    }
    //Check if the issue was sold otherwise delete the longest opening.
    if(intersect_line(o1,o2)){
        if(dist(o1->start,o1->end)<dist(o2->start,o2->end)){
            o2->label=16;
            
        }else{
            o1->label=16;
        }
    }
    return;
}