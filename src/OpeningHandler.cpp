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
                }else if(i==connectedGaps.size()-1){
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
            if(dist(newOp.start,newOp.end)<minGroupSize) continue;
            OpeningHandler::detectionMap[newOp.start.x][newOp.start.y].push_back(OpeningHandler::detectionList.size());
            OpeningHandler::detectionMap[newOp.end.x][newOp.end.y].push_back(OpeningHandler::detectionList.size());
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
        if(OpeningHandler::detectionList[detectIndex].label==10) continue;
        for(int side=0;side<2;side++){
            if(side && OpeningHandler::detectionList[detectIndex].connectedWallStart!=NULL ||
               !side && OpeningHandler::detectionList[detectIndex].connectedWallEnd!=NULL) continue;
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
    point_int firstEmpty={-1,-1};
    vector<opening> opToAdd;
    vector<wallCell*> cellsToEmty;
    vector<point_int> fronterToRemove;
    vector<int> detectionsToRemove;
    int occupideWall=0;
    step=map->ant_step(step,true);
    while(step.emty_cell){
        step=map->ant_step(step,true);
        if(step.end==start) break;
    }
    start=step.end;
    for(int s=0;s<2000000;s++){
        point_int oS=step.end;
        step=map->ant_step(step,true);
        wallCell w;
        w.emptyNeighbour=false;
        w.position=step.end;
        w.index=newWall->size();
        w.parent=newWall;
        if(!step.emty_cell) occupideWall++;
        if(firstEmpty.x==-1 && step.emty_cell){
            firstEmpty=step.end;
        }
        if(firstEmpty.x!=-1 && !step.emty_cell){
            if(dist(firstEmpty,step.end)>minFrontier){
                for(int i=0;i<cellsToEmty.size();i++){
                    cellsToEmty[i]->emptyNeighbour=true;
                }
                for(int i=0;i<detectionsToRemove.size();i++){
                    detectionList[detectionsToRemove[i]].label=10;
                }
                opening newOp;
                newOp.label=4;
                newOp.end=cellsToEmty[0]->position;
                newOp.connectedWallEnd=cellsToEmty[0];
                newOp.start=cellsToEmty.back()->position;
                newOp.connectedWallStart=cellsToEmty.back();
                opToAdd.push_back(newOp);
            }else{
                for(int i=0;i<fronterToRemove.size();i++){
                    map->setMap(fronterToRemove[i].x,fronterToRemove[i].y,MAP_OCCUPIED);
                }
            }
            firstEmpty={-1,-1};
            cellsToEmty.clear();
            detectionsToRemove.clear();
            fronterToRemove.clear();
        }
        

        wallPoints.push_back(step.end);
        wallCell* wp=newWall->push_back(w);
        if(firstEmpty.x!=-1){
            fronterToRemove.insert(fronterToRemove.end(),step.fronterPositions.begin(),step.fronterPositions.end());
            cellsToEmty.push_back(wp);
        }
        for(int i=0;i<detectionMap[step.end.x][step.end.y].size();i++){
            int detectIndex=detectionMap[step.end.x][step.end.y][i];
            if(detectionList[detectIndex].start==step.end){
                detectionList[detectIndex].connectedWallStart=wp;
                if(firstEmpty.x!=-1 && 
                   (detectionList[detectIndex].sideToMove==3 ||detectionList[detectIndex].sideToMove==2)){
                    detectionsToRemove.push_back(detectIndex);
                }
            }
            if(detectionList[detectIndex].end==step.end){
                detectionList[detectIndex].connectedWallEnd=wp;
                if(firstEmpty.x!=-1 && 
                   (detectionList[detectIndex].sideToMove==3 ||detectionList[detectIndex].sideToMove==1)){
                    detectionsToRemove.push_back(detectIndex);
                }
            }
            connectedDetections.push_back(detectIndex);
        }
        if(step.end==start){
            point_int test=map->ant_step(step,true).end;
            if(test==newWall->wall[0]->position)break;
        } 
    }
    if(newWall->wall.size()==2000000){
        //ROS_INFO("----");
        //ROS_INFO("%i, %i",start.x,start.y);
        //ROS_INFO("%i",map->getMap(start.x,start.y));
        for(int i=0;i<20;i++){
            //ROS_INFO("%i, %i",wallPoints[i].x,wallPoints[i].y);
        }
    }
    if(occupideWall>objectFilterMaxStep){
        newWall->index=OpeningHandler::wallList.size();
        OpeningHandler::wallList.push_back(newWall);
        for(int i=0;i<opToAdd.size();i++){
            OpeningHandler::add(opToAdd[i]);
        }
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
    newWall->clear();
    delete newWall;
}

void OpeningHandler::update(MapHandler* map){
    
    //Find Openings
    //ROS_INFO("h1");
    OpeningHandler::findOpenings(map);
    //ROS_INFO("h2");
    //Opening Optimization
    OpeningHandler::fixOverlapingPoints(map);
    //ROS_INFO("h3");
    //Fix overlap
    for(int i=0;i<OpeningHandler::size();i++){
        if(OpeningHandler::openingList[i]->label>10) continue;
        if(OpeningHandler::openingList[i]->label==4) continue;
        for(int j=0;j<OpeningHandler::size();j++){
            if(j==i) continue;
            if(OpeningHandler::openingList[j]->label>10) continue;
            if(OpeningHandler::openingList[j]->label==4) continue;
            if(!OpeningHandler::intersectOpenings(OpeningHandler::openingList[i],OpeningHandler::openingList[j])) continue;
            OpeningHandler::fixOverlap(OpeningHandler::openingList[i],OpeningHandler::openingList[j],map);
        }
    }
    //ROS_INFO("h4");
    //Remove openings that still overlap
    for(int i=0;i<OpeningHandler::size();i++){
        if(OpeningHandler::openingList[i]->label>10) continue;
        if(OpeningHandler::openingList[i]->label==4) continue;
        if(dist(OpeningHandler::openingList[i]->start(),OpeningHandler::openingList[i]->end())<minGroupSize){
            OpeningHandler::disable(OpeningHandler::openingList[i],14);
            i--;
            continue;
        }
        for(int j=0;j<OpeningHandler::size();j++){
            if(j==i) continue;
            if(OpeningHandler::openingList[j]->label>10) continue;
            if(!OpeningHandler::intersectOpenings(OpeningHandler::openingList[i],OpeningHandler::openingList[j])) continue;
            if(dist(OpeningHandler::openingList[i]->start(),OpeningHandler::openingList[i]->end())<
               dist(OpeningHandler::openingList[j]->start(),OpeningHandler::openingList[j]->end())&&
               OpeningHandler::openingList[j]->label!=4 || OpeningHandler::openingList[i]->label==4){
                OpeningHandler::disable(OpeningHandler::openingList[j],18);
                i--;
                break;
            }else{
                OpeningHandler::disable(OpeningHandler::openingList[i],18);
                i--;
                break;
            }
        }
    }
    //ROS_INFO("h5");
    for(int i=0;i<OpeningHandler::size();i++){
        if(OpeningHandler::openingList[i]->label>10) continue;
        if(OpeningHandler::openingList[i]->label==4) continue;
        if(!check_unnecessary_openings(OpeningHandler::openingList[i],map)) continue;
        OpeningHandler::remove(openingList[i]);
    }
    //ROS_INFO("h6");
    for(int i=0;i<OpeningHandler::size();i++){
        if(OpeningHandler::openingList[i]->label==1){
            opening newOp=OpeningHandler::openingList[i]->getOpening();
            newOp.flip();
            newOp.label=2;
            OpeningHandler::add(newOp);
        }else if(OpeningHandler::openingList[i]->label==4){
            opening newOp=OpeningHandler::openingList[i]->getOpening();
            newOp.flip();
            newOp.label=5;
            OpeningHandler::add(newOp);
        }
        
    }
    //ROS_INFO("h7");
}

void OpeningHandler::findOpenings(MapHandler* map){
    vector<opening> dList=OpeningHandler::detectionList;
    vector<int> ignorList;
    vector<int> emtyIgnorList;  
    for(int i=0; i<dList.size();i++){      
        
        opening op;
        int selectedIndex=checkForPotensialOpening(map,i,&dList,&ignorList,&op);

        if(selectedIndex==-1){
            if(!OpeningHandler::checkForWall(&dList.at(i),map) &&
               !dList.at(i).connectedWallStart->emptyNeighbour &&
               !dList.at(i).connectedWallEnd->emptyNeighbour)
                OpeningHandler::add(dList.at(i));
            dList.erase(dList.begin()+i);
            i--;
            continue;
        }
        
        int testIndex=checkForPotensialOpening(map,selectedIndex,&dList,&emtyIgnorList,NULL,true);
        if(testIndex!=i){
            ignorList.push_back(selectedIndex);
            i--;
            continue;
        }
        // if(testIndex==i){
        //     dList.erase(dList.begin()+std::max(i,selectedIndex));
        //     dList.erase(dList.begin()+std::min(i,selectedIndex));
        //     if(selectedIndex<i) i--;
        //     i--;
        // }
        // else{
        //     dList.erase(dList.begin()+i);
        //     i--;
        // }
        
        dList.erase(dList.begin()+std::max(i,selectedIndex));
        dList.erase(dList.begin()+std::min(i,selectedIndex));
        if(selectedIndex<i) i--;
        i--;

        ignorList.clear();
        OpeningHandler::add(op);
    }
}

bool checkForComonElement(vector<int> l1, int o2){
    for(int o1 : l1)
        if(o1==o2) return true;
    return false;
}

int OpeningHandler::checkForPotensialOpening(MapHandler* map,int targetIndex, vector<opening> *dList, vector<int> *ignorList, opening *op, bool returnFirst){
    opening bestNewOp;
    double bestScore=-1;
    int selectedIndex=-1;
    if(dList->at(targetIndex).sideToMove==3) return selectedIndex;
    for(int index=0;index<dList->size();index++){
        if(checkForComonElement(*ignorList,index)) continue;
        if(targetIndex==index) continue;
        if(dList->at(index).sideToMove==3) continue;
        if(dList->at(targetIndex).sideToMove==dList->at(index).sideToMove) continue;
        if(!OpeningHandler::intersectOpenings(&dList->at(targetIndex),&dList->at(index))) continue;
        point_int interPoint=OpeningHandler::findIntersectionPoint(dList->at(targetIndex),dList->at(index));

        opening newOp;
        if(dList->at(targetIndex).sideToMove==1){
            newOp.start=dList->at(index).start;
            newOp.connectedWallStart=dList->at(index).connectedWallStart;
            newOp.end=dList->at(targetIndex).end;
            newOp.connectedWallEnd=dList->at(targetIndex).connectedWallEnd;
        }else{
            newOp.start=dList->at(targetIndex).start;
            newOp.connectedWallStart=dList->at(targetIndex).connectedWallStart;
            newOp.end=dList->at(index).end;
            newOp.connectedWallEnd=dList->at(index).connectedWallEnd;
        }
        double score=dist(newOp.start,newOp.end);
        if(score<minGroupSize) continue;
        
        if(bestScore<0||score<bestScore){
            if((newOp.end.x-newOp.start.x)*(interPoint.y-newOp.start.y)-
                (newOp.end.y-newOp.start.y)*(interPoint.x-newOp.start.x)>0) continue;
            if(OpeningHandler::checkForWall(&newOp,map)) continue;
            //if(OpeningHandler::check_unnecessary_openings(newOp,map)) continue;
            selectedIndex=index;
            bestNewOp=newOp;
            bestScore=score;
            if(returnFirst && index!=0) break;
        }
    }
    if(op!=NULL) *op=bestNewOp;
    return selectedIndex;
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

bool OpeningHandler::check_unnecessary_openings(openingDetection* o, MapHandler* map){
    if(o->getConnection(true)->parent!=o->getConnection(false)->parent) return false;

    int maxSteps=(dist(o->getConnection(true)->position,o->getConnection(false)->position)*2);
    if(double(o->getConnection(true)->parent->getDistans(o->getConnection(true)->index,o->getConnection(false)->index))<maxSteps) return true;

    return false;
}

//move all opening point so non are on the same map cell
void OpeningHandler::fixOverlapingPoints(MapHandler* map){
    // move all points so they dont overlap
    for(int index=0; index<OpeningHandler::size();index++){
        for(int sids=0;sids<2;sids++){
            openingDetection* targetOp=OpeningHandler::openingList[index];
            point_int targetP=sids? targetOp->end():targetOp->start();
            wall* targetWall=targetOp->getConnection(!sids)->parent;
            int targetWallIndex=targetOp->getConnection(!sids)->index;
            if(OpeningHandler::checkForOpenings(targetWall->wall[targetWallIndex])<=1) continue;
            bool cw=true;
            int wMove=1;
            for(int s=0;s<objectFilterMaxStep;s++){
                int dir=cw?1:-1;
                int tIndex=targetWallIndex+wMove*dir;
                wallCell* w=targetWall->getCell(&tIndex);
                if(OpeningHandler::checkForOpenings(w)==0){
                    wallCell* moveToW;
                    for(int m=1;m<wMove;m++){
                        moveToW=w;
                        tIndex-=dir;
                        w=targetWall->getCell(&tIndex);
                        for(int i=0;i<w->connectedtOpeningStart.size();i++){
                            w->connectedtOpeningStart[i]->connect(true,moveToW);
                        }
                        for(int i=0;i<w->connectedtOpeningEnd.size();i++){
                            w->connectedtOpeningEnd[i]->connect(false,moveToW);
                        }
                    }
                    wallCell* w2=targetWall->getCell(&targetWallIndex);
                    openingDetection* opToMove=NULL;
                    bool moveStart=true;
                    double bestAngle=-1;
                    for(int i=0;i<w2->connectedtOpeningStart.size();i++){
                        openingDetection* op=w2->connectedtOpeningStart[i];
                        double ang=angleBetweenLines(op->end(),op->start(),w->position,w2->position);
                        if(ang<bestAngle || bestAngle<0){
                            opToMove=op;
                            moveStart=true;
                            bestAngle=ang;
                        }
                    }
                    for(int i=0;i<w2->connectedtOpeningEnd.size();i++){
                        openingDetection* op=w2->connectedtOpeningEnd[i];
                        double ang=angleBetweenLines(op->start(),op->end(),w->position,w2->position);
                        if(ang<bestAngle || bestAngle<0){
                            opToMove=op;
                            moveStart=false;
                            bestAngle=ang;
                        }
                    }
                    opToMove->connect(moveStart,w);
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

bool OpeningHandler::checkForWall(opening *o, MapHandler* map){
    vector<point_int> p=OpeningHandler::generateOpeningPoints(o);
    for(int i=0;i<p.size();i++){
        if(map->getMap(p[i].x,p[i].y)==MAP_OCCUPIED) return true;
    }
    return false;
}

int OpeningHandler::size(){
    return OpeningHandler::openingList.size();
}

int OpeningHandler::gapDetectionsSize(){
    return OpeningHandler::detectionList.size();
}

openingDetection* OpeningHandler::get(int index){
    if(index>OpeningHandler::openingList.size()) return NULL;

    return OpeningHandler::openingList[index];
}

opening* OpeningHandler::getDetection(int index){
    if(index>OpeningHandler::detectionList.size()) return NULL;

    return &OpeningHandler::detectionList[index];
}

openingDetection* OpeningHandler::add(opening newOp){
    if(newOp.start.x==-1||newOp.start.y==-1) return NULL;
    if(newOp.connectedWallEnd==NULL || newOp.connectedWallStart==NULL) return NULL;
    openingDetection* opToAdd= new openingDetection;
    opToAdd->label=newOp.label;
    opToAdd->connect(true,newOp.connectedWallStart);
    opToAdd->connect(false,newOp.connectedWallEnd);
    opToAdd->occupiedPoints=newOp.occupiedPoints;
    OpeningHandler::openingList.push_back(opToAdd);
    return opToAdd;
}

void OpeningHandler::remove(openingDetection* op){
    op->disconnect(true);
    op->disconnect(false);
    for(int i=0;i<OpeningHandler::openingList.size();i++){
        if(OpeningHandler::openingList[i]!=op) continue;
        OpeningHandler::openingList.erase(OpeningHandler::openingList.begin()+i);
        break;    
    }
    delete op;
}

void OpeningHandler::disable(openingDetection* op, int label){
    op->label=label;
    if(show_removed_openings) OpeningHandler::openingDebug.push_back(op->getOpening());
    OpeningHandler::remove(op);
}

void OpeningHandler::clear(){
    OpeningHandler::detectionList.clear();
    OpeningHandler::openingDebug.clear();
    for(int i=0;i<OpeningHandler::wallList.size();i++){
        OpeningHandler::wallList[i]->clear();
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

bool OpeningHandler::intersectOpenings(openingDetection *o1,openingDetection *o2){
    point_int p1Max, p1Min, p2Max,p2Min;
    double l=0;
    p1Max.x=std::max(o1->start().x,o1->end().x)+l;
    p1Max.y=std::max(o1->start().y,o1->end().y)+l;
    p1Min.x=std::min(o1->start().x,o1->end().x)-l;
    p1Min.y=std::min(o1->start().y,o1->end().y)-l;
    p2Max.x=std::max(o2->start().x,o2->end().x)+l;
    p2Max.y=std::max(o2->start().y,o2->end().y)+l;
    p2Min.x=std::min(o2->start().x,o2->end().x)-l;
    p2Min.y=std::min(o2->start().y,o2->end().y)-l;
    
    if(!(p1Min.x<=p2Max.x && p1Max.x>=p2Min.x &&
         p1Min.y<=p2Max.y && p1Max.y>=p2Min.y)) return false;
    if(o1->end()==o2->end()) return true;
    if(o1->start()==o2->start()) return true;
    if(o1->start()==o2->end()) return true;
    if(o1->end()==o2->start()) return true;     

    vector<point_int> p1,p2;
    p1= OpeningHandler::generateOpeningPoints(o1);
    p2= OpeningHandler::generateOpeningPoints(o2);
    for(int i1=0;i1<p1.size();i1++){
        for(int i2=0;i2<p2.size();i2++){
            if(p1[i1]==p2[i2]) return true;
        }
    }

    return false;
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

void OpeningHandler::fixOverlap(openingDetection *o1,openingDetection *o2, MapHandler* map){
    int conection_lenght[]={-1,-1};//direction and distans to conection
    bool moveEndO2[2];
    for(int sides=0;sides<2;sides++){
        for(int dir=0;dir<2;dir++){
            wallCell* connectedWall1=o1->getConnection(!sides);
            wallCell* connectedWall2=o2->getConnection(!dir);
            if(connectedWall1->parent!=connectedWall2->parent) continue;
            int connectedIndex1=connectedWall1->index;
            int connectedIndex2=connectedWall2->index;
            int connectL=connectedWall1->parent->getDistans(connectedIndex1,connectedIndex2);
            //if(connectL>searchLenghtOverlap) continue;
            if(conection_lenght[sides]!=-1 && connectL>=conection_lenght[sides]) continue;
            conection_lenght[sides]=connectL;
            moveEndO2[sides]=dir;
        }
    }
    //If overlapping opening doesnt share a wall with o then keep the shortest opening.
    
    if(conection_lenght[0]==-1 && conection_lenght[1]==-1){
        if(dist(o1->start(),o1->end())<dist(o2->start(),o2->end())){
            OpeningHandler::disable(o2,16);
        }else{
            OpeningHandler::disable(o1,16);
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
    opening newO1=o1->getOpening();
    opening newO2=o2->getOpening();
    point_int holder;
    int indexHolder;

    OpeningHandler::swapEnds(&newO1,!sides,&newO2,!moveEndO2[sides]);

    if(!OpeningHandler::checkForWall(&newO1,map)&&!OpeningHandler::checkForWall(&newO2,map)){
        wallCell* w1=o1->getConnection(!sides);
        wallCell* w2=o2->getConnection(!moveEndO2[sides]);
        o1->connect(!sides,w2);
        o2->connect(!moveEndO2[sides],w1);
        return;
    }
    //Check if the issue was sold otherwise delete the longest opening.
    if(OpeningHandler::intersectOpenings(o1,o2)){
        if(dist(o1->start(),o1->end())<dist(o2->start(),o2->end())){
            OpeningHandler::disable(o2,16);
            
        }else{
            OpeningHandler::disable(o1,16);
        }
    }
    return;
}

void OpeningHandler::swapEnds(opening* o1,bool swapStart1,opening* o2,bool swapStart2){
    point_int* o1p=swapStart1?&o1->start:&o1->end;
    point_int* o2p=swapStart2?&o2->start:&o2->end;

    wallCell** o1w=swapStart1?&o1->connectedWallStart:&o1->connectedWallEnd;
    wallCell** o2w=swapStart2?&o2->connectedWallStart:&o2->connectedWallEnd;

    point_int ph=*o1p;
    *o1p=*o2p;
    *o2p=ph;

    wallCell* wh=*o1w;
    *o1w=*o2w;
    *o2w=wh;
}

//retruns next wall cell on wall conected to op on 1: start 2: end. type is serching for a 1: start 2: end 3: both
wallCell OpeningHandler::getNextOpening(openingDetection* op, bool startSide, int type, bool cw,bool checkFirst, bool stopAtEmpty, vector<wallCell*>* pointList){
    int index=op->getConnection(startSide)->index;
    wall* targetWall=op->getConnection(startSide)->parent;
    int dir=cw?1:-1;

    if(checkFirst) index-=dir;

    for(int s=0;s<targetWall->size();s++){
        index+=dir;
        wallCell* w=targetWall->getCell(&index);
        if(pointList!=NULL) pointList->push_back(w);
        if(stopAtEmpty && w->emptyNeighbour)return *w;
        for(int i=0; i<w->connectedtOpeningStart.size();i++){
            if(w->connectedtOpeningStart[i]->label<10) continue;
            w->connectedtOpeningStart.erase(w->connectedtOpeningStart.begin()+i);
            i--;
        }
        for(int i=0; i<w->connectedtOpeningEnd.size();i++){
            if(w->connectedtOpeningEnd[i]->label<10) continue;
            w->connectedtOpeningEnd.erase(w->connectedtOpeningEnd.begin()+i);
            i--;
        }

        if((type==1 || type==3) && w->connectedtOpeningStart.size()>0) return *w;
        if((type==2 || type==3) && w->connectedtOpeningEnd.size()>0) return *w;
    }
    return *targetWall->getCell(&index);
}

vector<point_int> OpeningHandler::getPointsBetweenOpenings(openingDetection* o1, bool startAtStartO1, openingDetection* o2, bool startAtStartO2,vector<wallCell*>* wallPoints){
    vector<point_int> pointList;
    wall* o1w=o1->getConnection(startAtStartO1)->parent;
    wall* o2w=o2->getConnection(startAtStartO2)->parent;
    int o1i=o1->getConnection(startAtStartO1)->index;
    int o2i=o2->getConnection(startAtStartO2)->index;
    point_int o1p=startAtStartO1?o1->start():o1->end();
    point_int o2p=startAtStartO2?o2->start():o2->end();

    int numberOfSteps;
    if(o1i<=o2i){
        numberOfSteps=std::abs(o1i-o2i);
    }else{
        numberOfSteps=o1w->size()-o1i+o2i;
    }
    if(numberOfSteps>5000){
        o1->label=28;
        o2->label=28;
        return pointList;
    }
    int index=o1i;
    
    for(int s=0;s<numberOfSteps+1;s++){  
        if(wallPoints!=NULL) wallPoints->push_back(o1w->getCell(&index));
        pointList.push_back(o1w->getCell(&index)->position);
        if(index==o2i) break;
        index++;
    }
    return pointList;
}

void OpeningHandler::addCustomOpening(opening op,int id){
    for(int i=0;i<OpeningHandler::customOpeningList.size();i++){
        if(OpeningHandler::customOpeningIdList[i]!=id) continue;
        OpeningHandler::customOpeningList[i]=op;
        return;
    }
    OpeningHandler::customOpeningIdList.push_back(id);
    OpeningHandler::customOpeningList.push_back(op);
    return;
}

void OpeningHandler::removeCustomOpening(int id){
    for(int i=0;i<OpeningHandler::customOpeningList.size();i++){
        if(OpeningHandler::customOpeningIdList[i]!=id) continue;
        OpeningHandler::customOpeningList.erase(OpeningHandler::customOpeningList.begin()+i);
        OpeningHandler::customOpeningIdList.erase(OpeningHandler::customOpeningIdList.begin()+i);
        return;
    }
}

int OpeningHandler::maxCustomOpeningId(){
    int maxId=-1;
    for(int i=0;i<OpeningHandler::customOpeningList.size();i++){
        if(OpeningHandler::customOpeningIdList[i]<=maxId) continue;
        maxId=OpeningHandler::customOpeningIdList[i];
    }
    return maxId;
}

vector<point_int> OpeningHandler::generateOpeningPoints(openingDetection *o){
    vector<point_int> p;
    if(o->occupiedPoints.size()!=0 && 
    (o->start()==o->occupiedPoints[0] && 
    o->end()==o->occupiedPoints[o->occupiedPoints.size()-1]||
    o->end()==o->occupiedPoints[0] && 
    o->start()==o->occupiedPoints[o->occupiedPoints.size()-1])){
        
        p=o->occupiedPoints;
    
        return p;
    }

    p=drawLine(o->start(),o->end());
    o->occupiedPoints=p;
        
    return p;
}

vector<point_int> OpeningHandler::generateOpeningPoints(opening *o){
    vector<point_int> p;
    if(o->occupiedPoints.size()!=0 && 
    (o->start==o->occupiedPoints[0] && 
    o->end==o->occupiedPoints[o->occupiedPoints.size()-1]||
    o->end==o->occupiedPoints[0] && 
    o->start==o->occupiedPoints[o->occupiedPoints.size()-1])){
        
        p=o->occupiedPoints;
    
        return p;
    }

    p=drawLine(o->start,o->end);
    o->occupiedPoints=p;
        
    return p;
}