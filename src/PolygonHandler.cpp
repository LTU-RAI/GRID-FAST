#include "Utility.hh"
#include "PolygonHandler.hh"

PolygonHandler::PolygonHandler(){

}

PolygonHandler::~PolygonHandler(){

}

void PolygonHandler::updateIntersections(OpeningHandler* openingList, MapHandler* map){
    for(int opIndex=0;opIndex<openingList->size();opIndex++){
        openingDetection* op=openingList->get(opIndex);
        if(op->label>10) continue;
        if(op->parent!=NULL) continue;
        PolygonHandler::creatIntersection(openingList,op);
    }
    for(int opIndex=0;opIndex<openingList->size();opIndex++){
        openingDetection* op=openingList->get(opIndex);
        if(op->label<10) continue;
        openingList->disable(op,op->label);
        opIndex--;
    }
    for(int optTimes=0;optTimes<2;optTimes++){
        for(int pIndex=0;pIndex<PolygonHandler::polygonList.size();pIndex++){
            PolygonHandler::optimizeIntersection(PolygonHandler::polygonList[pIndex],openingList,map);
        }
    }
}

void PolygonHandler::creatIntersection(OpeningHandler* openingList,openingDetection* startOp){
    polygon newPoly;
    newPoly.openings.push_back(startOp);
    openingDetection* targetOp=startOp;
    while(true){
        wallCell w=openingList->getNextOpening(targetOp,true,3,true);
        //good connection
        if(w.connectedtOpeningEnd.size()>0){
            if(w.connectedtOpeningEnd[0]==startOp) break;
            for(int i=0;i<newPoly.openings.size();i++){
                if(newPoly.openings[i]==w.connectedtOpeningEnd[0]){
                    openingList->remove(newPoly.openings[i]);
                    return;
                }
            }
            newPoly.openings.push_back(w.connectedtOpeningEnd[0]);
            targetOp=w.connectedtOpeningEnd[0];
            continue;
        }
        //bad connection
        if(w.connectedtOpeningStart.size()>0){
            ROS_INFO("----------WARNING--------");
        }
    }
    if(newPoly.openings.size()<3){
        for(int i=0;i<newPoly.openings.size();i++){
            newPoly.openings[i]->label=28;
        }
        return;
    }
    newPoly.label=30;
    PolygonHandler::add(newPoly);
}

void PolygonHandler::getPathways(OpeningHandler* openingList){
    for(int index=0;index<PolygonHandler::polygonList.size();index++){
        for(int oIndex=0;oIndex<PolygonHandler::polygonList[index]->openings.size();index++){

        }
    }
}

void PolygonHandler::generatePolygonArea(OpeningHandler* openingList){
    for(int index=0;index<PolygonHandler::polygonList.size();index++){
        PolygonHandler::getArea(index,openingList);
    }
}

void PolygonHandler::getArea(int index, OpeningHandler* openingList){
    polygon* poly=PolygonHandler::polygonList[index];
    poly->polygon_points.clear();
    poly->polygon_points_desplay.clear();
    int openingsSize=poly->openings.size();
    for(int sideIndex=0;sideIndex<openingsSize;sideIndex++){
        vector<point_int> pointListOpening=fillPoints(poly->openings[sideIndex]->end(),
                                                      poly->openings[sideIndex]->start(),0.8);
        poly->polygon_points.insert(poly->polygon_points.end(),pointListOpening.begin(),pointListOpening.end());
        vector<point_int> pointList = openingList->getPointsBetweenOpenings(
                poly->openings[sideIndex],true,
                poly->openings[(sideIndex+1)%openingsSize],false);
        poly->polygon_points.insert(poly->polygon_points.end(),pointList.begin(),pointList.end());
        if(pointList.size()==0) continue;
        for(int i=0;i<pointList.size()-1;i+=polygonRez){
            poly->polygon_points_desplay.push_back(pointList[i]);
        }
        poly->polygon_points_desplay.push_back(pointList.back());
    }
    poly->center=PolygonHandler::getPolygonCenter(poly->polygon_points);
}
point_int PolygonHandler::getPolygonCenter(vector<point_int> sList){
    point_int center={0,0};
    for(int i=0; i<sList.size();i++){
        center.x+=sList[i].x;
        center.y+=sList[i].y;
    }
    center.x=center.x/(sList.size());
    center.y=center.y/(sList.size());

    return center;
}

int PolygonHandler::size(){
    return PolygonHandler::polygonList.size();
}

polygon* PolygonHandler::get(int index){
    return PolygonHandler::polygonList[index];
} 

polygon* PolygonHandler::add(polygon newPoly){
    polygon* polyToAdd=new polygon(newPoly);
    for(int i=0;i<polyToAdd->openings.size();i++){
        polyToAdd->openings[i]->parent=polyToAdd;
        //polyToAdd->openings[i]->label=PolygonHandler::polygonList.size()%8;
    }
    //polyToAdd->label=PolygonHandler::polygonList.size()%8;
    PolygonHandler::polygonList.push_back(polyToAdd);
    return polyToAdd;
}

void PolygonHandler::remove(polygon* poly, OpeningHandler* openingList){
    for(int i=0;i<poly->openings.size();i++){
        openingList->disable(poly->openings[i],26);
    }
    for(int i=0;i<PolygonHandler::polygonList.size();i++){
        if(PolygonHandler::polygonList[i]!=poly) continue;
        PolygonHandler::polygonList.erase(PolygonHandler::polygonList.begin()+i);
        break;    
    }
    delete poly;
}

void PolygonHandler::clear(){
    for(int i=0; i<PolygonHandler::polygonList.size(); i++){
        delete PolygonHandler::polygonList[i];
    }
    PolygonHandler::polygonList.clear();
}

void PolygonHandler::optimizeIntersection(polygon* poly,OpeningHandler* openingList, MapHandler* map){
    vector<vector<wallCell*>> walls;
    vector<int> startIndex;
    walls.resize(poly->openings.size());
    bool del=false;
    for(int sideIndex=0;sideIndex<poly->openings.size();sideIndex++){
        openingDetection* targetOp1=poly->openings[sideIndex];
        openingDetection* targetOp2=poly->openings[(sideIndex+1)%poly->openings.size()];
        vector<wallCell*> currentWall;
        vector<wallCell*> o1back;
        wallCell w=openingList->getNextOpening(targetOp1,true,2,false,true,true,&o1back);
        int loopfrom=o1back.size()-1;
        if(w.connectedtOpeningEnd.size()!=0){
            if(w.connectedtOpeningEnd[0]==targetOp1) loopfrom=loopfrom/2;
        }
        
        for(int i=loopfrom;i>=0;i--){
            currentWall.push_back(o1back[i]);
        }
        vector<wallCell*> o1ToO2;
        openingList->getPointsBetweenOpenings(targetOp1,true,targetOp2,false,&o1ToO2);
        currentWall.insert(currentWall.end(),o1ToO2.begin()+1,o1ToO2.end()-1);

        vector<wallCell*> o2front;
        w=openingList->getNextOpening(targetOp2,false,1,true,true,true,&o2front);
        int loopto=o2front.size();
        if(w.connectedtOpeningStart.size()!=0){
            if(w.connectedtOpeningStart[0]==targetOp2) loopto=loopto/2;
        }
        
        if(o2front.size()!=0) currentWall.insert(currentWall.end(),o2front.begin(),o2front.begin()+loopto);
        walls[sideIndex]=currentWall;
    }
    point_int centerP={0,0};
    vector<vector<wallCell*>> startPoints,endPoints;
    startPoints.resize(walls.size());
    endPoints.resize(walls.size());
    for(int sideIndex=0;sideIndex<poly->openings.size();sideIndex++){
        openingDetection* targetOp1=poly->openings[sideIndex];
        openingDetection* targetOp2=poly->openings[(sideIndex+1)%poly->openings.size()];
        double bestScore=-1;
        int bestINdex=0;
        for(int wallIndex=0;wallIndex<walls[sideIndex].size()-1;wallIndex++){
            double d1=dist(walls[sideIndex][wallIndex]->position,targetOp1->end());
            double d2=dist(walls[sideIndex][wallIndex+1]->position,targetOp2->start());
            double score=d1*d1+d2*d2;
            if(bestScore<0||score<bestScore){
                bestScore=score;
                bestINdex=wallIndex;
            }
        }
        startIndex.push_back(bestINdex);
        targetOp1->connect(true,walls[sideIndex][bestINdex]);
        targetOp2->connect(false,walls[sideIndex][bestINdex+1]);
        centerP.x+=walls[sideIndex][bestINdex]->position.x+walls[sideIndex][bestINdex+1]->position.x;
        centerP.y+=walls[sideIndex][bestINdex]->position.y+walls[sideIndex][bestINdex+1]->position.y;
        startPoints[sideIndex].resize(bestINdex+1);
        for(int i=bestINdex;i>=0;i--){
            startPoints[sideIndex][bestINdex-i]=walls[sideIndex][i];
        }
        int esize=walls[sideIndex].size()-bestINdex-1;
        endPoints[(sideIndex+1)%poly->openings.size()].resize(esize);
        for(int i=0;i<esize;i++){
            endPoints[(sideIndex+1)%poly->openings.size()][i]=walls[sideIndex][bestINdex+1+i];
        }

    }
    centerP.x=centerP.x/(poly->openings.size()*2);
    centerP.y=centerP.y/(poly->openings.size()*2);
    int listIndex=0;
    for(int sideIndex=0;sideIndex<poly->openings.size();sideIndex++){
        openingDetection* op=poly->openings[sideIndex];
        vector<wallCell*> startS,endS;
        startS=startPoints[listIndex];
        endS=endPoints[listIndex];
        listIndex++;
        opening test, hbest, best;
        test=op->getOpening();
        best=test;
        hbest=test;
        double bestScore=0, bestLength=0;
        bool first=true, changed=false;
        int sIndex=0, eIndex=0, decrisCount=0;
        while(true){
            if(sIndex+1<startS.size() && (eIndex+1>=endS.size() || 
                dist(startS[sIndex+1]->position,endS[eIndex]->position)<
                dist(startS[sIndex]->position,endS[eIndex+1]->position))){
                sIndex+=1;
            }else if(eIndex+1<endS.size()){
                eIndex+=1;
            }else{
                best=hbest;
                changed=true;
                break;
            }
            test.start=startS[sIndex]->position;
            test.connectedWallStart=startS[sIndex];
            test.end=endS[eIndex]->position;
            test.connectedWallEnd=endS[eIndex];
            double lenght=dist(test.start,test.end);
            if(lenght<minGroupSize) break;
            if(first || lenght<bestLength){
                bestLength=lenght;
            }else{
                if(decrisCount>=4){
                    best=hbest;
                    changed=true;
                    decrisCount=0;
                }else decrisCount+=1;
            }
            double score=lenght+dw*DFunction(dist(test.get_center(),centerP)); //dist(test.get_center(),poly_list[pIndex].center);
            if(first || score<bestScore){
                if(openingList->checkForWall(test,map)) continue;
                first=false;
                bestScore=score;
                hbest=test;
            }
        }
        if(!openingList->checkForWall(best,map)){
            op->connect(true,best.connectedWallStart);
            op->connect(false,best.connectedWallEnd);
        }
        if(!changed){
            if(poly->openings.size()<=3){
                PolygonHandler::remove(poly,openingList);
                break;
            }else{
                PolygonHandler::removeSideFromPolygon(poly,op,openingList);
                sideIndex-=1;
            }
        }
    }
}
double PolygonHandler::DFunction(double length){
    double minDistToCenter=6;
    double maxPenalty=1000;
    if(length>minDistToCenter) return (length-minDistToCenter);
    
    return maxPenalty-(maxPenalty/minDistToCenter*length);
}

void PolygonHandler::removeSideFromPolygon(polygon* poly,openingDetection* op, OpeningHandler* openingList){
    for(int i=0;i<poly->openings.size();i++){
        if(poly->openings[i]!=op) continue;
        poly->openings.erase(poly->openings.begin()+i);
        openingList->disable(op,26);
        return;
    }
}
