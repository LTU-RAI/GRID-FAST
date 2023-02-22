#include "Utility.hh"
#include "PolygonHandler.hh"

PolygonHandler::PolygonHandler(){

}

PolygonHandler::~PolygonHandler(){

}

void PolygonHandler::updateIntersections(OpeningHandler* openingList){
    for(int opIndex=0;opIndex<openingList->size();opIndex++){
        opening* op=openingList->get(opIndex);
        if(op->label>10) continue;
        if(op->parent_polygon!=NULL) continue;
        PolygonHandler::creatIntersection(openingList,op);
    }
    ROS_INFO("%li",PolygonHandler::polygonList.size());
}

void PolygonHandler::creatIntersection(OpeningHandler* openingList,opening* startOp){
    polygon newPoly;
    newPoly.openings.push_back(startOp);
    opening* targetOp=startOp;
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
            for(int i=0;i<newPoly.openings.size();i++){
                if(newPoly.openings[i]==w.connectedtOpeningStart[0]){
                    openingList->remove(newPoly.openings[i]);
                    return;
                }
            }
            opening newOp;
            newOp.label=2;
            newOp.start=w.connectedtOpeningStart[0]->end;
            newOp.connectedWallStart=w.connectedtOpeningStart[0]->connectedWallEnd;
            newOp.connectedWallIndexStart=w.connectedtOpeningStart[0]->connectedWallIndexEnd;
            newOp.end=w.connectedtOpeningStart[0]->start;
            newOp.connectedWallEnd=w.connectedtOpeningStart[0]->connectedWallStart;
            newOp.connectedWallIndexEnd=w.connectedtOpeningStart[0]->connectedWallIndexStart;
            opening* op=openingList->add(newOp);
            newPoly.openings.push_back(op);
            targetOp=op;
            continue;
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
        vector<point_int> pointListOpening=fillPoints(poly->openings[sideIndex]->end,
                                                      poly->openings[sideIndex]->start,0.8);
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
        polyToAdd->openings[i]->parent_polygon=polyToAdd;
    }
    PolygonHandler::polygonList.push_back(polyToAdd);
    return polyToAdd;
}

void PolygonHandler::clear(){
    for(int i=0; i<PolygonHandler::polygonList.size(); i++){
        delete PolygonHandler::polygonList[i];
    }
    PolygonHandler::polygonList.clear();
}

void optimizeIntersection(polygon* poly,OpeningHandler* openingList, MapHandler* map){
    vector<vector<point_int>> walls;
    vector<vector<int>> wallIndex;
    vector<int> startIndex;
    walls.resize(poly->openings.size());
    wallIndex.resize(poly->openings.size());
    bool del=false;
    for(int sideIndex=0;sideIndex<poly->openings.size();sideIndex++){
        opening* targetOp1=poly->openings[sideIndex];
        opening* targetOp2=poly->openings[(sideIndex+1)%poly->openings.size()];
        vector<point_int> currentWall;
        vector<point_int> o1back;
        wallCell w=openingList->getNextOpening(targetOp1,true,2,false,true,&o1back);

        int loopfrom=o1back.size()-1;
        if(w.connectedtOpeningEnd[0]==targetOp1) loopfrom=loopfrom/2;
        for(int i=loopfrom;i>=0;i--){
            currentWall.push_back(o1back[i]);
        }
        int o2wallIndexPos=currentWall.size()-1;
        vector<point_int> o1ToO2=openingList->getPointsBetweenOpenings(targetOp1,true,targetOp2,false);
        currentWall.insert(currentWall.end(),o1ToO2.begin(),o1ToO2.end());

        vector<point_int> o2front;
        w=openingList->getNextOpening(targetOp2,false,1,true,true,&o2front);
        int loopto=o1back.size()-1;
        if(w.connectedtOpeningStart[0]==targetOp2) loopto=loopto/2;
        if(o1back.size()!=0) currentWall.insert(currentWall.end(),o2front.begin(),o2front.begin()+loopto);

        walls[sideIndex]=currentWall;

        wallIndex[sideIndex].resize(currentWall.size());
        for(int i=0;i<wallIndex[sideIndex].size();i++){
            wallIndex[sideIndex][i]=targetOp2->connectedWallIndexEnd+(i-o2wallIndexPos);
        }
    }
    for(int sideIndex=0;sideIndex<poly->openings.size();sideIndex++){
        opening* targetOp1=poly->openings[sideIndex];
        opening* targetOp2=poly->openings[(sideIndex+1)%poly->openings.size()];
        double bestScore=-1;
        int bestINdex=0;
        for(int wallIndex=0;wallIndex<walls[sideIndex].size()-1;wallIndex++){
            double d1=dist(walls[sideIndex][wallIndex],targetOp1->end);
            double d2=dist(walls[sideIndex][wallIndex+1],targetOp2->start);
            double score=d1*d1+d2*d2;
            if(bestScore<0||score<bestScore){
                bestScore=score;
                bestINdex=wallIndex;
            }
        }
        startIndex.push_back(bestINdex);
        oplist[wallSharingOpenings[sideIndex].x].start=walls[sideIndex][bestINdex];
        oplist[wallSharingOpenings[sideIndex].y].end=walls[sideIndex][bestINdex+1];
    }
    poly_list[pIndex]=creat_polygon_area(poly_list[pIndex],pIndex);
    if(poly_list[pIndex].polygon_points.size()==0){
        poly_list=remove_parent_polygon(poly_list,poly_list[pIndex].sidesIndex[0], true, true);
        return poly_list;
    }
    poly_list[pIndex].center=get_polygon_center(poly_list[pIndex].polygon_points);
    int listIndex=0;
    for(int sideIndex=0;sideIndex<poly_list[pIndex].sidesIndex.size();sideIndex++){
        int opIndex=poly_list[pIndex].sidesIndex[sideIndex];
        vector<point_int>* secondList;
        int secondStart=-1;
        for(int i=0;i<walls.size();i++){
            if(opIndex==wallSharingOpenings[i].y){
                secondList=&walls[i];
                secondStart=startIndex[i];
            }
        }
        if(secondStart==-1 || secondList->size()<2){
            poly_list=remove_parent_polygon(poly_list,opIndex, true, true);
            break;
        }
        vector<point_int> startS,endS;
        for(int i=startIndex[listIndex];i>=0;i--){
            startS.push_back(walls[listIndex][i]);
        }
        for(int i=secondStart+1;i<secondList->size();i++){
            endS.push_back(secondList->at(i));
        }
        listIndex++;
        opening test, hbest, best;
        test=oplist[opIndex];
        best=test;
        hbest=test;
        double bestScore=0, bestLength=0;
        bool first=true, changed=false;
        int sIndex=0, eIndex=0, decrisCount=0;
        while(true){
            //ROS_INFO("%i, %i, %i, %i",startS[sI].x,startS[sI].y,endS[eI].x,endS[eI].y);
            if(sIndex+1<startS.size() && (eIndex+1>=endS.size() || 
                dist(startS[sIndex+1],endS[eIndex])<dist(startS[sIndex],endS[eIndex+1]))){
                sIndex+=1;
            }else if(eIndex+1<endS.size()){
                eIndex+=1;
            }else{
                best=hbest;
                changed=true;
                break;
            }
            test.start=startS[sIndex];
            test.end=endS[eIndex];
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
            double score=lenght+dw*DFunction(dist(test.get_center(),poly_list[pIndex].center)); //dist(test.get_center(),poly_list[pIndex].center);
            if(first || score<bestScore){
                first=false;
                bestScore=score;
                hbest=test;
            }
            
        }
        //ROS_INFO("%i, %i, %i, %i",best.start.x,best.start.y,best.end.x,best.end.y);
        if(!checkForWall(best,1,topMap))
        oplist[opIndex]=best;
        if(!changed){
            if(poly_list[pIndex].sidesIndex.size()<=3){
                poly_list=remove_parent_polygon(poly_list,opIndex, true, true);
                break;
            }else{
                oplist[opIndex].parent_polygon=-1;
                oplist[opIndex].label=26;
                poly_list[pIndex].sidesIndex.erase(poly_list[pIndex].sidesIndex.begin()+sideIndex);
                sideIndex-=1;
            }
        }
    }
    poly_list[pIndex]=creat_polygon_area(poly_list[pIndex],pIndex);
    if(poly_list[pIndex].polygon_points.size()==0){
        poly_list=remove_parent_polygon(poly_list,poly_list[pIndex].sidesIndex[0], true, true);
        return poly_list;
    }
    poly_list[pIndex].center=get_polygon_center(poly_list[pIndex].polygon_points);
    return poly_list;
}
