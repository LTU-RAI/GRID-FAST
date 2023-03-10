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

    PolygonHandler::getPathways(openingList);
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
            //ROS_INFO("----------WARNING--------");
            openingList->disable(w.connectedtOpeningStart[0],26);
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
    int pSize=PolygonHandler::polygonList.size();
    for(int index=0;index<pSize;index++){
        if(PolygonHandler::polygonList[index]->path) continue;
        for(int oIndex=0;oIndex<PolygonHandler::polygonList[index]->openings.size();oIndex++){
            openingDetection* op=PolygonHandler::polygonList[index]->openings[oIndex];
            if(op->parentCoridor!=NULL) continue;
            PolygonHandler::creatPathway(openingList,op);
        }
    }
}

void PolygonHandler::creatPathway(OpeningHandler* openingList,openingDetection* targetOp){
    polygon newPoly;
    newPoly.path=true;
    newPoly.openings.push_back(targetOp);
    vector<wallCell*> wList;
    wallCell w=openingList->getNextOpening(targetOp,false,1,true,true,false,&wList);

    if(w.connectedtOpeningStart[0]!=targetOp){
        newPoly.openings.push_back(w.connectedtOpeningStart[0]);
        newPoly.label=64;
    }else{
        bool test=false;
        for(int i=0;i<wList.size();i++){
            if(!wList[i]->emptyNeighbour) continue;
            test=true;
            break;
        }

        if(test){
            newPoly.label=52;
        }else{
            newPoly.label=41;
        }
    }
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
        vector<point_int> pointListOpening=fillPoints(poly->openings[sideIndex]->end(),
                                                      poly->openings[sideIndex]->start(),0.8,true);
        poly->polygon_points.insert(poly->polygon_points.end(),pointListOpening.begin(),pointListOpening.end());
        vector<point_int> pointList = openingList->getPointsBetweenOpenings(
                poly->openings[sideIndex],!poly->path,
                poly->openings[(sideIndex+1)%openingsSize],poly->path);
        poly->polygon_points.insert(poly->polygon_points.end(),pointList.begin(),pointList.end());
        if(pointList.size()==0) continue;
        for(int i=0;i<pointList.size()-1;i+=polygonRez){
            poly->polygon_points_desplay.push_back(pointList[i]);
        }
        poly->polygon_points_desplay.push_back(pointList.back());
    }
    for(int i=0;i<poly->polygon_points.size();i++){
        if(poly->polygon_points[i]!=poly->polygon_points[(i+1)%poly->polygon_points.size()]) continue;
        poly->polygon_points.erase(poly->polygon_points.begin()+i);
        i--;
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

void PolygonHandler::generateRobotPath(OpeningHandler* openingList, MapHandler* map, MapHandler* mapD){
    for(int pIndex=0;pIndex<PolygonHandler::polygonList.size();pIndex++){
        PolygonHandler::getPathForPolygon(PolygonHandler::polygonList[pIndex],openingList,map,mapD);
    }
}

void PolygonHandler::getPathForPolygon(polygon* poly,OpeningHandler* openingList, MapHandler* map, MapHandler* mapD){
    for(int sideIndex=0; sideIndex<poly->openings.size();sideIndex++){
        int label=poly->label;
        openingDetection* targetOp=poly->openings[sideIndex];
        point_int center=poly->center;
        point_int oCenter1=targetOp->getCenter();
        point_int oCenter2=center;
        if(poly->path && poly->openings.size()==2){
            oCenter2=poly->openings.back()->getCenter();
        }
        vector<point_int> np;
        if((!poly->path)&&!PolygonHandler::checkIfObstructed(center,oCenter1,map)){
            np.push_back(oCenter1);
            np.push_back(center);
        }else if(poly->path && poly->openings.size()==2 && !checkIfObstructed(oCenter1,oCenter2,map)){
            np.push_back(oCenter1);
            np.push_back(oCenter2);
        }else{
            vector<point_int> vp;
            if(!poly->path){
                vp=PolygonHandler::generateVoronoi(poly,map,mapD,oCenter1,center);
            }else if(poly->path && poly->openings.size()==1){
                vp=PolygonHandler::generateVoronoi(poly,map,mapD,oCenter1);
            }else{
                vp=PolygonHandler::generateVoronoi(poly,map,mapD,oCenter1,oCenter2);
            }
            bool toC=false;
            for(int n=0;n<vp.size();n+=voronoiRez){
                np.push_back(vp[n]);
                if((!poly->path) && !PolygonHandler::checkIfObstructed(vp[n],center,map)){
                    toC=true;
                    np.push_back(center);
                    break;
                }
            }
            if(!toC) np.push_back(vp[vp.size()-1]);
        }
        poly->pathList.push_back(np);
        if(poly->path) continue;
    }
}

bool PolygonHandler::checkIfObstructed(point_int p1, point_int p2, MapHandler* map){
    double lenght=dist(p1,p2);
    point normal={(p2.y-p1.y)/(lenght)*minGroupSize/2,-(p2.x-p1.x)/(lenght)*minGroupSize/2};
    point_int tp1=p1, tp2=p2;
    if(map->checkForWallRay(tp1,tp2)>0) return true;
    tp1.x=std::round(p1.x+normal.x);
    tp1.y=std::round(p1.y+normal.y);
    tp2.x=std::round(p2.x+normal.x);
    tp2.y=std::round(p2.y+normal.y);
    if(map->checkForWallRay(tp1,tp2)>0) return true;
    tp1.x=std::round(p1.x-normal.x);
    tp1.y=std::round(p1.y-normal.y);
    tp2.x=std::round(p2.x-normal.x);
    tp2.y=std::round(p2.y-normal.y);
    if(map->checkForWallRay(tp1,tp2)>0) return true;

    return false;
}

robotPath PolygonHandler::generateVoronoi(polygon* poly,MapHandler* map, MapHandler* mapD, point_int start, point_int end){
    vector<point_int> PL=poly->polygon_points;
    vector<point_int> filledPoints;
    if(poly->fillPoints.size()==0){
        filledPoints=fillPoly(PL);
        filledPoints.insert(filledPoints.end(),PL.begin(),PL.end());
        poly->fillPoints=filledPoints;
    }else{
        filledPoints=poly->fillPoints;
    }
    point_int maxP={-1,-1},minP={-1,-1};
    /*for(int n=0;n<filledPoints.size();n++){
        mapD->setMap(filledPoints[n].x,filledPoints[n].y,100);
    }*/
    for(int n=0;n<PL.size();n++){
        //mapD->setMap(PL[n].x,PL[n].y,100);
        if(PL[n].x>maxP.x) maxP.x=PL[n].x;
        if(PL[n].y>maxP.y) maxP.y=PL[n].y;
        if(PL[n].x<minP.x || minP.x==-1) minP.x=PL[n].x;
        if(PL[n].y<minP.y || minP.y==-1) minP.y=PL[n].y;
    }
    //maxP={maxP.x+1,maxP.y+1};
    //minP={minP.x-1,minP.y-1};
    if(start.x>maxP.x) maxP.x=start.x;
    if(start.y>maxP.y) maxP.y=start.y;
    if(start.x<minP.x || minP.x==-1) minP.x=start.x;
    if(start.y<minP.y || minP.y==-1) minP.y=start.y;
    if(end.x!=-1){
        if(end.x>maxP.x) maxP.x=end.x;
        if(end.y>maxP.y) maxP.y=end.y;
        if(end.x<minP.x || minP.x==-1) minP.x=end.x;
        if(end.y<minP.y || minP.y==-1) minP.y=end.y;
    }
    vector<vector<bool>> thinMap;
    thinMap.resize(maxP.x-minP.x+3);
    for(int i1=0;i1<thinMap.size();i1++){
        thinMap[i1].resize(maxP.y-minP.y+3);
        for(int i2=0;i2<thinMap[i1].size();i2++){
            thinMap[i1][i2]=false;
        }
    }
    for(int i=0;i<filledPoints.size();i++) 
        thinMap[filledPoints[i].x-minP.x+1][filledPoints[i].y-minP.y+1]=true;
    start={start.x-minP.x+1,start.y-minP.y+1};
    thinMap[start.x][start.y]=true;
    if(end.x!=-1){
        end={end.x-minP.x+1,end.y-minP.y+1};
        thinMap[end.x][end.y]=true;
    } 
    point_int P[]={{0,0},{-1,0},{-1,1},{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0}};
    bool check=false;
    while(!check){
        for(int sids=0;sids<2;sids++){
            vector<point_int> M;
            for(int i=1;i<thinMap.size()-1;i++){
                for(int j=1;j<thinMap[i].size()+1;j++){
                    point_int rp={i,j};
                    if(!thinMap[i][j] ||
                        rp==start || rp==end)continue;
                    int B=0;
                    for(int pIndex=1; pIndex<9;pIndex++){
                        if(thinMap[i+P[pIndex].x][j+P[pIndex].y]) B+=1;
                    }
                    if(B<2 || B>6) continue;

                    int A=0;
                    for(int pIndex=1; pIndex<9;pIndex++){
                        if(!thinMap[i+P[pIndex].x][j+P[pIndex].y] &&
                            thinMap[i+P[pIndex+1].x][j+P[pIndex+1].y]) A+=1;
                    }
                    if(A!=1) continue;
                    int c[]={1,3,5};
                    int d[]={3,4,7};
                    if(sids==1){
                        c[0]=1; c[1]=3; c[2]=7;
                        d[0]=1; d[1]=5; d[2]=7;
                    }
                    bool t=true;
                    for(int cIndex=0;cIndex<3;cIndex++){
                        if(!thinMap[i+P[c[cIndex]].x][j+P[c[cIndex]].y]){
                            t=false;
                            break;
                        }
                    }
                    if(t) continue;

                    t=true;
                    for(int dIndex=0;dIndex<3;dIndex++){
                        if(!thinMap[i+P[d[dIndex]].x][j+P[d[dIndex]].y]){
                            t=false;
                            break;
                        }
                    }
                    if(t) continue;

                    M.push_back(rp);
                }
            }
            if(M.size()==0){
                check=true;
                break;
            }else{
                for(int i=0;i<M.size();i++){
                    thinMap[M[i].x][M[i].y]=false;
                }
            }
        }
    }
    /*for(int x=0;x<thinMap.size();x++){
        for(int y=0;y<thinMap[x].size();y++){
            if(!thinMap[x][y]) continue;
            mapD->setMap(x+minP.x-1,y+minP.y-1,1);
        }
    }*/
    //Get voronoi path
    point_int P2[]={{-1,0},{0,1},{1,0},{0,-1},{-1,1},{1,1},{1,-1},{-1,-1}};
    vector<robotPath> paths;
    paths.resize(1);
    int cPathIndex=0;
    point_int curentPoint=start;
    point_int newPoint=curentPoint;
    paths[cPathIndex].push_back(curentPoint);
    check=false;
    while (!check){
        thinMap[curentPoint.x][curentPoint.y]=false;
        int c=0;
        bool flip=true;
        for(int pIndex=0;pIndex<8;pIndex++){
            if(curentPoint.x+P2[pIndex].x==end.x && curentPoint.y+P2[pIndex].y==end.y){
                newPoint={curentPoint.x+P2[pIndex].x,curentPoint.y+P2[pIndex].y};
                paths[cPathIndex].push_back(newPoint);
                check=true;
                break;
            }
            if(map->getMap(curentPoint.x+P2[pIndex].x+minP.x-1,curentPoint.y+P2[pIndex].y+minP.y-1)==100) continue;
            if(curentPoint.x+P2[pIndex].x<thinMap.size()&&curentPoint.x+P2[pIndex].x>=0&&
               curentPoint.y+P2[pIndex].y<thinMap[0].size()&&curentPoint.y+P2[pIndex].y>=0&& 
               thinMap[curentPoint.x+P2[pIndex].x][curentPoint.y+P2[pIndex].y]){
                if(pIndex>=4){
                    if(map->getMap(curentPoint.x+P2[(pIndex+4)%8].x+minP.x-1,curentPoint.y+P2[(pIndex+4)%8].y+minP.y-1)==100 &&
                        map->getMap(curentPoint.x+P2[(pIndex+5)%8].x+minP.x-1,curentPoint.y+P2[(pIndex+5)%8].y+minP.y-1)==100) continue;
                }
                c+=1;
                if(flip) newPoint={curentPoint.x+P2[pIndex].x,curentPoint.y+P2[pIndex].y};
                flip=false;
            }
        }
        if(check) break;
        if(c==0){
            if(cPathIndex-1<0){
                int maxS=-1;
                for (int p=0; p<paths.size(); p++){
                    if(int(paths[p].size())>maxS){
                        maxS=paths[p].size();
                        cPathIndex=p;
                    }
                }
                check=true;
                break;
                
            }else{
                cPathIndex-=1;
                curentPoint=paths[cPathIndex].back();
                continue;
            }
        }else if(c==1){
            paths[cPathIndex].push_back(newPoint);
        }else{
            paths.push_back(paths[cPathIndex]);
            cPathIndex=paths.size()-1;
            paths[cPathIndex].push_back(newPoint);
        }
        curentPoint=newPoint;
    }
    for(int i=0; i<paths[cPathIndex].size();i++){
        paths[cPathIndex][i].x+=minP.x-1;
        paths[cPathIndex][i].y+=minP.y-1;
    }
    return paths[cPathIndex];
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
        if(polyToAdd->path){
            polyToAdd->openings[i]->parentCoridor=polyToAdd;
        }else{
            polyToAdd->openings[i]->parent=polyToAdd;
        }
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
    ROS_INFO("o0");
    vector<vector<wallCell*>> walls;
    vector<int> startIndex;
    walls.resize(poly->openings.size());
    bool del=false;

    ROS_INFO("o1");
    for(int sideIndex=0;sideIndex<poly->openings.size();sideIndex++){
        ROS_INFO("t1");
        openingDetection* targetOp1=poly->openings[sideIndex];
        openingDetection* targetOp2=poly->openings[(sideIndex+1)%poly->openings.size()];
        vector<wallCell*> currentWall;
        vector<wallCell*> o1back;
        ROS_INFO("t2");
        wallCell w=openingList->getNextOpening(targetOp1,true,2,false,true,true,&o1back);
        int loopfrom=o1back.size()-1;
        if(w.connectedtOpeningEnd.size()!=0){
            if(w.connectedtOpeningEnd[0]==targetOp1) loopfrom=loopfrom/2;
        }
        ROS_INFO("t3");
        for(int i=loopfrom;i>=0;i--){
            currentWall.push_back(o1back[i]);
        }
        ROS_INFO("t3.5");
        vector<wallCell*> o1ToO2;
        openingList->getPointsBetweenOpenings(targetOp1,true,targetOp2,false,&o1ToO2);
        ROS_INFO("%i, %i",targetOp1->getConnection(true)->index,targetOp2->getConnection(false)->index);
        if(o1ToO2.size()==1){
            PolygonHandler::removeSideFromPolygon(poly,targetOp1,openingList);
            PolygonHandler::removeSideFromPolygon(poly,targetOp2,openingList); 
            return;
        }
        currentWall.insert(currentWall.end(),o1ToO2.begin()+1,o1ToO2.end()-1);
        ROS_INFO("t4");
        vector<wallCell*> o2front;
        w=openingList->getNextOpening(targetOp2,false,1,true,true,true,&o2front);
        int loopto=o2front.size();
        if(w.connectedtOpeningStart.size()!=0){
            if(w.connectedtOpeningStart[0]==targetOp2) loopto=loopto/2;
        }
        ROS_INFO("t5");
        if(o2front.size()!=0) currentWall.insert(currentWall.end(),o2front.begin(),o2front.begin()+loopto);
        walls[sideIndex]=currentWall;
        ROS_INFO("t6");
    }
    ROS_INFO("o2");
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
    ROS_INFO("o3");
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
                if(decrisCount>=6){
                    best=hbest;
                    changed=true;
                    decrisCount=0;
                }else decrisCount+=1;
            }
            double score=lenght+DFunction(dist(test.get_center(),centerP)); //dist(test.get_center(),poly_list[pIndex].center);
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
    ROS_INFO("o4");
}
double PolygonHandler::DFunction(double length){
    double minDistToCenter=4;
    double maxPenalty=1000;
    if(length>minDistToCenter) return dw*(length-minDistToCenter);
    
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
