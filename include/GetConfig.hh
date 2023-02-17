#pragma once

#include <fstream>

//Map setings
int mapSizeX=10;
int mapSizeY=10;
float resolution=0;
float mapOffsetX=0;
float mapOffsetY=0;
float mapHight=0;
//scan setings
int minGroupSize=6;
int minCoridorSize=2;
int cfilterSize=1;
int objectFilterMaxStep=40;
double openingRemoveScale=1.4;
int numberOfDir=4;
int extendDevider=2;
double dw=0.8;
int searchLenghtFitcoridor=60;
int searchLenghtFixOverlap=30;
bool removeOpeningsFirst=true;

//ant para
int searchLenghtClean=10;
int searchLenghtOverlap=10;
int sercheLenthAnt=600;
int sercheLenthAntConect=2000;
int sercheLenthAntConectPath=2000;
int maxAntGap=4;
int poligonRez=4;
int poligonRezPath=10;
int voronoiRez=6;
int minimumSercheLenght=5;

//Debugging
bool show_removed_openings=false;


void load_config_file(const std::string &file_path) {
    std::ifstream config_file(file_path);
    std::string line;
    while (std::getline(config_file, line)) {
        // Remove coments
        std::size_t separator_pos = line.find("#");
        if(separator_pos>0)
        line = line.substr(0, separator_pos);
        // Split the line into a key and a value
        separator_pos = line.find("=");
        std::string key = line.substr(0, separator_pos);
        std::string value = line.substr(separator_pos + 1);

        // Remove leading and trailing whitespaces from the key and value
        key.erase(key.find_last_not_of(" \n\r\t")+1);
        key.erase(0, key.find_first_not_of(" \n\r\t"));
        value.erase(value.find_last_not_of(" \n\r\t")+1);
        value.erase(0, value.find_first_not_of(" \n\r\t"));

        // Store the value in the appropriate global variable
        if (key == "minGroupSize") {
            minGroupSize = std::stoi(value);
        } else if (key == "minCoridorSize") {
            minCoridorSize = std::stod(value);
        } else if (key == "cfilterSize") {
            cfilterSize = std::stoi(value);
        }else if (key == "objectFilterMaxStep") {
            objectFilterMaxStep = std::stoi(value);
        }else if (key == "openingRemoveScale") {
            openingRemoveScale = std::stod(value);
        }else if (key == "numberOfDir") {
            numberOfDir = std::stoi(value);
        }else if (key == "extendDevider") {
            extendDevider = std::stoi(value);
        }else if (key == "dw") {
            dw = std::stod(value);
        }else if (key == "searchLenghtFitcoridor") {
            searchLenghtFitcoridor = std::stoi(value);
        }else if (key == "searchLenghtFixOverlap") {
            searchLenghtFixOverlap = std::stoi(value);
        }else if (key == "removeOpeningsFirst") {
            removeOpeningsFirst = value=="true"||value=="True"||value=="1";
        }else if (key == "searchLenghtClean") {
            searchLenghtClean = std::stoi(value);
        }else if (key == "searchLenghtOverlap") {
            searchLenghtOverlap = std::stoi(value);
        }else if (key == "sercheLenthAnt") {
            sercheLenthAnt = std::stoi(value);
        }else if (key == "sercheLenthAntConect") {
            sercheLenthAntConect = std::stoi(value);
        }else if (key == "sercheLenthAntConectPath") {
            sercheLenthAntConectPath = std::stoi(value);
        }else if (key == "maxAntGap") {
            maxAntGap = std::stoi(value);
        }else if (key == "poligonRez") {
            poligonRez = std::stoi(value);
        }else if (key == "poligonRezPath") {
            poligonRezPath = std::stoi(value);
        }else if (key == "voronoiRez") {
            voronoiRez = std::stoi(value);
        }else if (key == "minimumSercheLenght") {
            minimumSercheLenght = std::stoi(value);
        }else if (key == "show_removed_openings") {
            show_removed_openings = value=="true"||value=="True"||value=="1";
        }
    }
}

