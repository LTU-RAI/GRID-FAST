#pragma once

#include <fstream>

//Scan Settings
int minGroupSize=6;
int minCoridorSize=2;
int cfilterSize=1;
int objectFilterMaxStep=40;
int numberOfDir=4;
bool forceUpdate=false;

//Opening Settings
double dw=0.8;
int searchLenghtOverlap=10;

//Polygon Settings
int optimizationSteps=0;
double minDistToCenter=0;
double maxPenalty=0;
int polygonRez=4;

//Robot Path Settings
int voronoiRez=6;

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
        }else if (key == "numberOfDir") {
            numberOfDir = std::stoi(value);
        }else if (key == "forceUpdate") {
            forceUpdate = value=="true"||value=="True"||value=="1";
        }else if (key == "dw") {
            dw = std::stod(value);
        }else if (key == "searchLenghtOverlap") {
            searchLenghtOverlap = std::stoi(value);
        }else if (key == "optimizationSteps") {
            optimizationSteps = std::stoi(value);
        }else if (key == "minDistToCenter") {
            minDistToCenter = std::stod(value);
        }else if (key == "maxPenalty") {
            maxPenalty = std::stod(value);
        }else if (key == "polygonRez") {
            polygonRez = std::stoi(value);
        }else if (key == "voronoiRez") {
            voronoiRez = std::stoi(value);
        }else if (key == "show_removed_openings") {
            show_removed_openings = value=="true"||value=="True"||value=="1";
        }
    }
}

