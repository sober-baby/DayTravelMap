#ifndef m1Helper_h
#define m1Helper_h
#include "m1.h"
#include <iostream>
#include <vector>
#include <map>
#include <utility>
#include "globals.h"

/*  This class is used for writing the helper functions to be used in m1.cpp. The descriptions of each helper functions can be found below */




// Helper function declarations
std::string str_tolower(std::string s);
void loadStreetLengthsAndTravelTime();
void loadIntersectionStreetSegments();
void populateNodeIdToNodeMap();
void populateWayIdToWayMap();
void populateMapWithNodeTags();
void loadIntersectionofStreets();
void loadMultimapForPartialNames();
void closeMapFinal();
std::pair<std::pair<double, double>, std::pair<double, double>> LatlonToXY (LatLon point1, LatLon point2);






#endif