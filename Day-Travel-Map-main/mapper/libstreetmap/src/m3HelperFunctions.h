#ifndef m3Helper_h
#define m3Helper_h
#include "m3.h"
#include <iostream>
#include <vector>
#include <map>
#include <utility>
#include "globals.h"

/*  This header file is used for writing the helper functions to be used in m1/m2.cpp. The descriptions of each helper functions can be found below */

// Given a beginning position and destination, as well as the turn_penalty associated with each change of street, this function
// returns a boolean value that shows if there is a path, and storing the relevant information of each intersecti
bool bfsPath (int srcID, int destID, const double turn_penalty);

// After the intersection information is set by bfs path, it will start from the destination intersection and trace back all the path that
// leads to each intermediate intersections, and storing the StreetSegmentIdx in a std::vector and return as output
std::vector<StreetSegmentIdx> bfsTraceBack (int destID, std::vector<Node>& nodes);
std::vector<StreetSegmentIdx> bfsTraceBack2 (int destID);
// This function is used to compute the heuristic that will be used in min heap to enable best first search.
// It returns the approximate distance between two given intersections.
double distanceHeuristicFunc(const int& currID, const int& destID);

// Compute the time it takes (cumulatively) from the source node to the current node, takes in the ID of previous street and current street, as well
// as current intersection to determine whether it should increment by the street segment travel time only or with both street seg travel time and turn penalty
double getNodeTravelTime(const int& prevStrId, const int& currStrId, const int& currNodeID, const int& turn_penalty);


// Print travel directions
void printTravelDirections(ezgl::application* application);
// Draw path between two intersections
void drawPath(ezgl::renderer *g, const std::vector<StreetSegmentIdx>& path, const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids);
// Draw the arrows on the path between intersections 
std::pair<ezgl::point2d , ezgl::point2d> drawArrowOpposite(ezgl::point2d begin, double angle);
std::pair<ezgl::point2d , ezgl::point2d> drawArrow(ezgl::point2d begin, double angle);

// For m4
// void loadLandmarks(int numLandmarksPerRow);
// bool pointWithinBound(ezgl::point2d point, double boundX1, double boundX2, double boundY1, double boundY2);
// double distanceHeuristicFunc2(const int& currID, const int& destID);
std::pair<std::vector<double>, std::vector<std::vector<StreetSegmentIdx>>> multiTargetBfsPath(const double turn_penalty, const IntersectionIdx beginPos, const std::vector<IntersectionIdx>& interestedIntersections);
std::pair<double,  std::vector<std::vector<StreetSegmentIdx>>> getBestTimeAndPathForDepot(IntersectionIdx depotPos, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections);

#endif
