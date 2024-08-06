

#include "m3.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "cmath"
#include <sstream>
#include <globals.h>
#include "m3HelperFunctions.h"
#include <list>
#include "m2HelperFunctions.h"
#include "m1HelperFunctions.h"
#include "m1.h"




// Global Variables
extern GlobalVars mapData;




double computePathTravelTime(const double turn_penalty, const std::vector<StreetSegmentIdx>& path) {
    
    // Initialize variables
    int numStreetSegments = path.size();
    double totalTravelTime = 0;

    // Guard against empty input path
    if (path.empty()) {
        // If there is no path, path travel time is 0
        return 0;
    }

    int prevStreetId;

    // Loop through all the street segments in the given path
    for (int segId = 0; segId < numStreetSegments; segId++) {
        // Get the current street segment and street IDs
        int currStreetSegId = path[segId];
        int currStreetId = mapData.street_segments[currStreetSegId].streetId;

        
        if (segId == 0) {
            // If it is the first street segment in the path, then we only increment the street segment travel time
            totalTravelTime += findStreetSegmentTravelTime(currStreetSegId);
            // Store the information of previous street id for detecting street id change
            prevStreetId = currStreetId;
            
        } else {


            // If there is a change in street id, increment turn penalty
            if (currStreetId != prevStreetId) {
                totalTravelTime += turn_penalty;
                // Update previous street IDW
                prevStreetId = currStreetId;
            }
            
            // Add street segment travel time regardless  
            totalTravelTime += findStreetSegmentTravelTime(currStreetSegId);

        }
    }

    return totalTravelTime;
}

// For m3
std::vector<StreetSegmentIdx> findPathBetweenIntersections(const double turn_penalty, const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids) {

    // Allocate space to a list of nodes
    mapData.nodes.resize(getNumIntersections());

    // See if a path is found
    bool found = bfsPath(intersect_ids.first, intersect_ids.second, turn_penalty);

    // if found a path, get the path by tracing back the destination IDs
    if (found) {
        std::vector<StreetSegmentIdx> path = bfsTraceBack2(intersect_ids.second);
        // Clear gloabl variable that stores the node information
        mapData.nodes.clear();
        return path;
    }

    // If no path found, return empty vector
    return {};

}

//function to draw an arrow during navigation
std::pair<ezgl::point2d , ezgl::point2d> drawArrow(ezgl::point2d begin, double angle){

    //determine the length of arrow
    double d = 10;
    //variable to convert to radians
    double toRadians = M_PI / 180;
    //convert the angle to radians
    double angleRadians = angle * toRadians;
    double angleLeft= angleRadians + 20 * toRadians;
    double angleRight = angleRadians - 20 * toRadians;
    ezgl::point2d pointLeft = {begin.x - d * cos(angleLeft), begin.y - d*sin(angleLeft)};
    ezgl::point2d pointRight = {begin.x - d * cos(angleRight), begin.y - d*sin(angleRight)};

   

    return std::make_pair(pointLeft, pointRight);
}

//function to draw an opposite arrow during navigation
std::pair<ezgl::point2d , ezgl::point2d> drawArrowOpposite(ezgl::point2d begin, double angle){
    const double d = 10;
    double toRadians = M_PI / 180;
    // if (angle < 90 && angle > 0) {
    //     angle = -angle;
    // } else if (angle < 0 && angle >= -90) {
    //     angle = -angle;
    // } else if (angle >= 90 && angle < 180) {
    //     angle = angle - 180;
    // } else if (angle < -90 && angle > -180) {
    //     angle = 180 + angle;
    // }
    std::cout << "initial " <<angle << std::endl;
    if (angle >= 0) {
        angle -= 180;
        if (angle < -180) {
            angle += 360;
        }
    } else {
        angle += 180;
        if (angle > 180) {
            angle -= 360;
        }
    }
    std::cout << "final " <<angle << std::endl;
    double angleRadians = (angle) * toRadians;
    double angleLeft= angleRadians + 20 * toRadians;
    double angleRight = angleRadians - 20 * toRadians;
    ezgl::point2d pointLeft = {begin.x - d * cos(angleLeft), begin.y - d*sin(angleLeft)};
    ezgl::point2d pointRight = {begin.x - d * cos(angleRight), begin.y - d*sin(angleRight)};
    return std::make_pair(pointLeft, pointRight);
}

//function to draw the path between two intersection
void drawPath(ezgl::renderer *g, const std::vector<StreetSegmentIdx>& path, const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids){
    //return if there is no path needed to be drawn
    if(path.size() == 0){
        return;
    }
    //get the current zoom level
    double currentZoomLevel = getZoomLevel(g);
    //set the font
    g->format_font("Noto Sans CJK SC", ezgl::font_slant::normal, ezgl::font_weight::normal);
    //determine the colour of the path
    //determine if the icon indicating the start of path is on the from or two of the 
    g->draw_surface(mapData.poiSurfaceStart, mapData.intersections[intersect_ids.first].xy_loc, pow(currentZoomLevel, 0.3) * 0.0165);
    g->draw_surface(mapData.poiSurfaceEnd, mapData.intersections[intersect_ids.second].xy_loc, pow(currentZoomLevel, 0.3) * 0.0165);
    
    
    g->set_line_width(4);

    //for loop to loop through all the streetsegments
    for(int i = 0; i < path.size(); i++){
        int Id = path[i];
        //if statements to draw the arrows pointing to the correct directions
        if(i < path.size()-1){
            //set the color
            g->set_color(ezgl::PURPLE);
            //get the next streetsegment
            int nextId = path[i+1];
            //set drawvalid to false
            bool drawValid = false;
            //check if the angle is in range
            if(mapData.street_segments[nextId].angle[0] >= -360 && mapData.street_segments[nextId].angle[0] <= 360){
                drawValid = true;
            }
            //if the angle is in range
            if(drawValid){
                //check if the two street segments are from -> to , and to ->from
                if(mapData.street_segments[Id].to == mapData.street_segments[nextId].from || mapData.street_segments[Id].from == mapData.street_segments[nextId].from){
                    //create a pair of ezgl point2d to draw the arrows
                    std::cout << mapData.street_segments[nextId].angle[0] << std::endl;
                    std::pair<ezgl::point2d , ezgl::point2d> arrowPositions = drawArrow(mapData.street_segments[nextId].midpoint[0],mapData.street_segments[nextId].angle[0]);
                    //draw the left half of the arrow
                    g->draw_line(mapData.street_segments[nextId].midpoint[0], arrowPositions.first);
                    //draw the right half of the arrow
                    g->draw_line(mapData.street_segments[nextId].midpoint[0], arrowPositions.second);
                //when the two street segments are from - > to, and from -> to
                }else if (mapData.street_segments[Id].to == mapData.street_segments[nextId].to || mapData.street_segments[Id].from == mapData.street_segments[nextId].to){
                    std::pair<ezgl::point2d , ezgl::point2d> arrowPositions = drawArrowOpposite(mapData.street_segments[nextId].midpoint[0], mapData.street_segments[nextId].angle[0]);
                    

                    g->draw_line(mapData.street_segments[nextId].midpoint[0], arrowPositions.first);
                    g->draw_line(mapData.street_segments[nextId].midpoint[0], arrowPositions.second);
                }
            }
            
        }
       
       //set the color to blue
        g->set_color(ezgl::BLUE);
        //go through all the curve points for a given street segment
       
    }

    g->set_line_dash(ezgl::line_dash::asymmetric_5_3);
    for(int i = 0; i < path.size(); i++){
        int Id = path[i];
         for (int pointId = 0; pointId < mapData.street_segments[Id].all_curve_points.size() - 1; pointId ++){
            //draw a blue line
            g->draw_line(mapData.street_segments[Id].all_curve_points[pointId], mapData.street_segments[Id].all_curve_points[pointId+1]);
            
        }
    }
    g->set_line_dash(ezgl::line_dash::none);
}
    
void printTravelDirections(ezgl::application* application){
    // Retrieve and cast GObject pointers to GtkLabel for displaying selected intersections
    GObject* label1 = application->get_object("selecetedIntersection1");
    GObject* label2 = application->get_object("selecetedIntersection2");
    
    GtkLabel* intersectionLabel1 = GTK_LABEL(label1);
    GtkLabel* intersectionLabel2 = GTK_LABEL(label2);

    // Prepare and set the text for the starting point label
    std::string nameOfChosenInter1 = "Starting Point:" +getIntersectionName(mapData.intersect_ids.first);
    gtk_label_set_text(intersectionLabel1, nameOfChosenInter1.c_str());
   
    // Prepare and set the text for the destination point label
    std::string nameOfChosenInter2 = "Destination Point:" + getIntersectionName(mapData.intersect_ids.second);
    gtk_label_set_text(intersectionLabel2, nameOfChosenInter2.c_str());

    // Show the directions window    
    GObject* DirectionsWindow = application->get_object("DirectionsWindow");
    GtkWidget* displayDirectionWindow = GTK_WIDGET(DirectionsWindow);
    gtk_widget_show_all(displayDirectionWindow);

    // Get the label object where directions will be displayed
    GObject* DirectionTexts = application->get_object("DirectionTexts");
    GtkLabel* labelDirection = GTK_LABEL(DirectionTexts);

    // Initialize a string stream to accumulate the directions
    std::stringstream directionMsg;

    // Initialize a string stream to accumulate the directions
    for(size_t i = 0; i < mapData.path.size(); i++){
        // For the first segment, indicate starting street
        Street_segment_data currentStreet = mapData.street_segments[mapData.path[i]];
        if(i == 0){
            directionMsg << "Start from " << currentStreet.streetName << "\n";
        }else if(i == mapData.path.size()-1){
            // For the last segment, indicate arrival
            directionMsg << "Take " << currentStreet.streetName << " and you have arrived at your destination\n"; 
        }else{
            // For intermediate segments, provide turning instructions
            Street_segment_data prevStreet = mapData.street_segments[mapData.path[i-1]];
            double angle = findAngleBetweenStreetSegments(mapData.path[i-1], mapData.path[i]);
            // Check if the street name changes to indicate a turn
            if(prevStreet.streetName != currentStreet.streetName){
                // Determine turn direction based on the angle
                if(angle > 0 && angle < M_PI){ 
                    directionMsg << "Turn left to " << currentStreet.streetName << "\n";
                }else{
                    directionMsg << "Turn right to " << currentStreet.streetName << "\n";
                }
            }
        }
    }

    // Update the label text with the directions
    gtk_label_set_text(labelDirection, directionMsg.str().c_str());
}



// bool bfsPath (int srcID, int destID, const double turn_penalty) {

//     // Create a min heap that stores the element that have the smallest heuristics on top    
//     std::priority_queue<WaveElem, std::vector<WaveElem>, WaveElem::Compare> wavefront;

//     // Get the distance from source to destination and store it as the first element in the min heap
//     double distSrcToDest = distanceHeuristicFunc(srcID, destID);
//     wavefront.emplace(srcID, NO_EDGE, 0, distSrcToDest);

//     // When the wavefront is not empty, that is, there are still paths and intersections left to search before reaching the destination
//     while (!wavefront.empty()) {
//         // Take out the first element and pop() to remove the element and adjust heap
//         WaveElem curr = wavefront.top();
//         wavefront.pop();

//         // Fetch the current intersection and street segment IDs stored to the Wave element
//         IntersectionIdx currID = curr.interId;
//         StreetSegmentIdx segID = curr.segId;

//         // Since travel time is the top priority, we only consider time to ensure optimality tests are passed
//         // We only consider the current element if its travel time is better than the best travel time found to the node
//         if (curr.travelTime < mapData.nodes[currID].bestTime) {
//             // Update all node information since this is a better path to the node
//             // These two properties will be used for bfstraceback, so we store it before we check if we find the destination
//             mapData.nodes[currID].reachingEdge = segID;
//             mapData.nodes[currID].bestTime = curr.travelTime;

//             // If the current WaveElement stores the ID of the destination, it indicates that we have found the destination
//             if (currID == destID) {
//                 return true;
//             }

//             // If destination is not found
//             // For each outgoing street of the current intersection
//             // Get the current intersection id -- and then find all of its opposite ends]
//             // Can also call find street segment of intersections, but we access the global vars directly in hope of faster speed
//             std::vector<StreetSegmentIdx> currStreetSegsOfIntersection = mapData.intersection_street_segments[currID];
            
//             // Loop through all the streets that connect to the currenct intersections
//             for (auto it = currStreetSegsOfIntersection.begin(); it != currStreetSegsOfIntersection.end(); ++it) {
//                 int streetSegID = *it;
            
//                 // If intersection position is from, return to, if to, return from      
//                 int toNodeID;
//                 if (mapData.street_segments[streetSegID].from == currID) {
//                     toNodeID = mapData.street_segments[streetSegID].to;

//                     // If it is the first wave element, we do not have any turn penalty
//                     if (segID == NO_EDGE) {
//                         double distToDest = distanceHeuristicFunc(toNodeID, destID);
//                         wavefront.emplace(toNodeID, streetSegID, findStreetSegmentTravelTime(streetSegID), distToDest);
//                         continue;
//                     }

                    
                    
//                     // Apppend the wave element to the wavefront
//                     double distToDest = distanceHeuristicFunc(toNodeID, destID);
//                     double nodeTravelTime = getNodeTravelTime(segID, streetSegID, currID, turn_penalty);
//                     wavefront.emplace(toNodeID, streetSegID, nodeTravelTime, distToDest);
                                        
//                 } else if (mapData.street_segments[streetSegID].to == currID && !mapData.street_segments[streetSegID].oneWay) {
//                     // When the node is starting from the "to" node, we have to respect one way
//                     toNodeID = mapData.street_segments[streetSegID].from;
                    
//                     // See if it is the first element, if yes then we only add the current segment travel time
//                     if (segID == NO_EDGE) {
//                         double distToDest = distanceHeuristicFunc(toNodeID, destID);
//                         wavefront.emplace(toNodeID, streetSegID, findStreetSegmentTravelTime(streetSegID), distToDest);
//                         continue;
//                     }
                    
                    
//                     double nodeTravelTime = getNodeTravelTime(segID, streetSegID, currID, turn_penalty);
//                     double distToDest = distanceHeuristicFunc(toNodeID, destID);
//                     wavefront.emplace(toNodeID, streetSegID, nodeTravelTime, distToDest);

//                 } 

//             }
//         }        
//     }

//     return false;
// }

// Change this to allow multi djikstra
bool bfsPath (int srcID, int destID, const double turn_penalty) {
    // Will have to change to linked list later
    

    // std::list<WaveElem> wavefront;
    std::priority_queue<WaveElem, std::vector<WaveElem>, WaveElem::Compare> wavefront;
    wavefront.emplace(srcID, NO_EDGE, 0);
    // wavefront.push_back(WaveElem(srcID, NO_EDGE, 0));

    while (!wavefront.empty()) {
        WaveElem curr = wavefront.top();
        wavefront.pop();

        IntersectionIdx currID = curr.interId;

        if (curr.travelTime < mapData.nodes[currID].bestTime) {
            // Update if this is a better path to the node
            mapData.nodes[curr.interId].reachingEdge = curr.segId;
            mapData.nodes[curr.interId].bestTime = curr.travelTime;
            if (curr.interId == destID) {
                return true;
            }

            // for each outgoing edge of the current intersection
            // Find all the streets that branches from the intersection

            // Get the current intersection id -- and then find all of its opposite ends
            std::vector<StreetSegmentIdx> currStreetSegsOfIntersection = findStreetSegmentsOfIntersection(currID);

            for (auto it = currStreetSegsOfIntersection.begin(); it != currStreetSegsOfIntersection.end(); ++it) {
                int streetSegID = *it;
                // Can maybe use the global variable entirely??
                StreetSegmentInfo segInfo = getStreetSegmentInfo(streetSegID);
            
                // If intersection position is from, return to, if to, return from      
            
                int toNodeID;
                if (segInfo.from == currID) {
                    toNodeID = segInfo.to;
                    // wavefront.push_back(WaveElem(toNodeID, streetSegID, mapData.nodes[currID].bestTime + findStreetSegmentTravelTime(streetSegID)));

                    // Compare this street segment with previous one to see if turn penalty should be given
                    
                    int prevStreetSegId = curr.segId;
                    int currStreetSegId = streetSegID;

                    if (prevStreetSegId == NO_EDGE) {
                        wavefront.emplace(toNodeID, streetSegID, findStreetSegmentTravelTime(streetSegID));
                        continue;
                    }
                    int prevStreetId = mapData.street_segments[prevStreetSegId].streetId;
                    int currStreetId = mapData.street_segments[currStreetSegId].streetId;
                    double nodeTravelTime = 0;
                    if(prevStreetId != currStreetId) {
                        nodeTravelTime = mapData.nodes[currID].bestTime + findStreetSegmentTravelTime(streetSegID) + turn_penalty;
                    } else {
                        nodeTravelTime = mapData.nodes[currID].bestTime + findStreetSegmentTravelTime(streetSegID);
                    }

                    wavefront.emplace(toNodeID, streetSegID, nodeTravelTime);





                    
                    // When the node is starting from the "to" node, we have to respect one way
                } else if (segInfo.to == currID && !segInfo.oneWay) {
                    toNodeID = segInfo.from;
                    // wavefront.push_back(WaveElem(toNodeID, streetSegID, mapData.nodes[currID].bestTime + findStreetSegmentTravelTime(streetSegID)));

                    // Compare this street segment with previous one to see if turn penalty should be given
                    int prevStreetSegId = curr.segId;
                    int currStreetSegId = streetSegID;
                    if (prevStreetSegId == NO_EDGE) {
                        wavefront.emplace(toNodeID, streetSegID, findStreetSegmentTravelTime(streetSegID));
                        continue;
                    }
                    int prevStreetId = mapData.street_segments[prevStreetSegId].streetId;
                    int currStreetId = mapData.street_segments[currStreetSegId].streetId;
                    
                    double nodeTravelTime = 0;
                    if(prevStreetId != currStreetId) {
                        nodeTravelTime = mapData.nodes[currID].bestTime + findStreetSegmentTravelTime(streetSegID) + turn_penalty;
                    } else {
                        nodeTravelTime = mapData.nodes[currID].bestTime + findStreetSegmentTravelTime(streetSegID);
                    }

                    wavefront.emplace(toNodeID, streetSegID, nodeTravelTime);

                } 

            }
        }

        
        
        
        
    }

    return false;
}

double getNodeTravelTime(const int& prevStrId, const int& currStrId, const int& currNodeID, const int& turn_penalty) {

    int prevStreetId = mapData.street_segments[prevStrId].streetId;
    int currStreetId = mapData.street_segments[currStrId].streetId;
    double nodeTravelTime;
    // Compare this street segment with previous one to see if turn penalty should be given    
    if(prevStreetId != currStreetId) {
        nodeTravelTime = mapData.nodes[currNodeID].bestTime + findStreetSegmentTravelTime(currStrId) + turn_penalty;
    } else {
        nodeTravelTime = mapData.nodes[currNodeID].bestTime + findStreetSegmentTravelTime(currStrId);
    }
    return nodeTravelTime;
}

std::vector<StreetSegmentIdx> bfsTraceBack (int destID, std::vector<Node>& nodes) {
    std::list<StreetSegmentIdx> path;
    int currNodeID = destID;
    int prevEdge = nodes[currNodeID].reachingEdge;

    while(prevEdge != NO_EDGE) {
        path.push_front(prevEdge);
        
        // Node at the other end of prev edge

        if (mapData.street_segments[prevEdge].from == currNodeID) {
            currNodeID = mapData.street_segments[prevEdge].to;
        } else if (mapData.street_segments[prevEdge].to == currNodeID) {
            currNodeID = mapData.street_segments[prevEdge].from;
        }

        prevEdge = nodes[currNodeID].reachingEdge;
    }

    // Convert everything to vector, can be added as a helper function
    std::vector<StreetSegmentIdx> pathVec {std::begin(path), std::end(path)};

    return pathVec;



}

std::vector<StreetSegmentIdx> bfsTraceBack2 (int destID) {
    std::list<StreetSegmentIdx> path;
    int currNodeID = destID;
    int prevEdge = mapData.nodes[currNodeID].reachingEdge;

    while(prevEdge != NO_EDGE) {
        path.push_front(prevEdge);
        
        // Node at the other end of prev edge

        if (mapData.street_segments[prevEdge].from == currNodeID) {
            currNodeID = mapData.street_segments[prevEdge].to;
        } else if (mapData.street_segments[prevEdge].to == currNodeID) {
            currNodeID = mapData.street_segments[prevEdge].from;
        }

        prevEdge = mapData.nodes[currNodeID].reachingEdge;
    }

    // Convert everything to vector, can be added as a helper function
    std::vector<StreetSegmentIdx> pathVec {std::begin(path), std::end(path)};

    return pathVec;



}


double distanceHeuristicFunc(const int& currID, const int& destID) {

    // Euclidean Distance
    ezgl::point2d currNode = mapData.intersections[currID].xy_loc;
    ezgl::point2d prevNode = mapData.intersections[destID].xy_loc;
    double currNodeX = currNode.x;
    double currNodeY = currNode.y;
    double prevNodeX = prevNode.x;
    double prevNodeY = prevNode.y;

    double dxSquared = pow(currNodeX - prevNodeX, 2);
    double dySquared = pow(currNodeY - prevNodeY, 2);
    double distance = sqrt(dxSquared + dySquared);

    // For m4
    // double distance = 0;
    // // std::cout << mapData.landmarks.size() << std::endl;
    // for (int i = 0; i < mapData.landmarks.size(); ++i) {

    //     double distToSrc = mapData.landmarkDist[i][currID];
    //     double distToDest = mapData.landmarkDist[i][destID];

    //     double difference = abs(distToSrc - distToDest);

    //     if (difference > distance) {
    //         distance = difference;
    //     }

    // }

    return distance;
}

// For M4
// double distanceHeuristicFunc2(const int& currID, const int& destID) {

//     // Euclidean Distance
//     ezgl::point2d currNode = mapData.intersections[currID].xy_loc;
//     ezgl::point2d prevNode = mapData.intersections[destID].xy_loc;
//     double currNodeX = currNode.x;
//     double currNodeY = currNode.y;
//     double prevNodeX = prevNode.x;
//     double prevNodeY = prevNode.y;

//     double dxSquared = pow(currNodeX - prevNodeX, 2);
//     double dySquared = pow(currNodeY - prevNodeY, 2);
//     double distance = sqrt(dxSquared + dySquared);


//     return distance;
// }


// void loadLandmarks(int numLandmarksPerRow) {
//     // Using grid base search

//     // Choose 25 Landmarks

//     double maxLat = mapData.max_lat;
//     double minLat = mapData.min_lat;
//     double maxLon = mapData.max_lon;
//     double minLon = mapData.min_lon;


//     // Convert them into xy_coords
//     double mapBoundx1 = x_from_lon(minLon);
//     double mapBoundx2 = x_from_lon(maxLon);

//     double mapBoundy1 = y_from_lat(minLat);
//     double mapBoundy2 = y_from_lat(maxLat);
    
//     double dx = mapBoundx1 - mapBoundx2;
//     double dy = mapBoundy1 - mapBoundy2;

//     double xinterval = dx/numLandmarksPerRow;
//     double yinterval = dy/numLandmarksPerRow;

//     // mapData.landmarks.resize(numLandmarksPerRow*numLandmarksPerRow);

//     double prevX = mapBoundx1;
//     double prevY = mapBoundy1;
//     double currentX = mapBoundx1 - xinterval;
//     double currentY = mapBoundy1 - yinterval;



//     for (int i = 0; i < numLandmarksPerRow; ++i) {
//         for(int j = 0; j < numLandmarksPerRow; ++j) {
//             for (int interID = 0; interID < getNumIntersections(); ++interID) {
//                 // double x_loc = mapData.intersections[interID].xy_loc.x;
//                 // double y_loc = mapData.intersections[interID].xy_loc.y;
//                 if (pointWithinBound(mapData.intersections[interID].xy_loc, prevX - i*xinterval, currentX - i*xinterval, prevY - j*yinterval, currentY - j*yinterval)) {
//                     mapData.landmarks.push_back(interID);
//                     break;
//                 }

//             }
//         }
    
//     }

//     // Loop through landmark id and load the distance for all of the intersections
//     mapData.landmarkDist.resize(mapData.landmarks.size());
//     for (int landmarkNum = 0; landmarkNum < mapData.landmarks.size(); ++landmarkNum) {
//         mapData.landmarkDist[landmarkNum].resize(getNumIntersections());
//         for (int idx = 0; idx < getNumIntersections(); ++idx) {
//             mapData.landmarkDist[landmarkNum][idx] = distanceHeuristicFunc2(mapData.landmarks[landmarkNum], idx);
//         }
//     }

//     std::cout << "Loaded Landmarks1111" << std::endl; 



// }

// bool pointWithinBound(ezgl::point2d point, double boundX1, double boundX2, double boundY1, double boundY2) {

//     double left, right;
//     double top, bottom;
//     if (boundX1 < boundX2) {
//         left = boundX1;
//         right = boundX2;
//     } else {
//         left = boundX2;
//         right = boundX1;
//     }

//     if (boundY1 < boundY2) {
//         bottom = boundY1;
//         top = boundY2;
//     } else {
//         top = boundY2;
//         bottom = boundY1;
//     }


  
//     if(point.x < left || right < point.x || point.y < bottom || top < point.y) {
//       return false;
//     }

//     return true;
  

// }



// Use key intersections as a global variable to keep the indexing consistent？？



std::pair<std::vector<double>, std::vector<std::vector<StreetSegmentIdx>>> multiTargetBfsPath(const double turn_penalty, const IntersectionIdx beginPos, const std::vector<IntersectionIdx>& interestedIntersections) {

    // Removing the element that we are currently searching
    std::list<IntersectionIdx> intersectionsToSearch = {interestedIntersections.begin(), interestedIntersections.end()};
    // Initialize wavefront for searching
    std::priority_queue<WaveElem, std::vector<WaveElem>, WaveElem::Compare> wavefront;
    // Initialize nodes in here to allow parallel cores
    std::vector<Node> nodes;
    // std::cout << "begin pos is " << beginPos << std::endl;
    wavefront.emplace(beginPos, NO_EDGE, 0);
    intersectionsToSearch.remove(beginPos);

    // Initialize output street segs
    // Allocate space for the output paths vector
    int numInterestedIntersections = interestedIntersections.size();
    std::vector<double> outputTime;
    outputTime.resize(numInterestedIntersections);

    std::vector<std::vector<StreetSegmentIdx>> outputPathsFromTheIntersection;
    outputPathsFromTheIntersection.resize(numInterestedIntersections);
    // Allocate space for global vars
    nodes.resize(getNumIntersections());
    while (!wavefront.empty()) {
        WaveElem curr = wavefront.top();
        wavefront.pop();

        IntersectionIdx currID = curr.interId;

        if (curr.travelTime < nodes[currID].bestTime) {
            // Update if this is a better path to the node
            nodes[curr.interId].reachingEdge = curr.segId;
            nodes[curr.interId].bestTime = curr.travelTime;

            // Do a for loop here to find which intersection index it is in the original interestedIntersections
            for (int interestedInterID = 0; interestedInterID < numInterestedIntersections; ++interestedInterID) {
                if (interestedIntersections[interestedInterID] == curr.interId) {
                    // One less item to search
                    intersectionsToSearch.remove(curr.interId);

                    // Update current best time and best path
                    std::vector<StreetSegmentIdx>  currentPath = bfsTraceBack(curr.interId, nodes);
                    outputPathsFromTheIntersection[interestedInterID] = currentPath;

                    // if (!currentPath.empty() && mapData.street_segments[currentPath[0]].from == 49589) {
                    //     std::cout << mapData.street_segments[currentPath[0]].from << "  ";
                    // } else {
                    //     std::cout << "Placeholder" << "  ";
                    // }
                    
                    
                    
                    // std::cout << std::endl << "finished" << std::endl;

                    double currentTime = computePathTravelTime(turn_penalty, currentPath);
                    outputTime[interestedInterID] = currentTime;

                    // std::cout << "current time is: " << currentTime << std::endl;
                    if (intersectionsToSearch.empty()) {
                        // All the intersections are searched and found
                        // std::cout << "All path found" << std::endl;
                        std::pair<std::vector<double>, std::vector<std::vector<StreetSegmentIdx>>> output = std::make_pair(outputTime, outputPathsFromTheIntersection);
                        return output;
                    }

                    break;
                }
            }

            
            // for each outgoing edge of the current intersection
            // Find all the streets that branches from the intersection

            // Get the current intersection id -- and then find all of its opposite ends
            std::vector<StreetSegmentIdx> currStreetSegsOfIntersection = findStreetSegmentsOfIntersection(currID);

            for (auto it = currStreetSegsOfIntersection.begin(); it != currStreetSegsOfIntersection.end(); ++it) {
                int streetSegID = *it;
                // Can maybe use the global variable entirely??
                StreetSegmentInfo segInfo = getStreetSegmentInfo(streetSegID);
            
                // If intersection position is from, return to, if to, return from      
            
                int toNodeID;
                if (segInfo.from == currID) {
                    toNodeID = segInfo.to;
                    // wavefront.push_back(WaveElem(toNodeID, streetSegID, mapData.nodes[currID].bestTime + findStreetSegmentTravelTime(streetSegID)));

                    // Compare this street segment with previous one to see if turn penalty should be given
                    
                    int prevStreetSegId = curr.segId;
                    int currStreetSegId = streetSegID;

                    if (prevStreetSegId == NO_EDGE) {
                        wavefront.emplace(toNodeID, streetSegID, findStreetSegmentTravelTime(streetSegID));
                        continue;
                    }
                    int prevStreetId = mapData.street_segments[prevStreetSegId].streetId;
                    int currStreetId = mapData.street_segments[currStreetSegId].streetId;
                    double nodeTravelTime = 0;
                    if(prevStreetId != currStreetId) {
                        nodeTravelTime = nodes[currID].bestTime + findStreetSegmentTravelTime(streetSegID) + turn_penalty;
                    } else {
                        nodeTravelTime = nodes[currID].bestTime + findStreetSegmentTravelTime(streetSegID);
                    }

                    wavefront.emplace(toNodeID, streetSegID, nodeTravelTime);





                    
                    // When the node is starting from the "to" node, we have to respect one way
                } else if (segInfo.to == currID && !segInfo.oneWay) {
                    toNodeID = segInfo.from;
                    // wavefront.push_back(WaveElem(toNodeID, streetSegID, mapData.nodes[currID].bestTime + findStreetSegmentTravelTime(streetSegID)));

                    // Compare this street segment with previous one to see if turn penalty should be given
                    int prevStreetSegId = curr.segId;
                    int currStreetSegId = streetSegID;
                    if (prevStreetSegId == NO_EDGE) {
                        wavefront.emplace(toNodeID, streetSegID, findStreetSegmentTravelTime(streetSegID));
                        continue;
                    }
                    int prevStreetId = mapData.street_segments[prevStreetSegId].streetId;
                    int currStreetId = mapData.street_segments[currStreetSegId].streetId;
                    
                    double nodeTravelTime = 0;
                    if(prevStreetId != currStreetId) {
                        nodeTravelTime = nodes[currID].bestTime + findStreetSegmentTravelTime(streetSegID) + turn_penalty;
                    } else {
                        nodeTravelTime = nodes[currID].bestTime + findStreetSegmentTravelTime(streetSegID);
                    }

                    wavefront.emplace(toNodeID, streetSegID, nodeTravelTime);

                } 

            }
        }

        
        
        
        
    }


    for (int id = 0; id < interestedIntersections.size(); ++id) {
        // Some intersections failed to find a path
        for (auto it = intersectionsToSearch.begin(); it != intersectionsToSearch.end(); ++it) {
            int invalidIntersection = *it;
            if (interestedIntersections[id] == invalidIntersection) {
                // No path found, append travel time as max int
                outputTime[id] = 2147483647;
                // No path found, append path as empty path
                outputPathsFromTheIntersection[id] = {};
            } else if (interestedIntersections[id] == beginPos) {
                // If going for source to source, time is 0
                outputTime[id] = 0;
                // Path from src to src is empty
                outputPathsFromTheIntersection[id] = {};
            }
        }
            

            
    }
    

    return std::make_pair(outputTime, outputPathsFromTheIntersection);
}



std::pair<double,  std::vector<std::vector<StreetSegmentIdx>>> getBestTimeAndPathForDepot(IntersectionIdx depotPos, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections) {


    // Heuristic - Greedy Algorithms
    int numInterestedInter = interestedIntersections.size();
    std::vector<std::vector<StreetSegmentIdx>> finalPath;
    double finalTime;
    // for (int depotNum = 0; depotNum < depots.size(); ++depotNum) {
    // Search from each depot --- can maybe use parallel for this one?
    IntersectionIdx initPos = depotPos;
    IntersectionIdx currPos = depotPos;
    // Initialize an array of only pickups and dropoffs
    std::vector<IntersectionIdx> pickUpAndDropOffs = pickUps;
    int chosenPos;
    int prevChosen;
    while (!pickUpAndDropOffs.empty()) {

        double bestNextPosTime = 999999999;
        std::vector<StreetSegmentIdx> bestPathForNextPos;
        double secondBestNextPosTime = 10101010;
        std::vector<StreetSegmentIdx> secondBestPathForNextPos;

        // Access all the possible next positions
        for (int availPos = 0; availPos < pickUpAndDropOffs.size(); availPos++) {
            // Accessing the cost from current position to the nearest legal position 
            for (int i = 0; i < numInterestedInter; i++) {
                for (int j = 0; j < numInterestedInter; j++) {
                    // We check which index corresponds to the current position
                    if (interestedIntersections[i] == currPos && interestedIntersections[j] == pickUpAndDropOffs[availPos]) {
                        // Cost from current position to all the legal pick up and drop of positions
                        if (mapData.timeMatrix[i][j] < bestNextPosTime && i != j) {
                            if (bestNextPosTime != 999999999) {
                                // the best time has been populated
                                // we update second best time
                                secondBestNextPosTime = bestNextPosTime;
                                secondBestPathForNextPos = bestPathForNextPos;
                                prevChosen = chosenPos;
                            }
                            // Update best time
                            bestNextPosTime = mapData.timeMatrix[i][j];
                            // To optimize, we can save the index that have the best path and access it at the end
                            bestPathForNextPos = mapData.pathMatrix[i][j];
                            // Save the position we choose (for removal later)
                            chosenPos = pickUpAndDropOffs[availPos];
                        }
                    }

                }

            }

        }

        // Decision based on random number generation
        // If second best solution is populated
        // if (random() % 30 == 9 && secondBestNextPosTime != 10101010) {
            // Take the second best solution, if there is one
            // finalPath.push_back(secondBestPathForNextPos);
            // finalTime += secondBestNextPosTime;
            // chosenPos = prevChosen;
        // } else {
            // Add smaller paths to the final path
            finalPath.push_back(bestPathForNextPos);
            finalTime += bestNextPosTime;
        // }
        
           
        // Update valid positions
        int numRepeatedItems = 0;
        for (int searchNum = 0; searchNum < pickUpAndDropOffs.size(); searchNum++) {
            if (pickUpAndDropOffs[searchNum] == chosenPos) {
                // loop through pick ups and see which position it is in, and access the corresponding dropOffs, if possible
                for (int pickUpId = 0; pickUpId < pickUps.size(); ++pickUpId) {
                    if (pickUps[pickUpId] == chosenPos) {
                        pickUpAndDropOffs.push_back(dropOffs[pickUpId]);
                    }
                }
            }

        }

        // Remove searched pickup/dropoff and update the current position
        pickUpAndDropOffs.erase(std::remove(pickUpAndDropOffs.begin(), pickUpAndDropOffs.end(), chosenPos), pickUpAndDropOffs.end());
        currPos = chosenPos;

    }


    // Add the last segment and its travel time
    std::vector<StreetSegmentIdx> somePath;
    double someTime;
    // Accessing the cost from current position to the nearest legal position 
    for (int i = 0; i < numInterestedInter; i++) {
        for (int j = 0; j < numInterestedInter; j++) {
            // We check which index corresponds to the current position
            if (interestedIntersections[i] == chosenPos && interestedIntersections[j] == initPos) {
                // To optimize, we can save the index that have the best path and access it at the end
                somePath = mapData.pathMatrix[i][j];
                someTime += mapData.timeMatrix[i][j];

            }
        }
    }
    // Need to add from the last intersection back to the depot
    finalPath.push_back(somePath); 
    finalTime += someTime;

    return std::make_pair(finalTime, finalPath);



}