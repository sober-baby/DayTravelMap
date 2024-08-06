/*
 * Copyright 2024 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated
 * documentation files (the "Software") in course work at the University
 * of Toronto, or for personal use. Other uses are prohibited, in
 * particular the distribution of the Software either publicly or to third
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <iostream>

#include "globals.h"
#include "m1HelperFunctions.h"
#include "m2HelperFunctions.h"
#include "m3HelperFunctions.h"

// Initialize Global Variabless
GlobalVars mapData;
#define TIME_MEASURE(function, body)                                         \
  {                                                                          \
    auto start = std::chrono::high_resolution_clock::now();                  \
    body;                                                                    \
    auto stop = std::chrono::high_resolution_clock::now();                   \
    auto duration =                                                          \
        std::chrono::duration_cast<std::chrono::microseconds>(stop - start); \
    std::cout << function << ":" << duration.count() << "us" << std::endl;   \
  }

// beginning of load map
bool loadMap(std::string map_streets_database_filename) {
    bool load_successful = false; //Indicates whether the map has loaded 
                                  //successfully
    auto start = std::chrono::high_resolution_clock::now(); 

       
    std::cout << "loadMap: " << map_streets_database_filename << std::endl;

    //
    // Load your map related data structures here.
    //
    mapData.streetIdxNames.clear();

    if (map_streets_database_filename.find("streets.bin") != std::string::npos) {
        loadStreetsDatabaseBIN(map_streets_database_filename);
        map_streets_database_filename.replace(map_streets_database_filename.find("streets.bin"), 11, "osm.bin");
        loadOSMDatabaseBIN(map_streets_database_filename);
    } else if (map_streets_database_filename.find("osm.bin") != std::string::npos) {
        loadStreetsDatabaseBIN(map_streets_database_filename);
        map_streets_database_filename.replace(map_streets_database_filename.find("osm.bin"), 7, "streets.bin");
        loadOSMDatabaseBIN(map_streets_database_filename);
    } else {
        return false;
    }
    

    // calling helper functions in load map to maximize efficiency when calling certain functions
    // TIME_MEASURE("loadStreetLengthsAndTravelTime", loadStreetLengthsAndTravelTime());
    // TIME_MEASURE("loadIntersectionStreetSegments", loadIntersectionStreetSegments());
    // TIME_MEASURE("loadIntersectionofStreets", loadIntersectionofStreets());
    // TIME_MEASURE("loadMultimapForPartialNames", loadMultimapForPartialNames());
    // TIME_MEASURE("populateNodeIdToNodeMap", populateNodeIdToNodeMap());
    // TIME_MEASURE("populateWayIdToWayMap", populateWayIdToWayMap());

    // TIME_MEASURE("loadOSMNodes", loadOSMNodes());
    // TIME_MEASURE("loadOSMWays", loadOSMWays());
    // TIME_MEASURE("loadIntersectionData", loadIntersectionData());
    // TIME_MEASURE("loadStreetSegmentData", loadStreetSegmentData());
    // TIME_MEASURE("loadFeatures", loadFeatures());
    // TIME_MEASURE("loadOSMRailRoads", loadOSMRailRoads());
    // TIME_MEASURE("loadSubwayStations", loadSubwayStations());
    // TIME_MEASURE("loadPOIImage", loadPOIImage());
    // TIME_MEASURE("loadSubwayRelations", loadSubwayRelations());
    
    

loadStreetLengthsAndTravelTime();
loadIntersectionStreetSegments();
loadIntersectionofStreets();
loadMultimapForPartialNames();
populateNodeIdToNodeMap();
populateWayIdToWayMap();

loadOSMNodes();
loadOSMWays();
loadIntersectionData();
loadStreetSegmentData();
loadFeatures();
loadOSMRailRoads();
loadSubwayStations();
loadPOIImage();
loadSubwayRelations();
loadBusRoutes();
loadBikeRoutes();
    
    // M3 Load nodes


//  M4
//  loadLandmarks(3);

    


    load_successful = true; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    // Timer for the map
    auto stop = std::chrono::high_resolution_clock::now(); 
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start); 
    std::cout << "Loadmap" << ":" << duration.count() << "s" << std::endl; 

    return load_successful;
}

// functtion to close map
void closeMap() {
  // Clean-up your map related data structures here

  // Clear M1 Global Variables
  // Clear all street lengths
  mapData.streetLengths.clear();
  // Clear all 2D vector that was used to store all street connnecting all
  // intersection
  mapData.intersection_street_segments.clear();
  // Clear all global variable for travel time
  mapData.travelTime.clear();
  // Clear 2D vector that stores all intersections of a given street
  mapData.intersection_of_streets.clear();
  // Clear unordered maps used for OSM functions
  mapData.nodeIdToNodeMap.clear();
  mapData.wayIdToWayMap.clear();
  mapData.osmNodeTags.clear();
  // Clear vectors and multimap for findStreetIdsFromPartialStreetName
  mapData.streetNamesLowercased.clear();
  mapData.streetIdxNames.clear();

  // Clear M2 Global Variables
  mapData.intersections.clear();
  mapData.street_segments.clear();
  mapData.features.clear();
  mapData.osmNodes.clear();
  mapData.osmWays.clear();
  mapData.osmRailIndex.clear();
  mapData.railWayNodes.clear();
  mapData.subwayStations.clear();

  // Call closeStreetDatabase to unload a map / frees the memory used by the API
  closeStreetDatabase();
  closeOSMDatabase();
  return;
}

// function that returns distance between two points given two latlon positions
double findDistanceBetweenTwoPoints(LatLon point_1, LatLon point_2) {
  // Initialize Variables
  std::pair<std::pair<double, double>, std::pair<double, double>>
      resultCoordinates = LatlonToXY(point_1, point_2);
  double point1_x, point1_y, point2_x, point2_y;
  point1_x = resultCoordinates.first.first;
  point1_y = resultCoordinates.first.second;
  point2_x = resultCoordinates.second.first;
  point2_y = resultCoordinates.second.second;

  // Calculate the distance using x,y coordinates
  double distance =
      sqrt(pow((point2_y - point1_y), 2) + pow((point2_x - point1_x), 2));
  return distance;
}

// function that returns the length of a street segment given the street segment
// id
double findStreetSegmentLength(StreetSegmentIdx street_segment_id) {
  // The case that has no curve points
  // we need to find when we need to include curve points
  StreetSegmentInfo segmentInfo = getStreetSegmentInfo(
      street_segment_id);  // get street segment info from street_segment_id

  int numCurvePoints = segmentInfo.numCurvePoints;
  IntersectionIdx segmentFrom =
      segmentInfo.from;  // access the street segment starting point
  IntersectionIdx segmentTo =
      segmentInfo.to;  // access the street segment end point
  LatLon startingPointPosition =
      getIntersectionPosition(segmentFrom);  // starting point lat/lon
  LatLon endPointPosition =
      getIntersectionPosition(segmentTo);  // end point lat/lon
  double distance;
  // Look at a segment in three cases, 0,1 and many curve points
  // Case 1: 0 curve points
  if (numCurvePoints == 0) {
    // Length would just be distance between starting and ending position
    distance =
        findDistanceBetweenTwoPoints(startingPointPosition, endPointPosition);
  }
  // Case 2: 1 curve point
  else if (numCurvePoints == 1) {
    // Length would be starting position   curve point + curve point to ending
    // position
    LatLon curvePointPosition =
        getStreetSegmentCurvePoint(0, street_segment_id);
    distance =
        findDistanceBetweenTwoPoints(startingPointPosition,
                                     curvePointPosition) +
        findDistanceBetweenTwoPoints(curvePointPosition, endPointPosition);
  }
  // Case 3: many curve points
  else {
    // Length would be considered in three parts: begin, middle, end
    // begin: starting position to first curve point
    // middle: 1st - 2nd, 2nd - 3rd ... second last - last
    // end: last curve point to ending position
    distance = 0;
    // Initialize first and last curve point position
    LatLon firstCurvePointPosition =
        getStreetSegmentCurvePoint(0, street_segment_id);
    LatLon lastCurvePointPosition =
        getStreetSegmentCurvePoint(numCurvePoints - 1, street_segment_id);
    // distance += (begin + end)
    distance +=
        findDistanceBetweenTwoPoints(startingPointPosition,
                                     firstCurvePointPosition) +
        findDistanceBetweenTwoPoints(lastCurvePointPosition, endPointPosition);

    // Loop from first curve point to the second last curve point
    for (int point_num = 0; point_num < numCurvePoints - 1; point_num++) {
      LatLon leadingCurvePointPosition =
          getStreetSegmentCurvePoint(point_num, street_segment_id);
      LatLon trailingCurvePointPosition =
          getStreetSegmentCurvePoint(point_num + 1, street_segment_id);
      // distance += distance from current curve point to the next curve point
      distance += findDistanceBetweenTwoPoints(leadingCurvePointPosition,
                                               trailingCurvePointPosition);
    }
  }

  return distance;  // return some double value
}

// return travel time for a given street segment
double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id) {
  return mapData.travelTime[street_segment_id];
}

// return the angle between two given street segments
double findAngleBetweenStreetSegments(StreetSegmentIdx src_street_segment_id,
                                      StreetSegmentIdx dst_street_segment_id) {
  // Get src and dst street
  // Access its number of curve points
  StreetSegmentInfo srcSegmentInfo =
      getStreetSegmentInfo(src_street_segment_id);
  StreetSegmentInfo dstSegmentInfo =
      getStreetSegmentInfo(dst_street_segment_id);

  int srcNumCurvePoints = srcSegmentInfo.numCurvePoints;
  int dstNumCurvePoints = dstSegmentInfo.numCurvePoints;

  // Access possible points from segment info (src to = dst from, so we include
  // only one of them)
  IntersectionIdx srcSegmentFrom = srcSegmentInfo.from;
  IntersectionIdx srcSegmentTo = srcSegmentInfo.to;
  IntersectionIdx dstSegmentFrom = dstSegmentInfo.from;
  IntersectionIdx dstSegmentTo = dstSegmentInfo.to;

  // Initialize the three points of interest used to calculate angle using
  // cosine law
  LatLon firstPoint, secondPoint, thirdPoint;
  // Case 1: From -- to --- from --- to
  if (srcSegmentTo == dstSegmentFrom) {
    // Check if src has any curve points
    if (srcNumCurvePoints == 0) {
      // If no, first two points would be the starting and ending position of
      // the src segment
      firstPoint = getIntersectionPosition(srcSegmentFrom);
      secondPoint = getIntersectionPosition(srcSegmentTo);
    } else {
      // If yes, first point would be the last curve point of the src segment
      // and second point would be the ending position of the src segment
      firstPoint = getStreetSegmentCurvePoint(srcNumCurvePoints - 1,
                                              src_street_segment_id);
      secondPoint = getIntersectionPosition(srcSegmentTo);
    }

    // Check if dst has any curve points
    if (dstNumCurvePoints == 0) {
      // If no, third point would be the ending position of dst segment
      thirdPoint = getIntersectionPosition(dstSegmentTo);
    } else {
      // If yes, third point would be the first curve point of dst segment
      thirdPoint = getStreetSegmentCurvePoint(0, dst_street_segment_id);
    }
  }
  // Case 2: from --- to --- to --- from
  else if (srcSegmentTo == dstSegmentTo) {
    // Check if src has any curve points
    if (srcNumCurvePoints == 0) {
      // If no, first two points would be the starting and ending position of
      // the src segment
      firstPoint = getIntersectionPosition(srcSegmentFrom);
      secondPoint = getIntersectionPosition(srcSegmentTo);
    } else {
      // If yes, first point would be the last curve point of the src segment
      // and second point would be the ending position of the src segment
      firstPoint = getStreetSegmentCurvePoint(srcNumCurvePoints - 1,
                                              src_street_segment_id);
      secondPoint = getIntersectionPosition(srcSegmentTo);
    }

    // Check if dst has any curve points
    if (dstNumCurvePoints == 0) {
      // If no, third point would be the starting position of dst segment
      thirdPoint = getIntersectionPosition(dstSegmentFrom);
    } else {
      // If yes, third point would be the last curve point of dst segment
      thirdPoint = getStreetSegmentCurvePoint(dstNumCurvePoints - 1,
                                              dst_street_segment_id);
    }
  }
  // Case 3: to --- from ---- from --- to
  else if (srcSegmentFrom == dstSegmentFrom) {
    // Check if src has any curve points
    if (srcNumCurvePoints == 0) {
      // If no, first two points would be the ending and starting position of
      // the src segment
      firstPoint = getIntersectionPosition(srcSegmentTo);
      secondPoint = getIntersectionPosition(srcSegmentFrom);
    } else {
      // If yes, first point would be the first curve point of the src segment
      // and second point would be the starting position of the src segment
      firstPoint = getStreetSegmentCurvePoint(0, src_street_segment_id);
      secondPoint = getIntersectionPosition(srcSegmentFrom);
    }

    // Check if dst has any curve points
    if (dstNumCurvePoints == 0) {
      // If no, third point would be the ending position of dst segment
      thirdPoint = getIntersectionPosition(dstSegmentTo);
    } else {
      // If yes, third point would be the first curve point of dst segment
      thirdPoint = getStreetSegmentCurvePoint(0, dst_street_segment_id);
    }
  }
  // Case 4: to --- from --- to --- from
  else if (srcSegmentFrom == dstSegmentTo) {
    // Check if src has any curve points
    if (srcNumCurvePoints == 0) {
      // If no, first two points would be the ending and starting position of
      // the src segment
      firstPoint = getIntersectionPosition(srcSegmentTo);
      secondPoint = getIntersectionPosition(srcSegmentFrom);
    } else {
      // If yes, first point would be the first curve point of the src segment
      // and second point would be the starting position of the src segment
      firstPoint = getStreetSegmentCurvePoint(0, src_street_segment_id);
      secondPoint = getIntersectionPosition(srcSegmentFrom);
    }

    // Check if dst has any curve points
    if (dstNumCurvePoints == 0) {
      // If no, third point would be the ending position of dst segment
      thirdPoint = getIntersectionPosition(dstSegmentFrom);
    } else {
      // If yes, third point would be the last curve point of dst segment
      thirdPoint = getStreetSegmentCurvePoint(dstNumCurvePoints - 1,
                                              dst_street_segment_id);
    }

  } else {
    return NO_ANGLE;
  }

  // Get longitudes latitudes in radians
  std::pair<std::pair<double, double>, std::pair<double, double>> vec1 =
      LatlonToXY(firstPoint, secondPoint);
  std::pair<std::pair<double, double>, std::pair<double, double>> vec2 =
      LatlonToXY(secondPoint, thirdPoint);

  // Get first point x, y coordinates
  double x1_12 = vec1.first.first;
  double y1_12 = vec1.first.second;
  double x2_12 = vec1.second.first;
  double y2_12 = vec1.second.second;

  double x1_23 = vec2.first.first;
  double y1_23 = vec2.first.second;
  double x2_23 = vec2.second.first;
  double y2_23 = vec2.second.second;

  // Get the two vectors needed for dot product
  double vec1x = x2_12 - x1_12;
  double vec1y = y2_12 - y1_12;
  double vec2x = x2_23 - x1_23;
  double vec2y = y2_23 - y1_23;

  // Compute dot product
  double dot = vec1x * vec2x + vec1y * vec2y;

  // Compute determinant
  double det = vec1x * vec2y - vec1y * vec2x;

  // // Compute magnitudes
  // double magnitudeVec1 = sqrt(pow(vec1x, 2) + pow(vec1y, 2));
  // double magnitudeVec2 = sqrt(pow(vec2x, 2) + pow(vec2y, 2));

  // // Compute cos(theta)
  // double cost = sum/(magnitudeVec1 * magnitudeVec2);

  // // Compute theta
  // double theta = acos(cost);

  // if(cross < 0){
  //     theta = -theta;
  // }

  return atan2(det, dot);  // return some double value
}

// return true if two intersections are directly connected
bool intersectionsAreDirectlyConnected(
    std::pair<IntersectionIdx, IntersectionIdx> intersection_ids) {
  // According to test cases, when two intersections are equal we can legally
  // drive from one point to itself
  if (intersection_ids.first == intersection_ids.second) {
    return true;
  }

  // We loop through all the street segments that goes through the intersection,
  // if any segment has two intersections that matches the input, we return true
  for (int seg_num = 0;
       seg_num < getNumIntersectionStreetSegment(intersection_ids.first);
       seg_num++) {
    StreetSegmentIdx currentStreetSeg =
        getIntersectionStreetSegment(seg_num, intersection_ids.first);
    StreetSegmentInfo currentStreetInfo =
        getStreetSegmentInfo(currentStreetSeg);
    IntersectionIdx first = intersection_ids.first;
    IntersectionIdx second = intersection_ids.second;
    IntersectionIdx from = currentStreetInfo.from;
    IntersectionIdx to = currentStreetInfo.to;

    if ((first == from && second == to) || (first == to && second == from)) {
      return true;
    }
  }
  // did not find any matching street segment
  return false;
}

// return the closet intersection from a given position
IntersectionIdx findClosestIntersection(LatLon my_position) {
  // get the total number of intersections possible
  int num_of_intersections = getNumIntersections();
  // set the closest intersection to the first one and change it later
  int min = 0;
  // use a for loop to loop through all the possibile intersections
  for (int i = 1; i < num_of_intersections; i++) {
    // compare the distance between the current min intersection and another
    // intersection, if the latter is smaller, then update min
    if (findDistanceBetweenTwoPoints(my_position,
                                     getIntersectionPosition(min)) >
        findDistanceBetweenTwoPoints(my_position, getIntersectionPosition(i))) {
      min = i;
    }
  }
  // return the closest intersection
  return min;
}

// return the id of the closet intersection
std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(
    IntersectionIdx intersection_id) {
  // directly return the vector containing the pre-generated street segments at
  // a given intersection
  return mapData.intersection_street_segments[intersection_id];
}

// return all intersections of a street
std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id) {
  // directly return the vector containing the pre-generate intersections given
  // a street id

  std::vector<IntersectionIdx> setToVector(
      mapData.intersection_of_streets[street_id].begin(),
      mapData.intersection_of_streets[street_id].end());
  return setToVector;
  // Access the element at index 2 from the end

  // This will advance the iterator by 2 positions

  // Access the element at the desired index
}

// return all intersections of two streets
std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(
    std::pair<StreetIdx, StreetIdx> street_ids) {
  // get the intersections in the first street
  std::vector<IntersectionIdx> sorted_first(
      mapData.intersection_of_streets[street_ids.first].begin(),
      mapData.intersection_of_streets[street_ids.first].end());
  // get the intersections in the second street
  std::vector<IntersectionIdx> sorted_second(
      mapData.intersection_of_streets[street_ids.second].begin(),
      mapData.intersection_of_streets[street_ids.second].end());
  // create an empty vector to store the intersections at which the two given
  // streets intersect
  std::vector<IntersectionIdx> street_intersections;
  // sort the intersections in the first street
  std::sort(sorted_first.begin(), sorted_first.end());
  // sort the intersections in the second street
  std::sort(sorted_second.begin(), sorted_second.end());
  // find the set intersection of the first and second street intersection
  // vectors and store them in street_intersections
  std::set_intersection(sorted_first.begin(), sorted_first.end(),
                        sorted_second.begin(), sorted_second.end(),
                        back_inserter(street_intersections));
  // return the vector
  return street_intersections;
}

// return street from a partial street name
std::vector<StreetIdx> findStreetIdsFromPartialStreetName(
    std::string street_prefix) {
  std::vector<StreetIdx> street_ids_partial = {};
  // Check for empty string
  if (street_prefix == "") {
    return street_ids_partial;
  }

  // Remove spaces in input
  std::string lowercaseStreetPrefix = street_prefix;
  lowercaseStreetPrefix.erase(std::remove(lowercaseStreetPrefix.begin(),
                                          lowercaseStreetPrefix.end(), ' '),
                              lowercaseStreetPrefix.end());
  // Convert them to lowercases
  lowercaseStreetPrefix = str_tolower(lowercaseStreetPrefix);
  std::string upperBound =
      lowercaseStreetPrefix;  // Upper bound used for efficient searching
  upperBound[lowercaseStreetPrefix.size() - 1] =
      upperBound[lowercaseStreetPrefix.size() - 1] + 1;

  // Iterate over the multimap within the bounds defined by the modified street
  // prefix (lowerlowercaseStreetPrefix -> upperBound)
  for (auto it = mapData.streetIdxNames.lower_bound(lowercaseStreetPrefix);
       it != mapData.streetIdxNames.upper_bound(upperBound); ++it) {
    if (it->first.compare(0, lowercaseStreetPrefix.size(),
                          lowercaseStreetPrefix) == 0) {
      street_ids_partial.push_back(it->second);
    }
  }

  return street_ids_partial;
}

// return the street length
double findStreetLength(StreetIdx street_id) {
  return mapData
      .streetLengths[street_id];  // return values in the street length, loaded
                                  // when we call load map
}

// return the closest point of interest id
POIIdx findClosestPOI(LatLon my_position, std::string poi_name) {
  // Initialize Variables
  POIIdx closestPOI = 0;
  int closestDistance = 0;
  bool foundPOI = false;

  // Get the number of points of interest
  int numOfPOIs = getNumPointsOfInterest();

  // Use a for loop to loop through each POI
  for (int i = 0; i < numOfPOIs; i++) {
    // Find the POI that matches poi_name
    if (getPOIName(i) == poi_name) {
      // If we found that POI then,
      if (!foundPOI) {
        // Use the findDistanceBetweenTwoPoints function to find the distance
        // between my_position and the POI, and assign this to closestDistance
        closestDistance =
            findDistanceBetweenTwoPoints(my_position, getPOIPosition(i));
        // closestPOI gets assigned the current index (also known as the POIIdx)
        closestPOI = i;
        // Then, we set foundPOI to true such that we won't enter this condition
        // again
        foundPOI = true;
      }

      // The reason for the previous if statement is to assign an initial value
      // to closestDistance, such that, we can compare closestDistance with the
      // next matching POI

      // Then, we check: Is the next POI that we found closer than the current
      // closestDistance?
      if (closestDistance >
          findDistanceBetweenTwoPoints(my_position, getPOIPosition(i))) {
        // If yes, then we'll update closestDistance and closestPOI with the new
        // value
        closestDistance =
            findDistanceBetweenTwoPoints(my_position, getPOIPosition(i));
        closestPOI = i;
      }
    }
  }

  // return the closest POIIdx (which is an integer)
  return closestPOI;
}

// find and return the area of a feature
double findFeatureArea(FeatureIdx feature_id) {
  // Initialize Variables
  double areaOfTrapezoid = 0.0;
  double featureArea = 0.0;

  // Get total number of feature points for the given feature (points are
  // ordered)
  int numOfFeaturePoints = getNumFeaturePoints(feature_id);

  // Compare first and last LatLon Points to check if it's a polygon or not
  // Check if num of feature points > 2 since if it's not then its not a valid
  // polygon
  if (numOfFeaturePoints > 2 &&
      (getFeaturePoint(0, feature_id) ==
       getFeaturePoint(numOfFeaturePoints - 1, feature_id))) {
    // Use a for loop to calculate area of polygon
    for (int pointNum = 0; pointNum < numOfFeaturePoints; pointNum++) {
      // Point Num is what we will use to getFeaturePoint

      // Feature Point 1
      LatLon featurePoint_1 = getFeaturePoint(pointNum, feature_id);

      // Feature Point 2 (we do pointNum + 1 % numOfFeaturePoints as this ensure
      // after the last point, numOfFeaturePoints - 1 in the for loop condition,
      // the expression pointNum + 1 % numOfFeaturePoints will evaluate to
      // numOfFeaturePoints % numOfFeaturePoints = 0, so pointNum + 1 will go
      // back to first point of the polygon)
      LatLon featurePoint_2 =
          getFeaturePoint((pointNum + 1) % numOfFeaturePoints, feature_id);

      // Convert feature points from LatLon to (x,y)
      std::pair<std::pair<double, double>, std::pair<double, double>>
          coordinates = LatlonToXY(featurePoint_1, featurePoint_2);

      // Accessing the first pair of coordinates
      double point1_x = coordinates.first.first;
      double point1_y = coordinates.first.second;

      // Accessing the second pair of coordinates
      double point2_x = coordinates.second.first;
      double point2_y = coordinates.second.second;

      // Take feature points to find area of trapezoid
      areaOfTrapezoid = 0.5 * (point2_y - point1_y) * (point2_x + point1_x);

      // Add the sums of the area of trapezoid
      featureArea += areaOfTrapezoid;
    }

    return fabs(featureArea);  // return feature area (we take absolute value
                               // for cases where area is negative)
  } else {
    // If not then return 0.0
    return 0.0;
  }
}

// return the length of a specified OSM way
double findWayLength(OSMID way_id) {
  // Initialize Variables
  double totalLength = 0.0;

  // Directly access the way using the hash map
  auto it = mapData.wayIdToWayMap.find(way_id);

  // Check if the OSMWay with the corresponding way_id exists or not
  if (it == mapData.wayIdToWayMap.end()) {
    // If not, return 0.0
    return 0.0;
  }

  // Once we found the OSMWay that corresponds to the wayID
  const OSMWay* way = it->second;

  // We can use getWayMembers to get the members of the OSMWay (which are just
  // OSMNodes)
  const std::vector<OSMID>& wayMembers = getWayMembers(way);

  // Check if number of way members is <= 1 as this means that it's not an
  // OSMWay if yes, return 0.0
  if (wayMembers.size() <= 1) {
    return 0.0;
  }

  // Use a for loop to loop through all the wayMembers and add up all the total
  // length between two nodes
  for (unsigned j = 0; j < wayMembers.size() - 1; ++j) {
    // ID of first wayMembers (which are basically the node IDs)
    const OSMID id_1 = wayMembers[j];
    // ID of second node (that comes after first node)
    const OSMID id_2 = wayMembers[j + 1];

    // Directly access nodes from the map with the corresponding IDs
    const OSMNode* node1 = mapData.nodeIdToNodeMap[id_1];
    const OSMNode* node2 = mapData.nodeIdToNodeMap[id_2];

    // Use findDistanceBetweenTwoPoints function to find the length between the
    // two nodes
    totalLength += findDistanceBetweenTwoPoints(getNodeCoords(node1),
                                                getNodeCoords(node2));
  }

  // return the total length of the OSMway
  return totalLength;
}

// return the value associated with a specific key on a given OSM node
std::string getOSMNodeTagValue(OSMID osm_id, std::string key) {
  // Find the node in the osmNodeTags map using itss OSM ID
  const OSMNode* currNode = mapData.nodeIdToNodeMap[osm_id];

  for (int i = 0; i < getTagCount(currNode); i++) {
    std::string tagKey, value;
    std::tie(tagKey, value) = getTagPair(currNode, i);

    if (tagKey == key) {
      return value;
    }
  }

  return "";
}