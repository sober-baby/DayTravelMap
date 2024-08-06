#include "m1HelperFunctions.h"
#include "globals.h"
#include <unordered_set>

extern GlobalVars mapData;
// Convert a string to its lowercase
std::string str_tolower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c);});
    return s;
}


/*****************************   Helper Functions for LoadMap   **********************************/

void loadStreetLengthsAndTravelTime() {
    // Initialize maxSpeed Limit
    mapData.fastestSpeedLimit = 0;
    // Load the lengths in for each street and travel time for street segments
    
    // Allocate enough space for the std vector to store value of each street length and travel time
    mapData.streetLengths.reserve(getNumStreets());
    mapData.travelTime.reserve(getNumStreets());

    // Loop through to initialize initial length to 0;
    for (int street_num = 0; street_num < getNumStreets(); street_num++) {
        mapData.streetLengths[street_num] = 0.0;
    }

    // Loop through every street segment, add their length and travel time to respective street index
    for (int street_seg_num = 0; street_seg_num < getNumStreetSegments(); street_seg_num++) {
        // Add the length to respective streets
        StreetSegmentInfo segInfo = getStreetSegmentInfo(street_seg_num);
        double street_seg_length = findStreetSegmentLength(street_seg_num);
        mapData.streetLengths[segInfo.streetID] += street_seg_length;

        // Add the travel time in the list 
        float speedLimit = segInfo.speedLimit; // access speed limit from segment info
        double distance = findStreetSegmentLength(street_seg_num); // call the function above to get the street segment length
        double time = distance/speedLimit; // get time by distance / speed limit
        mapData.travelTime.push_back(time);

        // Added maxSpeed for m3
        if (speedLimit > mapData.fastestSpeedLimit) {
            mapData.fastestSpeedLimit = speedLimit;
        }
    }   
}

/*void loadIntersectionStreetSegments() {
    // Create empty vector for each intersection  (to achieve the function intersection_street_segments)
    mapData.intersection_street_segments.reserve(getNumIntersections());
    // looping through all the possible intersections on the map
    for(int i = 0; i < getNumIntersections(); i++){
        // create a empty vector for every intersection on the map
        mapData.intersection_street_segments.push_back(std::vector<StreetSegmentIdx>());
        // loop through all the street segments for a given intersection
        for(int j = 0; j < getNumIntersectionStreetSegment(i); j++){
            // push all the street segments into the 2D global vector for easy access
            int street_seg_id = getIntersectionStreetSegment(j, i);
            mapData.intersection_street_segments[i].push_back(street_seg_id);
        }
    }
}*/

void loadIntersectionStreetSegments() {
    // Create empty vector for each intersection  (to achieve the function intersection_street_segments)
    mapData.intersection_street_segments.resize(getNumIntersections());

    #pragma omp parallel for
    // looping through all the possible intersections on the map
    for(int i = 0; i < getNumIntersections(); i++){
        std::vector<StreetSegmentIdx> tempIntersectionStreetSegments;
        // create a empty vector for every intersection on the map
        tempIntersectionStreetSegments.resize(getNumIntersectionStreetSegment(i));
        // mapData.intersection_street_segments.push_back(std::vector<StreetSegmentIdx>());
        // loop through all the street segments for a given intersection
        for(int j = 0; j < getNumIntersectionStreetSegment(i); j++){
            // push all the street segments into the 2D global vector for easy access
            int street_seg_id = getIntersectionStreetSegment(j, i);
            //mapData.intersection_street_segments[i].push_back(street_seg_id);
            tempIntersectionStreetSegments.push_back(street_seg_id);
        }
        mapData.intersection_street_segments[i] = std::move(tempIntersectionStreetSegments);
    }
}

/*****************************   Helper Functions to help load the OSM data structure   **********************************/

void populateNodeIdToNodeMap() {
    // Map each nodeID to its corresponding OSMNode object for quick lookup of node details using the node ID
    for (unsigned i = 0; i < getNumberOfNodes(); ++i) {
        // Retrieve the node at index i
        const OSMNode* node = getNodeByIndex(i);
        // Store the node by value in the nodeIdToNodeMap, using the node's ID as the key
        mapData.nodeIdToNodeMap[node->id()] = node;
    }    
}


void populateWayIdToWayMap() {
    // Map each wayID to its corresponding OSMWay pointer for quick access to OSMWay objects using their IDs
    for (unsigned i = 0; i < getNumberOfWays(); ++i) {
        // Retrieve the way at index i
        const OSMWay* way = getWayByIndex(i); 
        // Store the pointer to the way in the wayIdToWayMap, using the way's ID as the key
        mapData.wayIdToWayMap[way->id()] = way;
    }
}

void populateMapWithNodeTags() {
    // Populating a map with tags for each OSM node, and associates an OSMID with its corresponding tags (stored in a nested unordered_map)
    for (unsigned i = 0; i < getNumberOfNodes(); ++i) {
        // Retrieve the node at index i
        const OSMNode* node = getNodeByIndex(i);
        // Extract the node's OSM ID
        OSMID osmID = node->id(); 
        // Prepare a map to store tags for this node
        std::unordered_map<std::string, std::string> tags; 

        // Iterate over all tags of the current node, and extract key-value pairs
        for(unsigned tagIndex = 0; tagIndex < getTagCount(node); ++tagIndex) {
            // Initialize Variables
            std::string key, value;
            // Extract each tag's key and value
            std::tie(key, value) = getTagPair(node, tagIndex);
            // Store into tag 
            tags[key] = value;
        }

        // Associate the node's ID with its tags in the osmNodeTags map
        mapData.osmNodeTags[osmID] = tags;
    }
}

 /*****************************   helper function for  findIntersectionsOfStreet     **********************************/

void loadIntersectionofStreets(){
    //get the number of streets
    int numStreets = getNumStreets();
    //get the number of street segments
    int NumStreetSegments = getNumStreetSegments();

   //loop through all the streets and add a empty vector
    for(int i = 0; i < numStreets; i++){
        mapData.intersection_of_streets.push_back(std::set<IntersectionIdx>());
    }

    //mapData.intersection_of_streets.resize(numStreets);

    //loop through all the street segments and determine which street they are on
    for(int j = 0; j < NumStreetSegments; j++){
        //get the street id for each interation
        StreetSegmentInfo segInfo = getStreetSegmentInfo(j);
        int street_id = segInfo.streetID;
        IntersectionIdx from = segInfo.from;
        IntersectionIdx to = segInfo.to;

        //use std built in find function to determine if the "from" intersection is already added to the vector, if not, push the value
        // auto it_from = std::find(mapData.intersection_of_streets[street_id].begin(), mapData.intersection_of_streets[street_id].end(), from);
        // if(it_from == mapData.intersection_of_streets[street_id].end()){
        //     mapData.intersection_of_streets[street_id].push_back(from);
        // }
        //use std built in find function to determine if the "to" intersection is already added to the vector, if not, push the value
        // auto it_to = std::find(mapData.intersection_of_streets[street_id].begin(), mapData.intersection_of_streets[street_id].end(), to);
        // if(it_to == mapData.intersection_of_streets[street_id].end()){
        //     mapData.intersection_of_streets[street_id].push_back(to);
        // }

        // mapData.intersection_of_streets[from] = ;
        //mapData.intersection_of_streets[street_id].push_back(to);

        //insert the from and t
        mapData.intersection_of_streets[street_id].insert(from);
        mapData.intersection_of_streets[street_id].insert(to);

    }
}

/*****************************   helper function for  findStreetIdsFromPartialStreetName     **********************************/

void loadMultimapForPartialNames() {
    for (int street_num = 0; street_num < getNumStreets(); street_num++) {
        std::string streetName = getStreetName(street_num);
        // Store the no space street names in respective index of a global variable containing all street names
        streetName.erase(std::remove(streetName.begin(), streetName.end(), ' '), streetName.end());
        // Lowercase to standardize it
        streetName = str_tolower(streetName);

        // add the street name to the global variable 
        mapData.streetIdxNames.insert(std::pair{streetName,street_num});
        
    }
}

/*****************************   Helper function to convert two points in latitude and longitude into cartesian coordinates     **********************************/

std::pair<std::pair<double, double>, std::pair<double, double>> LatlonToXY (LatLon point1, LatLon point2) {
    double lat1, lon1, lat2, lon2;                  
    double point1_x, point1_y, point2_x, point2_y;      

    // Get latitude and longitude information as double for both points in radians
    lat1 = point1.latitude()*kDegreeToRadian;  
    lon1 = point1.longitude()*kDegreeToRadian;
    lat2 = point2.latitude()*kDegreeToRadian;
    lon2 = point2.longitude()*kDegreeToRadian;

    // Find average latitude by averaging the latitude of the two points
    double latAvg = (lat1 + lat2)/2;

    // Apply equation (x,y) = (R*lon*cos(latAvg), R*lat), where latAvg is already in radians according to LatLon.h
    point1_x = kEarthRadiusInMeters*lon1*cos(latAvg);
    point1_y = kEarthRadiusInMeters*lat1;
    point2_x = kEarthRadiusInMeters*lon2*cos(latAvg);
    point2_y = kEarthRadiusInMeters*lat2;

    std::pair<std::pair<double, double>, std::pair<double, double>> resultCoordinates;
    resultCoordinates.first.first = point1_x;
    resultCoordinates.first.second = point1_y;
    resultCoordinates.second.first = point2_x;
    resultCoordinates.second.second = point2_y;

    return resultCoordinates;
}

void closeMapFinal() {
    // Clean-up your map related data structures here

    // Clear M1 Global Variables
    // Clear all street lengths
    mapData.streetLengths.clear();
    // Clear all 2D vector that was used to store all street connnecting all intersection
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

    ezgl::renderer::free_surface(mapData.poiSurface1);
    //ezgl::renderer::free_surface(mapData.poiSurface2);
    ezgl::renderer::free_surface(mapData.poiSurface3);
    ezgl::renderer::free_surface(mapData.poiSurface4);
    ezgl::renderer::free_surface(mapData.poiSurface5);
    ezgl::renderer::free_surface(mapData.poiSurface6);
    ezgl::renderer::free_surface(mapData.poiSurface7);
    ezgl::renderer::free_surface(mapData.poiSurface8);

    return;
}
