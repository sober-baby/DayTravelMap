
#ifndef GLOBALS_H
#define GLOBALS_H
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include <iostream>
#include <map>
#include <vector>
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include <cmath>
#include <utility>
#include "m1.h"
#include <sstream>
#include <fstream>
#include <chrono>
#include <set>
#include <queue>




/*
   This file contains all the data structures and global variables that will be used across different milestones.

*/

// Enumerations
// WayType defines the type of street segments that exists in OSM. It is used to classify street segments into major and non-major roads.
enum WayType {
    NA,
    MOTORWAY,
    TRUNK,
    PRIMARY,
    SECONDARY,
    TERTIARY,
    UNCLASSIFIED,
    RESIDENTIAL,
    MOTORWAY_LINK,
    TRUNK_LINK,
    PRIMARY_LINK,
    SECONDARY_LINK,
    TERTIARY_LINK,
    LIVING_STREET,
    SERVICE,
    PEDESTRIAN,
    TRACK,
    BUS_GUIDEWAY,
    ESCAPE,
    RACEWAY,
    ROAD,
    BUSWAY,
    FOOTWAY,
    BRIDLEWAY,
    STEPS,
    CORRIDOR,
    PATH,
    VIA_FERRANTA,
    CYCLEWAY,
    PROPOSED,
    CONSTRUCTION
};

// Data structures initialization
// Intersection_data defines data that is related to an intersection.
// It contains xy_loc (location of the intersection), name and a state indicating if the intersection has been highlighted.
struct Intersection_data {
   ezgl::point2d xy_loc;
   std::string name;
   bool highlight = false;
};


// Street_segment_data defines data that is related to a street segment.
// It contains numberOfCurvePoints, a list of all curve (including from and to in a street segment) point coordinates,
// wayType as defined above to classify major/minor streets, oneWay to indicate if the street segment is one-way.
struct Street_segment_data {
   IntersectionIdx from, to;
   int numberOfCurvePoints;
   int streetId;
   std::vector<ezgl::point2d> all_curve_points;
   std::vector<ezgl::point2d> midpoint;
   std::vector<double> angle;
   std::string streetName;
   WayType wayType;
   bool oneWay;

};

// Feature_data defines data that is related to a feature.
// It contains a featureType (used to determine feature-drawing order), number of feature points,
// all_feature_points (a list of all the ezgl::point2d coordinate that defines the boundary of the feature), and featureArea
struct Feature_data {
   FeatureType featureType;
   int numFeaturePoints;
   std::vector<ezgl::point2d> all_feature_points;
   double featureArea;
};

//strucutre to store POI_data, including the surface(which is the image), as well as its coordinate
struct POI_data{
   ezgl::surface* poi_Surface;
   ezgl::point2d poi_Coordinate;

};






// Subway_Station defines data that is related to a subway station
// Since each subway station is a node, we are able extract its location as ezgl::point2d and also its name
struct Subway_Station {
   ezgl::point2d xy_loc;
   std::string name;
};

struct Subway_Lines {
   std::vector<ezgl::point2d> wayPositions;
   std::string color;
};

struct Bus_Lines {
   std::vector<ezgl::point2d> wayPositions;
};

struct Bike_Route {
   std::vector<ezgl::point2d> wayPositions;
};

// M3 structures



#define NO_EDGE -1

struct Node {
   public:
      // Store outgoing edges?
      std::vector<StreetSegmentIdx> outgoingEdges;
      // See if the node is visited
      bool visited;

      // Id of the edge used to reach this node
      StreetSegmentIdx reachingEdge;

      // Store the shortest time to this node so far
      double bestTime = 999999999;


};

// M4 Struct
struct PickDropDepot {
   public:
      IntersectionIdx intersectionID;

      bool isPickUp;
      
      bool isDropOff;
      IntersectionIdx respPickUp;

      bool visited = false;

};

class GlobalVars {

   public:
      /****************  M1  **********************/
      // All street lengths
      std::vector<double> streetLengths;

      // 2D vector to store all street connnecting all intersection
      std::vector<std::vector<StreetSegmentIdx>> intersection_street_segments;

      // Global variable for travel time
      std::vector<double> travelTime;

      // 2D vector to store all intersections of a given street
      std::vector<std::set<IntersectionIdx>> intersection_of_streets;

      // Unordered maps for OSM functions
      std::unordered_map<OSMID, const OSMNode*> nodeIdToNodeMap;
      std::unordered_map<OSMID, const OSMWay*> wayIdToWayMap;
      std::unordered_map<OSMID, std::unordered_map<std::string, std::string>> osmNodeTags;

      // Vector and multimap for findStreetIdsFromPartialStreetName
      std::vector<std::string> streetNamesLowercased;
      std::multimap<std::string, int> streetIdxNames;

      /****************  M2  **********************/
      // Global Variable
      // The five variables below are used to define the map boundaries
      double latAvg;  
      bool switchState;
      bool darkMode;
      double max_lat; 
      double min_lat;
      double max_lon;
      double min_lon;
      bool choosingIntersection = false;
      bool clickingIntersection = false;
      bool typingIntersection = false;
      bool bikeRoutes = false;
      bool busRoutes = false;
      

      //load all the png(icons in this case) from file so that it can be later shown on the map



      // ezgl surfaces for plotting png on the map
      ezgl::surface* poiSurface1 = ezgl::renderer::load_png("./libstreetmap/src/images/default.png");
      ezgl::surface* poiSurface3 = ezgl::renderer::load_png("./libstreetmap/src/images/healthcare.png");
      ezgl::surface* poiSurface4 = ezgl::renderer::load_png("./libstreetmap/src/images/financial.png");
      ezgl::surface* poiSurface5 = ezgl::renderer::load_png("./libstreetmap/src/images/restaurant.png");
      ezgl::surface* poiSurface6 = ezgl::renderer::load_png("./libstreetmap/src/images/services.png");
      ezgl::surface* poiSurface7 = ezgl::renderer::load_png("./libstreetmap/src/images/entertainment.png");
      ezgl::surface* poiSurface8 = ezgl::renderer::load_png("./libstreetmap/src/images/government.png");
      ezgl::surface* poiSurfaceStart = ezgl::renderer::load_png("./libstreetmap/src/images/rec.png");
      ezgl::surface* poiSurfaceEnd = ezgl::renderer::load_png("./libstreetmap/src/images/location.png");

      //ezgl::surface* poiSurface2 = ezgl::renderer::load_png("libstreetmap/src/images/education.png");
      

      // A std vector that stores entries of intersection data
      std::vector<Intersection_data> intersections;

      // Store all the street_segments related data
      std::vector<Street_segment_data> street_segments;

      // Store all the features-related data
      std::vector<Feature_data> features;
      std::vector<Feature_data> nonWaterFeatures;

      // Stores all the osmNode id in pair with their respective node index, we can then use OSMID to access nodes
      std::map<OSMID, int> osmNodes;

      // Stores all the osmWays that has certain keys into other vectors/maps
      std::map<OSMID, std::pair<std::string, std::string>> osmWays;

      //create a std::vector of POI data structures 
      std::vector<POI_data> POIs_data;
      // Stores all the osmWays that are railways and its index, we can then use the index to access the way
      std::map<OSMID, int> osmRailIndex;

      // Stores all the railWay's nodes position in ezgl::point2d
      std::vector<std::vector<ezgl::point2d>> railWayNodes;

      // Stores all the subway stations' relevant information as defined in the structures above
      std::vector<Subway_Station> subwayStations;


      // Store the subway lines information
      std::vector<Subway_Lines> subways;

      // Store the bus lines information
      std::vector<Bus_Lines> bus;

      // Store the bike routes information
      std::vector<Bike_Route> bike;


      // Used in load map selection, contains all the map names 
      std::vector<std::string> mapNames = {
         "beijing_china",
         "boston_usa",
         "cape-town_south-africa",
         "golden-horseshoe_canada",
         "hamilton_canada",
         "hong-kong_china",
         "iceland",
         "interlaken_switzerland",
         "kyiv_ukraine",
         "london_england",
         "new-delhi_india",
         "new-york_usa",
         "rio-de-janeiro_brazil",
         "saint-helena",
         "singapore",
         "sydney_australia",
         "tehran_iran",
         "tokyo_japan",
         "toronto_canada"
      };
      
      std::unordered_map<std::string, std::string> mapCityCountry{
         {"beijing_china", "Beijing,CN"},
         {"boston_usa", "Boston,US"},
         {"cape-town_south-africa", "Cape Town,ZA"},
         {"golden-horseshoe_canada", "Golden Horseshoe,CA"},
         {"hamilton_canada", "Hamilton,CA"},
         {"hong-kong_china", "Hong Kong,HK"},
         {"iceland", "Iceland"},
         {"interlaken_switzerland", "Interlaken,CH"},
         {"kyiv_ukraine", "Kyiv,UA"},
         {"london_england", "London,GB"},
         {"new-delhi_india", "New Delhi,IN"},
         {"new-york_usa", "New York,US"},
         {"rio-de-janeiro_brazil", "Rio de Janeiro,BR"},
         {"saint-helena", "Saint Helena"},
         {"singapore", "Singapore,SG"},
         {"sydney_australia", "Sydney,AU"},
         {"tehran_iran", "Tehran,IR"},
         {"tokyo_japan", "Tokyo,JP"},
         {"toronto_canada", "Toronto,CA"}
      };

      std::vector<std::string> globalStreetNames;

      std::vector<int> previousHighlightedIntersectionID;


      // M3
      std::vector<Node> nodes;
      double fastestSpeedLimit;
      std::vector<int> landmarks;
      std::vector<std::vector<double>> landmarkDist;

      // Global var to store the intersections to be passed into the find path function
      std::pair<IntersectionIdx, IntersectionIdx> intersect_ids; 
      std::vector<IntersectionIdx> selectedIntersection;
      std::vector<StreetSegmentIdx> path;

      std::vector<IntersectionIdx> mouseClickedInter;
      std::vector<IntersectionIdx> checkingID;

      std::vector<IntersectionIdx> clickedInter;



      // M4
      // Global variables for exhaustive pathfinding
      std::vector<std::vector<double>> timeMatrix;
      std::vector<std::vector<std::vector<StreetSegmentIdx>>> pathMatrix;
      double minPathTime = 99999999;
      std::vector<double> chosenPath;
      std::unordered_map<int, int> intersectionToMatrixIndex;
};


extern GlobalVars mapData;
class WaveElem {

   public:
      IntersectionIdx interId;
      StreetSegmentIdx segId;
      double travelTime;
      // double distToDest;


      // Constructor
      WaveElem(int n, int e , double time/*, double distance*/) {
         interId = n;
         segId = e;
         travelTime = time;
         // distToDest = distance;
      }

      // Can also declare this as a class outside
      class Compare {
         public:
            bool operator()(const WaveElem& a, const WaveElem& b) {
               
               // std::cout << a.travelTime << std::endl;
               
               // double heuristicA = a.travelTime + a.distToDest/(mapData.fastestSpeedLimit);
               // double heuristicB = b.travelTime + b.distToDest/(mapData.fastestSpeedLimit);
               // return heuristicA > heuristicB;
               return a.travelTime > b.travelTime;
            }
      };
    
};
#endif