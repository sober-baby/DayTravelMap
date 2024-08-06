#ifndef m2Helper_h
#define m2Helper_h
#include "m2.h"
#include <iostream>
#include <vector>
#include <map>
#include <utility>
#include "globals.h"


/*  This class is used for writing the helper functions to be used in m1/m2.cpp. The descriptions of each helper functions can be found below */


// Helper Function Declaration
void draw_main_canvas(ezgl::renderer *g);
void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y);
void load_css();
void initial_setup(ezgl::application* application, bool newWindow);
void helpButton(GtkWidget* /*widget*/, ezgl::application* application);
void searchButton(GtkWidget* /*widget*/, ezgl::application* application);
void loadNewCity(GtkWidget* /*widget*/, ezgl::application* application);
void nightMode(GtkSwitch* /*self*/, gboolean state, ezgl::application* app);
void create_mssg_button_cbk(GtkWidget* /*widget*/, ezgl::application* app);
void popUpSearch(GtkWidget* /*widget*/, ezgl::application* application);
void cancelSearch(GtkWidget* /*widget*/, ezgl::application* application);
void closedButton(GtkWidget* /*widget*/, ezgl::application* application);
void closedHelpButton(GtkWidget* /*widget*/, ezgl::application* application);
void confirmButton(GtkWidget* /*widget*/, ezgl::application* application);
void displayBikeRoutes(GtkWidget* /*widget*/, ezgl::application* application);
void displayBusRoutes(GtkWidget* /*widget*/, ezgl::application* application);

// These four functions convert the coordinates between LatLon and ezgl::point2d
float x_from_lon(float lon);
float y_from_lat(float lat);
float lon_from_x(float x);
float lat_from_y(float y);


// Loading Functions
// These functions load the related data structure upon calling loadMap()
void loadIntersectionData();
void loadFeatures();
void loadStreetSegmentData();
void loadOSMWays();
void loadOSMNodes();
void loadOSMRailRoads();
void loadSubwayStations();
void loadPOIImage();
void loadSubwayRelations();
void loadFeatureType(FeatureType desiredFeatureType);
void loadBusRoutes();
void loadBikeRoutes();


// Drawing Functions
// These functions draw the related map elements upon calling drawMap()
void drawStreetSegments(ezgl::renderer *g);
void drawStreetNames(ezgl::renderer *g);
void drawIntersections(ezgl::renderer *g);
void drawFeatures(ezgl:: renderer *g);
void drawPOIImage(ezgl:: renderer *g);
void drawRailWays(ezgl::renderer *g);
void drawSubwayStations(ezgl::renderer *g);
void drawLevelsOfStreets(int levelOfFeatures, ezgl::renderer *g);
void drawOneWayArrow(ezgl::renderer *g);
void drawSubwayLines(ezgl::renderer *g);
void drawBusRoutes(ezgl::renderer *g);
void drawBikeRoutes(ezgl::renderer *g);
size_t writeCallback(void *contents, size_t size, size_t nmemb, std::string *s);
void parseTrafficData(ezgl::application* app, const std::string& xmlData);
std::string fetchTrafficData(const std::string& url);
void displayLiveWeatherInfo(ezgl::application* app, std::string city);
void displayLiveWeather(GtkWidget* /*widget*/, ezgl::application* application);
void closedWeatherButton(GtkWidget* /*widget*/, ezgl::application* application);
void closeTrafficButton(GtkWidget* /*widget*/, ezgl::application* application);

// Determine whether features should display based on their area, a helper function to drawFeature to reduce code redundancy
// features vs. non water features are loaded into different global variables in loadFeatures.
void plotFeatures(FeatureIdx i, int numFeaturePoints, int areaLimit, ezgl::renderer *g);
void plotNonWaterFeatures(FeatureIdx i, int numFeaturePoints, int areaLimit, ezgl::renderer *g);

// Determine which layer of streets to display, a helper function to drawStreetSegments to reduce code redundancy
void plotStreets(int streetSegID, int pointId, int layerNum, ezgl::renderer *g);


// Miscellaneous Helper Functions
// Given a feature type, this function gives the renderer a specific color for the feature
void setFeatureColor(FeatureType featureType, ezgl:: renderer *g);

// This function returns the ratio between the original world and the current visible world (orig/curr)
double getZoomLevel(ezgl::renderer *g);

// This function converts a string (e.g "Motorway") to a way type (e.g WayType::MOTORWAY).
WayType stringToWayType(std::string type);

// Given a way type, this function gives the renderer a specific color for the way
void setRoadColor(WayType wayType, ezgl::renderer *g);

// This function returns the angle between a horizontal line and the line between the two points.
double calculateAngle(const ezgl::point2d point1, const ezgl::point2d point2);

// This function returns the average coordinate of two points.
ezgl::point2d getMidpoint(const ezgl::point2d& point1, const ezgl::point2d& point2);

POIIdx findClosestPOIID(LatLon my_position);

int hexToDecimalR(std::string hexString);
int hexToDecimalG(std::string hexString);
int hexToDecimalB(std::string hexString);


void drawScale(ezgl::renderer *g);

#endif