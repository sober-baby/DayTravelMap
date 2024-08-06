#include "m2HelperFunctions.h"
#include "m3HelperFunctions.h"
#include "m3.h"
#include "globals.h"
#include <set>
#include <string>
#include <fstream>
#include <unordered_set>
#include <curl/curl.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

/*
   This document contains all the function implementations that we use for M2. The function descriptions can be found in m2HelperFunctions.h
*/


// Access the existing global variables
extern GlobalVars mapData;


void draw_main_canvas(ezgl::renderer *g) {
   // Chrono time testing for drawing map
   auto start = std::chrono::high_resolution_clock::now();

   // Initialize dark mode to be false
   mapData.darkMode = false;

   // Draw the window
   g -> draw_rectangle({0, 0}, {1000, 1000});

   // Toggle to dark mode if switch state signal is received from input
   if (mapData.switchState) {      
      // Night Mode
      ezgl::rectangle visible_world = g->get_visible_world();
      g -> set_color(50, 50, 50);
      g -> fill_rectangle(visible_world);
      mapData.darkMode = true;
   } 

   // Draw all the relevant features


   
   drawFeatures(g);
   drawStreetSegments(g);
   drawRailWays(g);
   drawSubwayLines(g);
   drawPOIImage(g);
   if (mapData.bikeRoutes) {
     drawBikeRoutes(g);
   }

   if (mapData.busRoutes) {
     drawBusRoutes(g);
   }
   drawSubwayStations(g);
   drawIntersections(g);
   drawStreetNames(g);
   drawOneWayArrow(g);
   // printTravelDirections();
   drawScale(g);


   


   // drawPath(g, findPathBetweenIntersections(0, {500,600}));
   //drawPath(g, findPathBetweenIntersections(0, {777,88}));
   // printTravelDirections(findPathBetweenIntersections(0, {777,88}));

   // is this code logic right?
   drawPath(g, mapData.path, mapData.intersect_ids);

   // std::vector<IntersectionIdx> a = {1,3,5,7,9};
   // std::vector<IntersectionIdx> b = {1,3,5,7,8,9,10,11};
   // std::vector<IntersectionIdx> c = {1,2,3,4,5,6,7,9,11};
   // std::vector<IntersectionIdx> d = removeDuplicatesOfThree(a,b,c);
   // for(int i = 0; i < d.size(); i++){
   //    std::cout << d[i] << std::endl;
   // }


   
   // Chrono stop timer
   auto stop = std::chrono::high_resolution_clock::now();
   auto duration = std::chrono::duration_cast<std::chrono::microseconds> (stop - start);
   // std::cout << "Search Button: " << duration.count() << "us" << std::endl;
}


// Helper functions for converting coordinates
float x_from_lon(float lon) {
   return kEarthRadiusInMeters * lon * kDegreeToRadian * cos(mapData.latAvg * kDegreeToRadian);
}
float y_from_lat(float lat) {
   return kEarthRadiusInMeters * lat * kDegreeToRadian;
}
float lon_from_x(float x) {
   return x / kEarthRadiusInMeters / kDegreeToRadian / cos(mapData.latAvg * kDegreeToRadian);
}
float lat_from_y(float y) {
   return y / kEarthRadiusInMeters / kDegreeToRadian;
}


//Helper functions for calculating midpoint
ezgl::point2d getMidpoint(const ezgl::point2d& point1, const ezgl::point2d& point2) {
   //find the midpoint of x values
   double midX = (point1.x + point2.x) / 2.0;
   //find the midpoint of y vaules
   double midY = (point1.y + point2.y) / 2.0;
   //return the midpoint of two points
   return ezgl::point2d(midX, midY);
}
 
//Helper function to help with calculating the angle between two points
double calculateAngle(const ezgl::point2d point1, const ezgl::point2d point2) {

   double point1x = point1.x;
   double point2x = point2.x;
   double point1y = point1.y;
   double point2y = point2.y;

   if (point1.x < point2.x) {
      std::swap(point1x, point2x);
      std::swap(point1y, point2y);
   }
   //ge the difference between two points
   double deltaX = point2.x - point1.x;
   double deltaY = point2.y - point1.y;
   // if(deltaX < 0.1 && deltaX > -0.1){
   //    return 90;
   // }
   

   //determine the angle in radians between two points
   double angleRadians = std::atan2(deltaY, deltaX);
    //convert the angle to degrees
   double angleDegrees = angleRadians * (180.0 / M_PI);
   //check if the text is flipped and then flip it back
   // if (angleDegrees > 90 && angleDegrees < 270){
   //    return angleDegrees -= 180;
   // }
   //return the angless
   return angleDegrees;
}


// Returns the closest point of interest id given the position of the POI
POIIdx findClosestPOIID(LatLon my_position) {
   // Initialize Variables 
   POIIdx closestPOI = 0;
   int closestDistance = 0;
   bool foundPOI = false;

   // Get the number of points of interest
   int numOfPOIs = getNumPointsOfInterest();

   // Use a for loop to loop through each POI
   for(int i = 0; i < numOfPOIs; i++) {
      // If we found that POI then,
      if (!foundPOI) {
         // Use the findDistanceBetweenTwoPoints function to find the distance between
         // my_position and the POI, and assign this to closestDistance
         closestDistance = findDistanceBetweenTwoPoints(my_position, getPOIPosition(i));
         // closestPOI gets assigned the current index (also known as the POIIdx)
         closestPOI = i;
         // Then, we set foundPOI to true such that we won't enter this condition again
         foundPOI = true;
      }

      // The reason for the previous if statement is to assign an initial value to closestDistance,
      // such that, we can compare closestDistance with the next matching POI

      // Then, we check: Is the next POI that we found closer than the current closestDistance
      if (closestDistance > findDistanceBetweenTwoPoints(my_position, getPOIPosition(i))) {
         // If yes, then we'll update closestDistance and closestPOI with the new value
         closestDistance = findDistanceBetweenTwoPoints(my_position, getPOIPosition(i));
         closestPOI = i;
      } 

    }

    //return the closest POIIdx (which is an integer)
    return closestPOI;  
}

/*** Code that displays live information extracted from APIs ***/
size_t writeCallback(void *contents, size_t size, size_t nmemb, std::string *s) {
    size_t newLength = size * nmemb;
    try {
        s->append((char*)contents, newLength);
    } catch(std::bad_alloc &e) {
        // handle memory problem
        return 0;
    }
    return newLength;
}

// This service provides information about the speeds and travel times of the road fragment closest to the given coordinates. It is designed to work alongside the Flow
// Tiles to support clickable flow data visualizations. With this API, the client side can connect any place in the map with flow data on the closest road and present it to the user.

// Function to parse traffic data from XML and update the GUI elements accordingly
void parseTrafficData(ezgl::application* app, const std::string& xmlData, IntersectionIdx inter_id) {
    // Use a string stream to handle the XML data for parsing
    std::istringstream xmlDataStream(xmlData);
    boost::property_tree::ptree pt;

    try {
        // Parse the XML into the property tree
        boost::property_tree::read_xml(xmlDataStream, pt);

        // Get references to GUI elements that will display the traffic information
        GObject* trafficPopUp = app->get_object("LiveTrafficPopUp");
        GtkWidget* traffic = GTK_WIDGET(trafficPopUp);
        gtk_widget_show_all(traffic); // Show the traffic pop-up with all its widgets

        // Retrieve and cast GObject pointers to specific label widgets
        GObject* interName = app->get_object("intersectionName");
        GtkLabel* interNameLabel = GTK_LABEL(interName);        
        
        GObject* roadClosed = app->get_object("roadClosure");
        GtkLabel* roadClosedLabel = GTK_LABEL(roadClosed);

        GObject* currSpeed = app->get_object("currentSpeed");
        GtkLabel* currSpeedLabel = GTK_LABEL(currSpeed);

        GObject* actualSpeed = app->get_object("freeFlowSpeed");
        GtkLabel* actualSpeedLabel = GTK_LABEL(actualSpeed);

        GObject* currTravelTime = app->get_object("currentTravelTime");
        GtkLabel* currTravelTimeLabel = GTK_LABEL(currTravelTime);

        GObject* freeTravelTime = app->get_object("freeFlowTravelTime");
        GtkLabel* freeTravelTimeLabel = GTK_LABEL(freeTravelTime);
 
        // Create string streams to format text for display
        std::stringstream roadMsg;
        std::stringstream currSpeedMsg;
        std::stringstream actualSpeedMsg;
        std::stringstream currTravelTimeMsg;
        std::stringstream actualTravelTimeMsg;

        std::stringstream ss;
        ss << mapData.intersections[inter_id].name;
// std::cout << "Intersection: " << mapData.intersections[inter_id].name.c_str() << std::endl;
        gtk_label_set_text(interNameLabel, ss.str().c_str());

        // Access traffic data from the property tree
        bool roadClosure = pt.get<bool>("flowSegmentData.roadClosure");
        int currentSpeed = pt.get<int>("flowSegmentData.currentSpeed");
        int freeFlowSpeed = pt.get<int>("flowSegmentData.freeFlowSpeed");
        int currentTravelTime = pt.get<int>("flowSegmentData.currentTravelTime");
        int freeFlowTravelTime = pt.get<int>("flowSegmentData.freeFlowTravelTime");

        // Format the fetched traffic data
        currSpeedMsg << currentSpeed << " km/h";
        actualSpeedMsg << freeFlowSpeed << " km/h";
        currTravelTimeMsg << currentTravelTime << " sec";
        actualTravelTimeMsg << freeFlowTravelTime << " sec";

        // Update the road closure status
        if (roadClosure) {
            roadMsg << "Road is currently closed";
        } else {
            roadMsg << "Road is currently NOT closed";
        }

        // Set the formatted strings to the respective labels in the GUI
        gtk_label_set_text(roadClosedLabel, roadMsg.str().c_str());
        gtk_label_set_text(currSpeedLabel, currSpeedMsg.str().c_str());
        gtk_label_set_text(actualSpeedLabel, actualSpeedMsg.str().c_str());
        gtk_label_set_text(currTravelTimeLabel, currTravelTimeMsg.str().c_str());
        gtk_label_set_text(freeTravelTimeLabel, actualTravelTimeMsg.str().c_str());

    } catch (boost::property_tree::xml_parser::xml_parser_error& e) {
        // Handle XML parsing errors
        std::cerr << "Exception caught: " << e.what() << std::endl;
    } catch (boost::property_tree::ptree_bad_path& e) {
        // Handle errors in accessing the property tree
        std::cerr << "Bad path: " << e.what() << std::endl;
    } catch (boost::property_tree::ptree_error& e) {
        // Handle generic property tree errors
        std::cerr << "PTree error: " << e.what() << std::endl;
    }
}

// Fetches live traffic data 
std::string fetchTrafficData(const std::string& url) {
    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        res = curl_easy_perform(curl);
        if(res != CURLE_OK) {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
            return "";
        }
        curl_easy_cleanup(curl);
    }
   // read buffer contains the entire live traffic data
    return readBuffer;
}

// Function to display live weather information in the GUI application
void displayLiveWeatherInfo(ezgl::application* app, std::string city) {
    // Retrieve GUI elements by their identifiers to display weather data
    GObject* temperature = app->get_object("temperature");
    GtkLabel* tempLabel = GTK_LABEL(temperature); // Temperature label

    GObject* weather = app->get_object("weather");
    GtkLabel* weatherLabel = GTK_LABEL(weather); // Weather description label

    GObject* humidityObject = app->get_object("humidity");
    GtkLabel* humidityLabel = GTK_LABEL(humidityObject); // Humidity label

    // Prepare streams to format the data that will be displayed
    std::stringstream temperatureMsg;
    std::stringstream weatherMsg;
    std::stringstream humidityMsg;

    // Initialize CURL library for making HTTP requests
    CURLcode res = curl_global_init(CURL_GLOBAL_ALL);
    if (res != CURLE_OK) {
        std::cout << "ERROR: Unable to initialize libcurl" << std::endl;
        return;
    }

    // Create a CURL handle for the request
    CURL *curlHandle = curl_easy_init();
    if (!curlHandle) {
        std::cout << "ERROR: Unable to get easy handle" << std::endl;
        curl_global_cleanup();
        return;
    }

    // Construct the API URL using the city's name and a predefined API key
    std::string cityCountry = mapData.mapCityCountry[std::string(city)];
    std::string url = "https://api.openweathermap.org/data/2.5/weather?q=" + cityCountry + "&appid=f822b510e6a1a001cbabc0a2a599a4b3";

    std::string readBuffer;
    // Configure CURL for the HTTP request
    curl_easy_setopt(curlHandle, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curlHandle, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curlHandle, CURLOPT_WRITEDATA, &readBuffer);
    
    // Perform the HTTP GET request
    res = curl_easy_perform(curlHandle);

    if (res != CURLE_OK) {
        std::cout << "ERROR: " << curl_easy_strerror(res) << std::endl;
    } else {
        std::cout << "Weather data fetched successfully!" << std::endl;
        try {
            // Parse the JSON response from the API
            auto json = nlohmann::json::parse(readBuffer);

            // Extract weather data from JSON
            double temperatureInKelvin = json["main"]["temp"];
            double temperatureInCelsius = temperatureInKelvin - 273.15;
            std::string weatherDescription = json["weather"][0]["description"];
            int humidity = json["main"]["humidity"];

            // Format the data to display
            temperatureMsg << temperatureInCelsius << std::endl;
            weatherMsg << weatherDescription << std::endl;
            humidityMsg << humidity << std::endl;

            // Update the GUI labels with the fetched weather data
            gtk_label_set_text(tempLabel, temperatureMsg.str().c_str());
            gtk_label_set_text(weatherLabel, weatherMsg.str().c_str());
            gtk_label_set_text(humidityLabel, humidityMsg.str().c_str());
        } catch(nlohmann::json::parse_error &e) {
            std::cout << "Error parsing JSON!" << std::endl;
        }
    }

    // Clean up CURL resources
    curl_easy_cleanup(curlHandle);
    curl_global_cleanup();
}

// Function that gives out a command/event whenever the user clicks the mouse
void act_on_mouse_click(ezgl::application* app, GdkEventButton* /*event*/, double x, double y) {
   auto start = std::chrono::high_resolution_clock::now();

   // Get current mouse click position
   // std::cout << "Mouse clicked at (" << x << "," << y << ")\n";
   LatLon pos = LatLon(lat_from_y(y), lon_from_x(x));

   // Find closest intersection 
   int inter_id = findClosestIntersection(pos);
   // std::cout << "Closest Intersection: " << mapData.intersections[inter_id].name << "\n";

   // Find the distance between the mouse click and the intersection
   LatLon inter_pos = getIntersectionPosition(inter_id); // Gets the intersection location in LatLon
   double distanceBetweenMouseAndInter = findDistanceBetweenTwoPoints(pos, inter_pos);

   // Get closest POI's ID
   int poi_id = findClosestPOIID(pos);

   // Find the distance between the mouse click and the POI
   LatLon poi_pos = getPOIPosition(poi_id); // Gets the POI location in LatLon
   double distanceBetweenMouseAndPOI = findDistanceBetweenTwoPoints(pos, poi_pos);

   if (!mapData.choosingIntersection && !mapData.clickingIntersection) {
      // mapData.intersections[mapData.mouseClickedInter[0]].highlight = false;

      // Compare the two distances 
      if (distanceBetweenMouseAndInter >= distanceBetweenMouseAndPOI) {
         // Then POI is closer to the mouse
         
         // Turn off the highlight state of any previously highlighted intersections
         // for (int prevIntersectionID : mapData.previousHighlightedIntersectionID) {
         //    if (prevIntersectionID >= 0) { // Check if a valid ID
         //          mapData.intersections[prevIntersectionID].highlight = false;
         //    }
         // }
         // Clear the previousHighlightedIntersectionID vector
         mapData.previousHighlightedIntersectionID.clear();

         // Display the POI information in a pop up message
         std::string nameOfPOI = getPOIName(poi_id);
         std::string typeOfPOI = getPOIType(poi_id);

         // Concatenating the name and type with a separator
         std::string messageContent = nameOfPOI + " - " + typeOfPOI;

         // Create a pop up message displaying the closest POI's information
         app->create_popup_message("Closest POI", messageContent.c_str());
      } else {
         // Intersection is closer to the POI
         // First, check if this intersection is already highlighted
         if (std::find(mapData.previousHighlightedIntersectionID.begin(), mapData.previousHighlightedIntersectionID.end(), inter_id) == mapData.previousHighlightedIntersectionID.end()) {
            // This intersection is not in the previously highlighted list, so clear old highlights
            for (int prevIntersectionID : mapData.previousHighlightedIntersectionID) {
                  if (prevIntersectionID >= 0 && mapData.selectedIntersection.size() == 2) { // Check if a valid ID
                     mapData.intersections[prevIntersectionID].highlight = false;
                  }
            }

            // Add the intersection ID that was retrieved by the mouse click
            mapData.mouseClickedInter.push_back(inter_id);
            
            // Ensure only one intersection is retrieved
            if(mapData.mouseClickedInter.size() == 1) {
               // Check if there's any previous intersections used to find path
               if(mapData.selectedIntersection.size() > 1 && mapData.selectedIntersection[1] != -1) {
                  mapData.intersections[mapData.selectedIntersection[0]].highlight = false;
                  mapData.intersections[mapData.selectedIntersection[1]].highlight = false;         
                  mapData.selectedIntersection.clear();
                  mapData.selectedIntersection.resize(0);
               }
               // Get the window popup
               GObject* confirmPopUp = app->get_object("ConfirmationPopUp");
               GtkWidget* confirm = GTK_WIDGET(confirmPopUp);
               gtk_widget_show_all(confirm);
            }

            // Gets the LatLon of the intersection points
            double lonOfInter = inter_pos.longitude();
            double latOfInter = inter_pos.latitude();

            // Put it into a string
            std::ostringstream stream;
            stream << latOfInter << "," << lonOfInter;
            std::string coordinates = stream.str();
            
            // Concat to the url at the end
            std::string url = "https://api.tomtom.com/traffic/services/4/flowSegmentData/absolute/10/xml?key=OTTlm5RflAhG1ib2EB0fObpzjWVmUn3A&point=" + coordinates;
            // Fetch the Live Traffic Data using the url
            std::string xmlData = fetchTrafficData(url);

            if (!xmlData.empty()) {
               // Parses the Live Traffic Data
               parseTrafficData(app, xmlData, inter_id);
            } else {
               std::cerr << "Failed to fetch or parse traffic data." << std::endl;
            }

            // Highlight the new intersection
            mapData.intersections[inter_id].highlight = true;

            // Clear the vector since we're going to add a new entry
            mapData.previousHighlightedIntersectionID.clear();

            // Add this intersection ID to the tracking vector
            mapData.previousHighlightedIntersectionID.push_back(inter_id);
         }
         else {
            // If it's the same intersection, just ensure it remains highlighted
            mapData.intersections[inter_id].highlight = true;
         }

         // Print information to terminal
         // std::stringstream ss;
         // ss << "Intersection: " << mapData.intersections[inter_id].name;

         // Use .c_str() to convert the std::string returned by ss.str() to const char*
         // app->create_popup_message("Intersection Found!", ss.str().c_str());
      }
   }

   /***** Clicking an intersection for multiple intersection *****/
   // Using the closest intersection logic:
   else if (mapData.choosingIntersection && !mapData.clickingIntersection){
      for(IntersectionIdx actualInter : mapData.checkingID) {
         if(inter_id == actualInter && mapData.mouseClickedInter.size() == 0) {
            // Allow the users to click the intersection from multiple intersections
            mapData.mouseClickedInter.push_back(inter_id);
         }
      }      
      // Ensure only one intersection is retrieved
      if(mapData.mouseClickedInter.size() == 1 && inter_id == mapData.mouseClickedInter[0]) {
         // Check if there's any previous intersections used to find path
         if(mapData.selectedIntersection.size() > 1 && mapData.selectedIntersection[1] != -1) {
            mapData.intersections[mapData.selectedIntersection[0]].highlight = false;
            mapData.intersections[mapData.selectedIntersection[1]].highlight = false;         
            mapData.selectedIntersection.clear();
            mapData.selectedIntersection.resize(0);
         }
         
         // Get the window popup
         GObject* confirmPopUp = app->get_object("ConfirmationPopUp");
         GtkWidget* confirm = GTK_WIDGET(confirmPopUp);
         gtk_widget_show_all(confirm);
      }
      
      for(IntersectionIdx intersection : mapData.mouseClickedInter) {
         std::cout << "Intersection:" << intersection << std::endl;
      }
   } 

   app->refresh_drawing(); // Refresh the drawing to update the highlight

   // Force a redraw
   auto stop = std::chrono::high_resolution_clock::now();
   auto duration = std::chrono::duration_cast<std::chrono::microseconds> (stop - start);
   std::cout << "Search Button: " << duration.count() << "us" << std::endl;
   app -> refresh_drawing(); // force a redraw after clicking
}

/*********** CODE FOR MAIN.UI ***********/
// This function is used for CSS Styling
void load_css(){
    GtkCssProvider* css = gtk_css_provider_new();
    gtk_css_provider_load_from_path(css, "libstreetmap/src/style.css", NULL);

    GdkDisplay *display = gdk_display_get_default();
    GdkScreen *screen = gdk_display_get_default_screen(display);
    gtk_style_context_add_provider_for_screen(screen, GTK_STYLE_PROVIDER(css), GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    
    g_object_unref(css);
}

// Displays pop up information for a guide so users can learn the interface
void helpButton(GtkWidget* /*widget*/, ezgl::application* application) {
   // Get the window popup
   GObject* helpPopUp = application->get_object("helpPopUp");
   GtkWidget* help = GTK_WIDGET(helpPopUp);
   gtk_widget_show_all(help);
}

// Searches if there is a intersection between two streets
void searchButton(GtkWidget* /*widget*/, ezgl::application* application) {
   if(mapData.typingIntersection) {
      // auto start = std::chrono::high_resolution_clock::now();

      // Hide the Pop Up Window
      GObject* windowPopUp = application->get_object("popUpSearchWindow");
      GtkWidget* window = GTK_WIDGET(windowPopUp);
      gtk_widget_hide(window);

      // Toggle between the states
      GObject* gtk_object_1 = application->get_object("Text1");
      GObject* gtk_object_2 = application->get_object("Text2");

      // Get gtk_object with ID "Text1" and "Text2"
      GtkEntry* gtk_entry_1 = GTK_ENTRY(gtk_object_1);
      GtkEntry* gtk_entry_2 = GTK_ENTRY(gtk_object_2);
      const gchar* text1 = gtk_entry_get_text(gtk_entry_1);
      const gchar* text2 = gtk_entry_get_text(gtk_entry_2);

      // Check if search bar is empty
      if ((std::string)text1 == "" && (std::string)text2 == "") {
         // Both Search Bars are empty
         application->create_popup_message("Warning!", "Both Search Bars are empty. Please re-enter Street 1 & 2.");
         return;
      } else if((std::string)text1 == "") {
         // Search Bar 1 is empty
         application->create_popup_message("Warning!", "Search Bar 1 is empty. Please re-enter Street 1.");
         return;
      } else if((std::string)text2 == "") {
         // Search Bar 2 is empty
         application->create_popup_message("Warning!", "Search Bar 2 is empty. Please re-enter Street 2.");
         return;
      }

      // Populate the global vector Street Names with the current city's street name
      for(int i = 0; i < getNumStreets(); i++) {
         mapData.globalStreetNames.push_back(getStreetName(i));
      }

      // Check if streets input exist or not by looping through Street Names Vector
      bool text1Found = false, text2Found = false;
      for(const auto& streetName : mapData.globalStreetNames) {
         // Check if both street name exists
         if((std::string)text1 == streetName) {
            text1Found = true;
         }
         if((std::string)text2 == streetName) {
            text2Found = true;
         }
         if(text1Found && text2Found) {
            break; // If both are found, no need to continue the loop
         }
      }

      // Checks which streets does not exist
      if (!text1Found && !text2Found) {
         // Both streets does not exist
         application->create_popup_message("Warning!", "Both streets does not exist. Please re-enter Street 1 & 2.");
         return;
      } else if(!text1Found) {
         // Street 1 does not exist
         application->create_popup_message("Warning!", "Street 1 does not exist. Please re-enter Street 1.");
         return;
      } else if (!text2Found) {
         // Street 2 does not exist
         application->create_popup_message("Warning!", "Street 2 does not exist. Please re-enter Street 2.");
         return;      
      }

      // Get the two streets
      // Given the street names, get their Street ID
      std::vector<StreetIdx> streetId1 = findStreetIdsFromPartialStreetName(text1);      
      std::vector<StreetIdx> streetId2 = findStreetIdsFromPartialStreetName(text2);

      // Have all intersections between those street be shown graphically
      std::vector<IntersectionIdx> inter_id; // To accumulate all intersections

      inter_id.clear();
      inter_id = {};
      for (StreetIdx streetIDS1 : streetId1) {
         for (StreetIdx streetIDS2 : streetId2) {
            std::vector<IntersectionIdx> all_inter_ids = findIntersectionsOfTwoStreets(std::make_pair(streetIDS1, streetIDS2));
            // Add each element of inter_id to all_inter_ids using push_back
            for (IntersectionIdx id : all_inter_ids) {
               inter_id.push_back(id);
            }
         }
      }

      // Remove the previous highlight by looping through all of the previous highlighted intersections
      for (int prevIntersectionID : mapData.previousHighlightedIntersectionID) {
         mapData.intersections[prevIntersectionID].highlight = false;
      }
      application->refresh_drawing(); // Refresh the drawing to update the highlight

      // Clear the global variables
      mapData.previousHighlightedIntersectionID.clear();
      mapData.previousHighlightedIntersectionID = {};

      // Check if there are intersections between the two streets
      if (inter_id.empty()) {
         // No intersections found
         application->create_popup_message("Sorry!", "No intersections found between the two streets.");
      } else {

         /**** autozoom into that intersection ****/
         std::string main_canvas_id = application->get_main_canvas_id();
         auto canvas = application->get_canvas(main_canvas_id);

         // Amount we want to zoom into the screen
         double window_size = 250.0;

         // Get the location of the intersection
         ezgl::point2d intersection_location = mapData.intersections[inter_id.front()].xy_loc - ezgl::point2d{window_size/2.0, window_size/2.0};
         ezgl::rectangle new_world = ezgl::rectangle(intersection_location, window_size, window_size);

         // Set the canvas to zoom in to that intersection
         canvas->get_camera().set_world(new_world); 

         std::string intersectionName = mapData.intersections[inter_id.front()].name;
         std::cout << intersectionName << std::endl;
         // application->create_popup_message("Intersections Between The Two Streets", intersectionName.c_str());
         
         for (IntersectionIdx intersection : inter_id) {
            // Highlight the Closest Intersection graphically
            mapData.intersections[intersection].highlight = true; 
            mapData.previousHighlightedIntersectionID.push_back(intersection);
         } 
         
         application->refresh_drawing(); // Refresh the drawing to update the highlight    
         if(inter_id.size() == 1) {
            // Remove all of the previous intersections used for drawing the path
            if(mapData.selectedIntersection.size() > 1 && mapData.selectedIntersection[1] != -1) {
               mapData.intersections[mapData.selectedIntersection[0]].highlight = false;
               mapData.intersections[mapData.selectedIntersection[1]].highlight = false;         
               mapData.selectedIntersection.clear();
               mapData.selectedIntersection.resize(0);
            }
            // Only one intersection so store this intersection into global vars
            // This is the global variable
            mapData.selectedIntersection.push_back(inter_id.front());

            for (IntersectionIdx intersection : mapData.selectedIntersection) {
               // Highlight the Closest Intersection graphically
               mapData.intersections[intersection].highlight = true; 
               mapData.previousHighlightedIntersectionID.push_back(intersection);
            } 

            for (IntersectionIdx id : mapData.selectedIntersection) {
               std::cout << "Intersections: " << id << std::endl;
            }
            // Check if we have collected two intersections
            if (mapData.selectedIntersection.size() == 2) {
               // Get the Point2D Location of both intersections
               ezgl::point2d location1 = mapData.intersections[mapData.selectedIntersection[0]].xy_loc;
               ezgl::point2d location2 = mapData.intersections[mapData.selectedIntersection[1]].xy_loc;

               ezgl::point2d avgLocationOfInter;
               avgLocationOfInter.x = (location1.x + location2.x)/2;
               avgLocationOfInter.y = (location1.y + location2.y)/2;

               double zoom_level = 1000;

               // Get the location of the intersection
               ezgl::point2d path_location = avgLocationOfInter - ezgl::point2d{zoom_level/2.0, zoom_level/2.0};
               new_world = ezgl::rectangle(path_location, zoom_level, zoom_level);

               // Set the canvas to zoom in to that intersection
               canvas->get_camera().set_world(new_world); 

               // Convert the first two elements of the vector to a pair
               mapData.intersect_ids = std::make_pair(mapData.selectedIntersection[0], mapData.selectedIntersection[1]);

               double turn_penalty = 15.0;
               // Call the function to find the path between intersections
               mapData.path = findPathBetweenIntersections(turn_penalty, mapData.intersect_ids);

               application->refresh_drawing(); // Refresh drawing

               for(StreetSegmentIdx all_segments : mapData.path) {
                  std::cout << "Street Segments ID: " << all_segments << std::endl;
               }

               // Call print travel directions to display the directions
               printTravelDirections(application);

               mapData.selectedIntersection.clear();
               mapData.selectedIntersection.resize(0);
            }
         } else {
            // This means that there's multiple intersections
         
            // Ask user to pick one intersection
            application->create_popup_message("Multiple Intersections Found", "Please click on one of the intersections as your starting point");
            mapData.choosingIntersection = true;
            for(IntersectionIdx check : inter_id) {
               mapData.checkingID.push_back(check);
            }

         } 
      }
      
      return;
   } else {
      application->create_popup_message("Warning!", "Currently in Clicking Intersection Mode");
   }
}

void displayBusRoutes(GtkWidget* /*widget*/, ezgl::application* application) {
   mapData.busRoutes = !mapData.busRoutes;  // Toggle the state of mapData.bikeRoutes
   application->refresh_drawing();            // Refresh the map to reflect the change
   application->update_message("Bus Routes Displayed");
}

void displayBikeRoutes(GtkWidget* /*widget*/, ezgl::application* application) {
   mapData.bikeRoutes = !mapData.bikeRoutes;  // Toggle the state of mapData.bikeRoutes
   application->refresh_drawing();            // Refresh the map to reflect the change
   application->update_message("Bike Routes Displayed");
}

void displayLiveWeather(GtkWidget* /*widget*/, ezgl::application* application) {
   GObject* newCity = application->get_object("MapNames");
   GtkComboBoxText* comboBox = (GtkComboBoxText*)newCity;
   const gchar* city = gtk_combo_box_text_get_active_text(comboBox);

   if (city == NULL || strcmp(city, "") == 0) {
      displayLiveWeatherInfo(application, "toronto_canada");
   } else {
      displayLiveWeatherInfo(application, city);
   }

   // Get the window popup
   GObject* weatherPopUp = application->get_object("weatherPopUp");
   GtkWidget* weather = GTK_WIDGET(weatherPopUp);
   gtk_widget_show_all(weather);
}

// Close direction pop up when clicked
void closedButton(GtkWidget* /*widget*/, ezgl::application* application) {
   GObject* DirectionsWindow = application->get_object("DirectionsWindow");
   GtkWidget* displayDirectionWindow = GTK_WIDGET(DirectionsWindow);
   gtk_widget_hide(displayDirectionWindow);
}

// Close Help Pop Up when clicked
void closedHelpButton(GtkWidget* /*widget*/, ezgl::application* application) {
   GObject* helpWindow = application->get_object("helpPopUp");
   GtkWidget* closeHelpWindow = GTK_WIDGET(helpWindow);
   gtk_widget_hide(closeHelpWindow);
}

// Close Live Weather Pop Up When Clicked
void closedWeatherButton(GtkWidget* /*widget*/, ezgl::application* application) {
   GObject* weatherWindow = application->get_object("weatherPopUp");
   GtkWidget* closeWeatherWindow = GTK_WIDGET(weatherWindow);
   gtk_widget_hide(closeWeatherWindow);
}

void closeTrafficButton(GtkWidget* /*widget*/, ezgl::application* application) {
   GObject* trafficWindow = application->get_object("LiveTrafficPopUp");
   GtkWidget* closeTrafficWindow = GTK_WIDGET(trafficWindow);
   gtk_widget_hide(closeTrafficWindow);
}

void closeConfirmInter(GtkWidget* /*widget*/, ezgl::application* application) {
   GObject* confirmIntersection = application->get_object("ConfirmationPopUp");
   GtkWidget* closeInterWindow = GTK_WIDGET(confirmIntersection);
   gtk_widget_hide(closeInterWindow);
   mapData.intersections[mapData.mouseClickedInter[0]].highlight = false;
   application->refresh_drawing();
   mapData.mouseClickedInter.clear();
}

// Called when the confirm button is clicked
void confirmButton(GtkWidget* /*widget*/, ezgl::application* application) {
   // Hide the pop up window
   GObject* confirmPopUp = application->get_object("ConfirmationPopUp");
   GtkWidget* confirm = GTK_WIDGET(confirmPopUp);
   gtk_widget_hide(confirm);
   
   // Unhighlight all other intersections from a multiple intersections 
   if(mapData.choosingIntersection) {
      for(IntersectionIdx actualInter : mapData.checkingID) {
         mapData.intersections[actualInter].highlight = false;
      }  
      // Ensure the first intersection is highlighted
      mapData.intersections[mapData.mouseClickedInter[0]].highlight = true;
      application->refresh_drawing();
      // Change the choosing an intersection from multiple intersection to false
      mapData.choosingIntersection = false;
   } 

   // Check if selected intersection is still <= 2
   if(mapData.selectedIntersection.size() <= 2) {
      // If yes, add to selected intersection with the clicked intersection
      mapData.selectedIntersection.push_back(mapData.mouseClickedInter[0]);
      if(mapData.selectedIntersection.size() == 2) {
         // ensure the two intersections are highlighted
         mapData.intersections[mapData.selectedIntersection[0]].highlight = true;
      }
   }

   // Clear the vector
   mapData.mouseClickedInter.clear();

   for(IntersectionIdx temp : mapData.selectedIntersection) {
      std::cout << "Intersection: " << temp << std::endl;
   }

   if (mapData.selectedIntersection.size() == 2) {
      // Get the canvas
      std::string main_canvas_id = application->get_main_canvas_id();
      auto canvas = application->get_canvas(main_canvas_id);
      
      // Get the location of the selected intersections
      ezgl::point2d location1 = mapData.intersections[mapData.selectedIntersection[0]].xy_loc;
      ezgl::point2d location2 = mapData.intersections[mapData.selectedIntersection[1]].xy_loc;

      // Find the average of the x and y of the two points
      ezgl::point2d avgLocationOfInter;
      avgLocationOfInter.x = (location1.x + location2.x)/2;
      avgLocationOfInter.y = (location1.y + location2.y)/2;

      double zoom_level = 1000;

      ezgl::point2d path_location = avgLocationOfInter - ezgl::point2d{zoom_level/2.0, zoom_level/2.0};
      ezgl::rectangle new_world = ezgl::rectangle(path_location, zoom_level, zoom_level);

      // Set the canvas to zoom in to that intersection
      canvas->get_camera().set_world(new_world); 

      // Convert the first two elements of the vector to a pair
      mapData.intersect_ids = std::make_pair(mapData.selectedIntersection[0], mapData.selectedIntersection[1]);

      std::cout << "First Inter_ID: " << mapData.intersect_ids.first << std::endl;
      std::cout << "Second Inter_ID: " << mapData.intersect_ids.second << std::endl;

      double turn_penalty = 15.0;
      // Call the function to find the path between intersections
      mapData.path = findPathBetweenIntersections(turn_penalty, mapData.intersect_ids);

      application->refresh_drawing(); // Refresh the app drawing

      // Display the travel directions for the path to the GUI
      printTravelDirections(application);

      for(StreetSegmentIdx all_segments : mapData.path) {
         std::cout << "Street Segments ID: " << all_segments << std::endl;
      }
   }   
}

// Function for loading a new city
void loadNewCity(GtkWidget* /*widget*/, ezgl::application* application){
   // Get the Combo Box Text Object
   GObject* newCity = application->get_object("MapNames");
   GtkComboBoxText* comboBox = (GtkComboBoxText*)newCity;
   const gchar* city = gtk_combo_box_text_get_active_text(comboBox);

   // Call close map to close current map
   closeMap();

   // Base path where the map files are located
   std::string fullPath = "/cad2/ece297s/public/maps/" + (std::string)city + ".streets.bin"; // Concatenate to form the full path

   // Clear the current contents
   mapData.globalStreetNames.clear();
   
   // Load up new map
   loadMap(fullPath);

   // Populate the global street names vector with names from the new map
   for(int i = 0; i < getNumStreets(); i++) {
      mapData.globalStreetNames.push_back(getStreetName(i));
   }
   
   // Get the GTK ListStore
   GtkListStore* liststore1 = (GtkListStore*)application->get_object("liststore1");   
   GtkListStore* liststore2 = (GtkListStore*)application->get_object("liststore2");

   // We clear the previous liststore so that we can update the liststore with the new street names
   gtk_list_store_clear(liststore1);
   gtk_list_store_clear(liststore2);

   GtkTreeIter iter;

   // Populate all the list store with the new city's street names
   for(const auto& streetName : mapData.globalStreetNames) {
      gtk_list_store_append(liststore1, &iter);
      gtk_list_store_set(liststore1, &iter, 0, streetName.c_str(), -1);
      gtk_list_store_append(liststore2, &iter);
      gtk_list_store_set(liststore2, &iter, 0, streetName.c_str(), -1);
   }
   
   // Change canvas world coordinates
   application->change_canvas_world_coordinates("MainCanvas", ezgl::rectangle ({x_from_lon(mapData.min_lon), y_from_lat(mapData.min_lat)},
                                {x_from_lon(mapData.max_lon), y_from_lat(mapData.max_lat)}));
   
   // Refresh the map drawing
   application->refresh_drawing();
}

//function to change the user interface to night mode
void nightMode(GtkSwitch* /*self*/, gboolean state, ezgl::application* app){
   // If state is true, so night mode is turned on
   if(state) {
      // Set global variable switchState to true
      mapData.switchState = !mapData.switchState;
      // Refresh the map so that the map changes colour
      app->refresh_drawing();
      app->update_message("Night mode turned on");
   } else {
      // Set global variable switchState to true
      mapData.switchState = !mapData.switchState;
      // Refresh the map so that the map changes colour
      app->refresh_drawing();
      app->update_message("Night mode turned off");
   }
}

void cancelSearch(GtkWidget* /*widget*/, ezgl::application* application) {
   // Get the window popup
   GObject* windowPopUp = application->get_object("popUpSearchWindow");
   GtkWidget* window = GTK_WIDGET(windowPopUp);
   gtk_widget_hide(window);
}

void popUpSearch(GtkWidget* /*widget*/, ezgl::application* application) {
   mapData.typingIntersection = true;
   GObject* text1 = application->get_object("Text1");
   GObject* text2 = application->get_object("Text2");

   // Gets the GTK Entry text and set it to empty to reset it
   GtkEntry* gtk_entry_1 = GTK_ENTRY(text1);
   gtk_entry_set_text(gtk_entry_1, "");
   GtkEntry* gtk_entry_2 = GTK_ENTRY(text2);
   gtk_entry_set_text(gtk_entry_2, "");

   // Get the window popup
   GObject* windowPopUp = application->get_object("popUpSearchWindow");
   GtkWidget* window = GTK_WIDGET(windowPopUp);
   
   // Get the GTK List Store (this stores all the street names)
   GtkListStore* liststore1 = (GtkListStore*)application->get_object("liststore1");   
   GtkListStore* liststore2 = (GtkListStore*)application->get_object("liststore2");

   // Clearing the GtkListStore before appending new data
   gtk_list_store_clear(liststore1);
   gtk_list_store_clear(liststore2);

   GtkTreeIter Iter;

   // Loop through all the city's street names and populate the list store
   for(int i = 0; i< getNumStreets(); i++) {
      gtk_list_store_append(liststore1, &Iter);
      gtk_list_store_set(liststore1, &Iter, 0, getStreetName(i).c_str(), -1);
      gtk_list_store_append(liststore2, &Iter);
      gtk_list_store_set(liststore2, &Iter, 0, getStreetName(i).c_str(), -1);
   }

   gtk_widget_show_all(window);
}

//function to setup the application/map itself
void initial_setup(ezgl::application* application, bool /*new_window*/) {   
   // Gets the GTK Object aka the switch called "switch"
   GObject* buttonNightMode = application->get_object("switch");
   // Gets the GTK Object aka the Combo Box Text called "MapNames"
   GObject* newCity = application->get_object("MapNames");   
   // Gets the GTK Object aka the button called "HelpButton"
   GtkWidget* HelpButton = GTK_WIDGET(application->get_object("HelpButton"));
   // Gets the GTK Object aka the search button 
   GObject* popUpSearchButton = application->get_object("popUpSearch");
   // Gets the GTK Object aka the cancel button
   GObject* cancelButton = application->get_object("CancelButton");
   // Gets the GTK Object called "SearchBar"
   GObject* searchBar = application->get_object("SearchBar");
   // Gets the GTK Object called "Confirm"
   GObject* confirm = application->get_object("Confirm");
   // Gets the GTK Object called "displayBike"
   GObject* displayBikeRoute = application->get_object("displayBike");
   // Gets the GTK Object called "displayBus"
   GObject* displayBusRoute = application->get_object("displayBus");
   // Gets the GTK Object called "displayWeather"
   GObject* displayWeather = application->get_object("displayWeather");

   // Gets the comboBoxText that stores all the cities
   GtkComboBoxText* comboBox = GTK_COMBO_BOX_TEXT(newCity);

   // Loop through the names, appending the full path for each
   for (const std::string& name : mapData.mapNames) {
      gtk_combo_box_text_append_text(comboBox, name.c_str());
   }

   // Signal for displaying the bus routes
   g_signal_connect(displayWeather, // pointer to the UI widget
                  "clicked", // Signal state of button being changed
                  G_CALLBACK(displayLiveWeather), // name of callback function
                  application); // passing an application ptr.

   // Signal for displaying the bus routes
   g_signal_connect(displayBusRoute, // pointer to the UI widget
                  "clicked", // Signal state of button being changed
                  G_CALLBACK(displayBusRoutes), // name of callback function
                  application); // passing an application ptr.

   // Signal for displaying the bike routes
   g_signal_connect(displayBikeRoute, // pointer to the UI widget
                  "clicked", // Signal state of button being changed
                  G_CALLBACK(displayBikeRoutes), // name of callback function
                  application); // passing an application ptr.
   
   // Signal for changing the city
   g_signal_connect(newCity, // pointer to the UI widget
                  "changed", // Signal state of combo box text being changed
                  G_CALLBACK(loadNewCity), // name of callback function
                  application); // passing an application ptr.

   // Signal for night mode
   g_signal_connect(buttonNightMode, // pointer to the UI widget
                     "state-set", // Signal state of switch being changed
                     G_CALLBACK(nightMode), // name of callback function
                     application); // passing an application ptr.

   // Signal for help button
   g_signal_connect(HelpButton, // pointer to the UI widget
                     "clicked", // Signal state of button being changed
                     G_CALLBACK(helpButton), // name of callback function
                     application); // passing an application ptr.

   // Signal for creating the pop up button
   g_signal_connect(popUpSearchButton, // pointer to the UI widget
                  "clicked", // Signal state of button being clicked
                  G_CALLBACK(popUpSearch), // name of callback function
                  application); // passing an application ptr.

   // Signal for search bar
   g_signal_connect(searchBar, // pointer to the UI widget
                     "clicked", // Signal search button being clicked
                     G_CALLBACK(searchButton), // name of callback function
                     application); // passing an application ptr.

   // Signal for cancel button
   g_signal_connect(cancelButton, // pointer to the UI widget
                  "clicked", // Signal search button being clicked
                  G_CALLBACK(cancelSearch), // name of callback function
                  application); // passing an application ptr.

   // Signal for confirm button for search bar
   g_signal_connect(confirm, // pointer to the UI widget
                  "clicked", // Signal search button being clicked
                  G_CALLBACK(confirmButton), // name of callback function
                  application); // passing an application ptr.
     

   GObject* closeButton = application->get_object("closeButton");
   // Signal for cancel mouse 
   g_signal_connect(closeButton, // pointer to the UI widget
                  "clicked", // Signal search button being clicked
                  G_CALLBACK(closedButton), // name of callback function
                  application); // passing an application ptr 

   GObject* closeButton1 = application->get_object("closeButton1");
   // Signal for cancel mouse 
   g_signal_connect(closeButton1, // pointer to the UI widget
                  "clicked", // Signal search button being clicked
                  G_CALLBACK(closedHelpButton), // name of callback function
                  application); // passing an application ptr  

   GObject* closeButton2 = application->get_object("closeButton2");
   // Signal for cancel mouse 
   g_signal_connect(closeButton2, // pointer to the UI widget
                  "clicked", // Signal search button being clicked
                  G_CALLBACK(closedWeatherButton), // name of callback function
                  application); // passing an application ptr  

   GObject* closeButton3 = application->get_object("closeButton3");
   // Signal for cancel mouse 
   g_signal_connect(closeButton3, // pointer to the UI widget
                  "clicked", // Signal search button being clicked
                  G_CALLBACK(closeTrafficButton), // name of callback function
                  application); // passing an application ptr  

   GObject* closeButton4 = application->get_object("closeButton4");
   // Signal for cancel mouse 
   g_signal_connect(closeButton4, // pointer to the UI widget
                  "clicked", // Signal search button being clicked
                  G_CALLBACK(closeConfirmInter), // name of callback function
                  application); // passing an application ptr  

   load_css(); // load in the CSS Styling
}

/*********** CODE FOR LOADING DATA STRUCTURES ***********/

//code to load the intersection data
void loadIntersectionData() {
   // Give a value to initial max/min lat/lon  
   mapData.max_lat = getIntersectionPosition(0).latitude(); 
   mapData.min_lat = mapData.max_lat;
   mapData.max_lon = getIntersectionPosition(0).longitude();
   mapData.min_lon = mapData.max_lon;


   // Allocate space for the global variable that contains intersection data
   mapData.intersections.resize(getNumIntersections());
   
   // Loop through all intersections and compute map boundaries
   for (int id = 0; id < getNumIntersections(); ++id) {  
      LatLon currentPosition = getIntersectionPosition(id);
      double currentLat = currentPosition.latitude();
      double currentLon = currentPosition.longitude();

      // Find Map Boundaries, if find a value that is larger or smaller, replace max/min respectively
      mapData.max_lat = std::max(mapData.max_lat, currentLat);
      mapData.min_lat = std::min(mapData.min_lat, currentLat);
      mapData.max_lon = std::max(mapData.max_lon, currentLon);
      mapData.min_lon = std::min(mapData.min_lon, currentLon);
   }

   // Get average latitude from the max/min lat
   mapData.latAvg = (mapData.max_lat + mapData.min_lat) / 2;
   
   // Loop through all the intersections and load its position in the global variable as ezgl::point2d
   for (int id = 0; id < getNumIntersections(); ++id) {  
      LatLon currentPos = getIntersectionPosition(id);
      mapData.intersections[id].xy_loc.x = x_from_lon(currentPos.longitude());
      mapData.intersections[id].xy_loc.y = y_from_lat(currentPos.latitude());
      mapData.intersections[id].name = getIntersectionName(id);
   }
}


//code to load data for each street segnment. icluding midpoints and angle
void loadStreetSegmentData() {
   // Allocate space for street segments
   mapData.street_segments.resize(getNumStreetSegments());

   // Loop through all the street segments and store relevant information
   for (int id = 0; id < getNumStreetSegments(); ++id) {
      // Retrieve info from the API
      StreetSegmentInfo segInfo = getStreetSegmentInfo(id);
      mapData.street_segments[id].from = segInfo.from;
      mapData.street_segments[id].to = segInfo.to;
      // Load numCurvePoints into our data structure
      int numberCurvePoints = segInfo.numCurvePoints;
      mapData.street_segments[id].numberOfCurvePoints = numberCurvePoints;
      
      // Get the first point (street segment from)
      LatLon latlonfrom = getIntersectionPosition(segInfo.from);
      ezgl::point2d from = ezgl::point2d { x_from_lon(latlonfrom.longitude()), y_from_lat(latlonfrom.latitude()) };
      mapData.street_segments[id].all_curve_points.push_back(from);
      ezgl::point2d currentCurvePoint;

      //Get the name of the street
      mapData.street_segments[id].streetName = getStreetName(segInfo.streetID);
      //store the id of the street
      mapData.street_segments[id].streetId = segInfo.streetID;

      // Get the curve points (if any)
      if (numberCurvePoints != 0) {
         for (int point = 0; point < numberCurvePoints; ++point) {
            LatLon curvePoint = getStreetSegmentCurvePoint(point, id);
            currentCurvePoint = ezgl::point2d {x_from_lon(curvePoint.longitude()), y_from_lat(curvePoint.latitude())};
            mapData.street_segments[id].all_curve_points.push_back(currentCurvePoint);
            if (point == 0){
               //push the midpoint and angle of the street segment
               mapData.street_segments[id].midpoint.push_back(getMidpoint(from, currentCurvePoint));
               mapData.street_segments[id].angle.push_back(calculateAngle(from, currentCurvePoint));
            } else if (point < numberCurvePoints-1){
               //push the midpoint and angle of the street segment
               LatLon CurvePoint = getStreetSegmentCurvePoint(point+1, id);
               ezgl::point2d nextCurvePoint = ezgl::point2d {x_from_lon(CurvePoint.longitude()), y_from_lat(CurvePoint.latitude())};
               mapData.street_segments[id].midpoint.push_back(getMidpoint(currentCurvePoint, nextCurvePoint));
               mapData.street_segments[id].angle.push_back(calculateAngle(currentCurvePoint, nextCurvePoint));
            }
         }
      }

      // Get the last point (street segment to)
      LatLon latlonto = getIntersectionPosition(segInfo.to);
      ezgl::point2d to = ezgl::point2d (x_from_lon(latlonto.longitude()), y_from_lat(latlonto.latitude()) );
      mapData.street_segments[id].all_curve_points.push_back(to);

      //Get midpoint and angle for streets with no curve points or last curvepoint to "to"
      if (numberCurvePoints == 0){
         mapData.street_segments[id].midpoint.push_back(getMidpoint(from, to));
         mapData.street_segments[id].angle.push_back(calculateAngle(from, to));
      }else{
         mapData.street_segments[id].midpoint.push_back(getMidpoint(currentCurvePoint, to));
         mapData.street_segments[id].angle.push_back(calculateAngle(currentCurvePoint, to));

      }


      // Get way type from OSMID information and load it into our data structure
      OSMID streetSegId = segInfo.wayOSMID;
      std::pair<std::string, std::string> segmentTagPair = mapData.osmWays[streetSegId];
      std::string wayTypeString = segmentTagPair.second;
      mapData.street_segments[id].wayType = stringToWayType(wayTypeString);

      // Load one way info into our data structure
      mapData.street_segments[id].oneWay = segInfo.oneWay;
      
   }

}



//code for loading all the feature in to a structure
void loadFeatures() {
   int numberOfFeatures = getNumFeatures();
   

   // Loop through all the street segments and store relevant information
   
   
   loadFeatureType(FeatureType::LAKE);

   loadFeatureType(FeatureType::ISLAND);

   loadFeatureType(FeatureType::GREENSPACE);
 
   loadFeatureType(FeatureType::GOLFCOURSE);
   
   loadFeatureType(FeatureType::RIVER);

   loadFeatureType(FeatureType::STREAM);

   loadFeatureType(FeatureType::BUILDING);
   
   // Load every other feature in one go to reduce loading time and complexity
   for (int featureId = 0; featureId < numberOfFeatures; featureId++) {
      FeatureType featureType = getFeatureType(featureId);
      if (featureType == PARK || featureType == BEACH || featureType == GLACIER || featureType == UNKNOWN) {
         Feature_data data;
         // Load feature type, feature area and number of feature points

         data.featureType = featureType;
         data.featureArea = findFeatureArea(featureId);
         int numFeaturePoints = getNumFeaturePoints(featureId);
         data.numFeaturePoints = numFeaturePoints;

         //store the features coordinates
         std::vector<ezgl::point2d> featurePoints;
         for (int featurePointId = 0; featurePointId < numFeaturePoints; featurePointId++) {
            LatLon featurePointPosition = getFeaturePoint(featurePointId, featureId);
            ezgl::point2d currentPosition = {x_from_lon(featurePointPosition.longitude()), y_from_lat(featurePointPosition.latitude())};
            featurePoints.push_back(currentPosition);
           
         }
         data.all_feature_points = featurePoints;
         mapData.nonWaterFeatures.push_back(data);         
      }
   }
}

void loadFeatureType(FeatureType desiredFeatureType) {
   int numberOfFeatures = getNumFeatures();
   for (int featureId = 0; featureId < numberOfFeatures; featureId++) {
      FeatureType featureType = getFeatureType(featureId);
      //check which feature type it belongs to
      if (featureType == desiredFeatureType) {
         Feature_data data;

         // Load feature type, feature area and number of feature points
         data.featureType = featureType;
         data.featureArea = findFeatureArea(featureId);
         int numFeaturePoints = getNumFeaturePoints(featureId);
         data.numFeaturePoints = numFeaturePoints;
         std::vector<ezgl::point2d> featurePoints;

         //store the features coordinates
         for (int featurePointId = 0; featurePointId < numFeaturePoints; featurePointId++) {
            LatLon featurePointPosition = getFeaturePoint(featurePointId, featureId);
            ezgl::point2d currentPosition = {x_from_lon(featurePointPosition.longitude()), y_from_lat(featurePointPosition.latitude())};
            featurePoints.push_back(currentPosition);
           
         }
         data.all_feature_points = featurePoints;
         mapData.features.push_back(data);
      
      }
   }

}

//function to load all the OSM ways
void loadOSMWays() {
   int numberOfWays = getNumberOfWays();
   // Loop thorough all the ways and sstore information
   for (unsigned i = 0; i < numberOfWays; i++) {
      const OSMWay *currWay = getWayByIndex(i);
      OSMID ID = currWay -> id();

      for (unsigned j = 0; j < getTagCount(currWay); j++) {
         std::string key, value;
         std::tie(key, value) = getTagPair(currWay, j);
         
         // If the way is a highway, the tag pair is stored into osmWays for major/minor roads classification
         if (key == "highway") {
            std::pair<std::string, std::string> currTagPair = getTagPair(currWay, j);
            mapData.osmWays[ID] = currTagPair;
         }

         // If the way is a railway, the way OSMID and the corresponding index is stored in a map global variable
         if (key == "railway" ) {
            // Match OSMID with the way index given
            mapData.osmRailIndex[ID] = i;
         }
         
      }
      
   }
   
}

// To be called after load OSMWays and load OSMNodes, when all the railRoad index are stored in osmRailIndex and all the nodes and respective indices are stored
void loadOSMRailRoads() {
   
   // Loop through all the OSM Railway
   for (auto it = mapData.osmRailIndex.begin(); it != mapData.osmRailIndex.end(); it++) {
      // Accessing the key
      int value = it -> second;
      const OSMWay* currRailWay = getWayByIndex(value);

      // Get all the members of the way (in node OSMID)
      const std::vector<OSMID> membersOfWay = getWayMembers(currRailWay);

      // Store the relevant node coordinates
      std::vector<ezgl::point2d> wayNodesCoords;

      for (int nodeNum = 0; nodeNum < membersOfWay.size(); nodeNum++) {
         // Store relevant node position in a global variable
         OSMID nodeOSMID = membersOfWay[nodeNum];
         // get node by node index
         const OSMNode* currNode = mapData.nodeIdToNodeMap[nodeOSMID];
         
         // Convert all node coordinates and push back to the list of node postitions
         LatLon nodeCoordsLatLon = getNodeCoords(currNode);
         ezgl::point2d nodeCoordsXY = ezgl::point2d {x_from_lon(nodeCoordsLatLon.longitude()), y_from_lat(nodeCoordsLatLon.latitude())};
         wayNodesCoords.push_back(nodeCoordsXY);
      }

      mapData.railWayNodes.push_back(wayNodesCoords);
   }
}

//function to load all the subway stations
void loadSubwayStations() {
   int numberOfNodes = getNumberOfNodes();
   
   // Loop through all the nodes
   for (unsigned i = 0; i < numberOfNodes; i++) {
      const OSMNode *currNode = getNodeByIndex(i);
      Subway_Station station;

      // For each node, loop through all of its tags
      for (unsigned j = 0; j < getTagCount(currNode); j++) {
         std::string key, value;
         std::tie(key, value) = getTagPair(currNode, j);
         
         // Extract information when key value corresponds to "subway station"
         if (key == "station" && value == "subway") {
            
            // Store the location information for each subway station
            std::pair<std::string, std::string> currTagPair = getTagPair(currNode, j);
            LatLon nodeCoordsLatLon = getNodeCoords(currNode);
            station.xy_loc = ezgl::point2d{x_from_lon(nodeCoordsLatLon.longitude()), y_from_lat(nodeCoordsLatLon.latitude())};
            
            // Loop through the tag count again to find its name
            for (unsigned innerTag = 0; innerTag < getTagCount(currNode); innerTag++) {
               std::string innerKey, innerValue;
               std::tie(innerKey, innerValue) = getTagPair(currNode, innerTag);
               if (innerKey == "name") {
                  station.name = innerValue;
                  break;
               }
            }

            // Store the instance of subway station into the global variable
            mapData.subwayStations.push_back(station);
            break;
         }
   
         
      }
      
      
   }
}

// To be called after load OSMWays
void loadOSMNodes() {

   int numberOfNodes = getNumberOfNodes();

   // Loop through all the nodes and map the OSMID to its respective node index
   for (unsigned i = 0; i < numberOfNodes; i++) {
      const OSMNode *currNode = getNodeByIndex(i);
      OSMID ID = currNode -> id();
      mapData.osmNodes[ID] = i;
  
   }
}

//function to load the relation between subways
void loadSubwayRelations() {
   for (unsigned i = 0; i < getNumberOfRelations(); i++) {
      const OSMRelation* currRelation = getRelationByIndex(i);

      for (unsigned j = 0; j < getTagCount(currRelation); j++) {
         std::string key, value;
         std::tie(key, value) = getTagPair(currRelation, j);

         if (key == "route" && value == "subway") {

            std::vector<TypedOSMID> relationMembers = getRelationMembers(currRelation);
            std::vector<std::string> relationMemberRoles = getRelationMemberRoles(currRelation);

            for (int memberID = 0; memberID < relationMembers.size(); memberID++) {

               if (relationMembers[memberID].type() == TypedOSMID::Way) {

                  if (relationMemberRoles[memberID] != "platform") {
                     
                     const OSMWay* subwayWays = mapData.wayIdToWayMap[relationMembers[memberID]];

                     const std::vector<OSMID> subwayMembers = getWayMembers(subwayWays);
                     
                     Subway_Lines line;

                     std::vector<ezgl::point2d> subwayNodeCoordsXY;

                     for (int nodeNum = 0; nodeNum < subwayMembers.size(); nodeNum++) {
                        const OSMNode* currNode = mapData.nodeIdToNodeMap[subwayMembers[nodeNum]];
                        LatLon nodeCoordsLatLon = getNodeCoords(currNode);
                        ezgl::point2d nodeCoordsXY = ezgl::point2d {x_from_lon(nodeCoordsLatLon.longitude()), y_from_lat(nodeCoordsLatLon.latitude())};
                        subwayNodeCoordsXY.push_back(nodeCoordsXY);
                     }

                     line.wayPositions = subwayNodeCoordsXY;

                     for (unsigned k = 0; k < getTagCount(currRelation); k++) {
                        std::string innerKey, innerValue;
                        std::tie(innerKey, innerValue) = getTagPair(currRelation, k);

                        if (innerKey == "colour") {
                           line.color = innerValue;
                        }
                     }

                     mapData.subways.push_back(line);

                  }


               }



            }


         }

      }
   }
}

//function to draw all the subway lines
void drawSubwayLines(ezgl::renderer *g) {
   
   double currZoomLevel = getZoomLevel(g);
   if (currZoomLevel > 8) {
      for (int i = 0; i < mapData.subways.size(); i++) {
         if (mapData.subways[i].color[0] == '#') {
            std::string hexString = mapData.subways[i].color;
            g -> set_color(hexToDecimalR(hexString), hexToDecimalG(hexString), hexToDecimalB(hexString));
         } else if (mapData.subways[i].color == "red") {
            g -> set_color(ezgl::RED);
         } else if (mapData.subways[i].color == "yellow") {
            g -> set_color(ezgl::YELLOW);
         } else if (mapData.subways[i].color == "green") {
            g -> set_color(ezgl::GREEN);
         } else if (mapData.subways[i].color == "brown") {
            g -> set_color(139, 69, 19);
         } else if (mapData.subways[i].color == "gray") {
            g -> set_color(47, 79, 79);
         } else if (mapData.subways[i].color == "white") {
            g -> set_color(ezgl::WHITE);
         } else if (mapData.subways[i].color == "black") {
            g -> set_color(ezgl::BLACK);
         } else if (mapData.subways[i].color == "blue") {
            g -> set_color(ezgl::BLUE);
         } else if (mapData.subways[i].color == "orange") {
            g -> set_color(ezgl::ORANGE);
         } else if (mapData.subways[i].color == "purple") {
            g -> set_color(ezgl::PURPLE);
         } else  {
            g -> set_color(ezgl::BLACK);
         }
         



         for (int j = 0; j < mapData.subways[i].wayPositions.size(); j++) {
            // && size != 0
            if (j + 1 != mapData.subways[i].wayPositions.size()) {
               g->set_line_width(6);
               g->draw_line(mapData.subways[i].wayPositions[j], mapData.subways[i].wayPositions[j+1]);

            }
         
         }
      }
   }

   
}

// Draws different level of features, 1 is only motorway, 2 or higher would increasingly draw more features
void drawLevelsOfStreets(int levelOfFeatures, ezgl::renderer *g) {
   // Loop through all the street segments and their curve pointss
   for (int id = 0; id < mapData.street_segments.size(); ++id) {
      for (int pointId = 0; pointId < mapData.street_segments[id].all_curve_points.size(); ++pointId) {
         if (pointId + 1 != mapData.street_segments[id].all_curve_points.size()) {
            // Loop through the levels of features
            for (int k = 1; k <= levelOfFeatures; k++) {
               plotStreets(id, pointId, k, g);
            }         
         }
      }
   }
}

//fuction to draw street segments
void drawStreetSegments(ezgl::renderer *g) {
   double currentZoomLevel = getZoomLevel(g);

   // Draw different streets according to zoom level
   if (currentZoomLevel <= 1) {
      drawLevelsOfStreets(1, g);
   } else if (currentZoomLevel < 2) {
      drawLevelsOfStreets(2, g);
   } else if (currentZoomLevel < 4) {
      drawLevelsOfStreets(3, g);
   } else if (currentZoomLevel < 8) {
      drawLevelsOfStreets(4, g);
   } else {
      drawLevelsOfStreets(5, g);
   }
   
}

void drawIntersections(ezgl::renderer *g) {
   for (size_t i = 0; i < mapData.intersections.size(); ++i) {
      if (mapData.intersections[i].highlight) {
         g -> set_color(ezgl::RED);
         float width = 8;   
         float height = width;

         // Center the intersection location
         ezgl::point2d inter_loc = mapData.intersections[i].xy_loc - ezgl::point2d{width/2, height/2};
         
         g->fill_rectangle(inter_loc, width, height);
      } 
  }
}

//function to draw all the features on the map
void drawFeatures(ezgl:: renderer *g) {
   double currentZoomLevel = getZoomLevel(g);

   
   for (size_t i = 0; i < mapData.features.size(); ++i) {
      int numFeaturePoints = mapData.features[i].numFeaturePoints;
      if (currentZoomLevel < 1) { 
         // reduce # of parameters
         plotFeatures(i, numFeaturePoints, 80000, g); 
      } else if (currentZoomLevel < 3) {
         plotFeatures(i, numFeaturePoints, 40000, g);
      } else if (currentZoomLevel < 8) {
         plotFeatures(i, numFeaturePoints, 20000, g);
      } else if (currentZoomLevel < 12) {
         plotFeatures(i, numFeaturePoints, 10000, g);
      } else if (currentZoomLevel < 16) {
         plotFeatures(i, numFeaturePoints, 6000, g);
      } else if (currentZoomLevel < 25) {
         plotFeatures(i, numFeaturePoints, 2000, g);
      } else {
         plotFeatures(i, numFeaturePoints, 0, g);
      }
   }

   for (size_t i = 0; i < mapData.nonWaterFeatures.size(); ++i) {
      int numFeaturePoints = mapData.nonWaterFeatures[i].numFeaturePoints;
      if (currentZoomLevel < 1) { 
         // reduce # of parameters
         plotNonWaterFeatures(i, numFeaturePoints, 80000, g); 
      } else if (currentZoomLevel < 3) {
         plotNonWaterFeatures(i, numFeaturePoints, 40000, g);
      } else if (currentZoomLevel < 8) {
         plotNonWaterFeatures(i, numFeaturePoints, 20000, g);
      } else if (currentZoomLevel < 12) {
         plotNonWaterFeatures(i, numFeaturePoints, 10000, g);
      } else if (currentZoomLevel < 16) {
         plotNonWaterFeatures(i, numFeaturePoints, 6000, g);
      } else if (currentZoomLevel < 25) {
         plotNonWaterFeatures(i, numFeaturePoints, 2000, g);
      } else {
         plotNonWaterFeatures(i, numFeaturePoints, 0, g);
      }   
   }

}

//function to draw the subway stations
void drawSubwayStations(ezgl::renderer *g) {
   double currZoomLevel = getZoomLevel(g);
   if (currZoomLevel > 8) {
      if (mapData.darkMode) {
         g->set_color(ezgl::WHITE);
      } else {
         g->set_color(ezgl::BLACK);
      }
      for (int i = 0; i < mapData.subwayStations.size(); i++) {
         g->format_font("Noto Sans CJK SC", ezgl::font_slant::normal, ezgl::font_weight::normal);
         ezgl::point2d adjustedLocation = ezgl::point2d {mapData.subwayStations[i].xy_loc.x, mapData.subwayStations[i].xy_loc.y - 10};
         g->fill_arc(adjustedLocation, 3, 0, 360);
         g->draw_text(mapData.subwayStations[i].xy_loc, mapData.subwayStations[i].name);
      }
     
   }
   
}

//function to draw the rail ways
void drawRailWays(ezgl::renderer *g) {
   double currZoomLevel = getZoomLevel(g);
   if (currZoomLevel > 15) {
      if (mapData.darkMode) {
         g->set_color(ezgl::YELLOW);
      } else {
         g->set_color(ezgl::ORANGE);
      }
      for (int i = 0; i < mapData.railWayNodes.size(); i++) {
         for (int j = 0; j < mapData.railWayNodes[i].size(); j++) {
            // && size != 0
            if (j + 1 != mapData.railWayNodes[i].size()) {
               g->set_line_width(1);
               g->draw_line(mapData.railWayNodes[i][j], mapData.railWayNodes[i][j+1]);

            }
         
         }
      }
   }
   
}


/***************************code to display street information on map*****************************/


//functoin to draw streetnames on the map
void drawStreetNames(ezgl::renderer *g) {
   //get the zoom level of the map  
   double currentZoomLevel = getZoomLevel(g);
   g->format_font("Noto Sans CJK SC", ezgl::font_slant::normal, ezgl::font_weight::normal);
   //set the colour and font size of the streetname
   if (mapData.darkMode) {
      g->set_color(ezgl::WHITE);
   } else {
      g->set_color(80, 80, 80);   
   }

   ezgl::rectangle world = g -> get_visible_world();
   
   
   //loop through all the streetsegments as well as each segments curvepoint
   for (int Id = 0; Id < mapData.street_segments.size(); Id += 1) {
      ezgl::point2d midPoint = mapData.street_segments[Id].midpoint[0];
      bool plot = false;
      //check if the window has this point
      if (world.contains(midPoint.x, midPoint.y)) {
         //loop through all the curved points and check the zoomlevel to draw the approporate number of street name drawn
         for (int pointId = 0; pointId < mapData.street_segments[Id].all_curve_points.size(); pointId += 20) {
            if(mapData.street_segments[Id].wayType == MOTORWAY && pointId ==0 && Id%300 == 0 && currentZoomLevel > 7){
               g->set_font_size(8);
               plot = true;
            }else if (mapData.street_segments[Id].wayType != TERTIARY && pointId + 1 <= mapData.street_segments[Id].all_curve_points.size() && currentZoomLevel > 36 && Id%50 ==0) { 
               g->set_font_size(8);           
               plot = true;
            }else if (mapData.street_segments[Id].wayType != TERTIARY && pointId + 1 <= mapData.street_segments[Id].all_curve_points.size() && currentZoomLevel > 165 && Id%20 ==0) { 
               g->set_font_size(10);             
               plot = true;
            }
            else if (pointId + 1 <= mapData.street_segments[Id].all_curve_points.size() && currentZoomLevel > 459 && Id%10 ==0) {  
               g->set_font_size(12);            
               plot = true;
            }
            if(mapData.street_segments[Id].streetName != "<unknown>" && plot){
                  //obtain the angle between two points for a given segment, note that this is pregenerated in loadstreetsegmentdata
               double angle = mapData.street_segments[Id].angle[pointId];
               std::cout << angle << std::endl;
               //ensure that there is a valid angle
               if(angle < -360 || angle > 360){
                  angle = 0;
               }
               //set the text rotation so that the streetname aligns with the street
               g->set_text_rotation(angle);
               //avoid drawing the streets that have no name
               g->draw_text(mapData.street_segments[Id].midpoint[pointId], mapData.street_segments[Id].streetName);
               }
         }
      }
   }
}




//function to load POI images to a POI strucutre pre-defined in globals.h
void loadPOIImage(){
   //get the total number of POI's
   int numberOfPOIs = getNumPointsOfInterest();
   //allocate space for the POIs_data vector
   mapData.POIs_data.resize(numberOfPOIs);

   //loop through all the POIs on the map and assign a surface based on their type, as well as pre-load their corrdinates to the strucuture in globals.h
   for(int Id = 0; Id < numberOfPOIs; Id++){
      //get the POI coordinate
      ezgl::point2d poiCoordinate(x_from_lon(getPOIPosition(Id).longitude()), y_from_lat(getPOIPosition(Id).latitude()));
      //get the POI type
      std::string poiType = getPOIType(Id);
      //default surface is surface1
      ezgl::surface* poiSurface = mapData.poiSurface1;
      //assign icons for POI's based on their type
      if (poiType == "hospital"  || poiType == "pharmacy" ) {
         poiSurface = mapData.poiSurface3;
      } else if (poiType == "atm" || poiType == "bank") {
         poiSurface = mapData.poiSurface4;
      } else if (poiType == "bar" || poiType == "pub" || poiType == "food" || poiType == "restaurant" || poiType == "fast_food" || poiType == "cafe" || poiType == "cafe;bakery") {
         poiSurface = mapData.poiSurface5;
      } else if (poiType == "food_bank" || poiType == "fire_station" || poiType == "car_rental" || poiType == "drinking_water" || poiType == "gym" || poiType == "park") {
         poiSurface = mapData.poiSurface6;
      } else if (poiType == "cinema" || poiType == "nightclub" || poiType == "social_facility" || poiType == "massage" || poiType == "social_services") {
         poiSurface = mapData.poiSurface7;
      } else if (poiType == "fire_station" || poiType == "police") {
         poiSurface = mapData.poiSurface8;
}
   //add the POI's coordinate and surface to the vector of structures
   mapData.POIs_data[Id].poi_Coordinate = poiCoordinate;
   mapData.POIs_data[Id].poi_Surface = poiSurface;
   //ezgl::renderer::free_surface(poiSurface);
   }
}

//function to draw the POI's icon as well as its corrosponding name
void drawPOIImage(ezgl::renderer *g){
   ezgl::rectangle world = g -> get_visible_world();
   if (mapData.darkMode) {
      g->set_color(ezgl::WHITE);
   } else {
      g->set_color(ezgl::BLACK);   
   }
   g->set_font_size(9);
   //obtain the zoomlevel
   double currentZoomLevel = getZoomLevel(g);
   //for loop to go through one tenth of the POI's to increase responsiveness
   for (int Id = 0; Id < mapData.POIs_data.size(); Id+=10) {
      ezgl::point2d midPoint = mapData.POIs_data[Id].poi_Coordinate;
      //only display streetname at a given zoom to meet usability and responsiveness
      if(currentZoomLevel > 60 && (Id%5) == 0 && world.contains(midPoint.x, midPoint.y)){
         //draw the surface(icon image), as well as change the size of the surface during user zoom to improve usability
         g->draw_surface(mapData.POIs_data[Id].poi_Surface, mapData.POIs_data[Id].poi_Coordinate, pow(currentZoomLevel, 0.3) * 0.0165);
         //draw the text describing the POI
         if(currentZoomLevel > 165){
         ezgl::point2d poiTextCoordinate(mapData.POIs_data[Id].poi_Coordinate.x, mapData.POIs_data[Id].poi_Coordinate.y - 20 + pow(currentZoomLevel - 165.382, 0.3));
         g->draw_text(poiTextCoordinate, getPOIName(Id));
         }
      }
      if(currentZoomLevel > 165 && world.contains(midPoint.x, midPoint.y)){
         //draw the surface(icon image), as well as change the size of the surface during user zoom to improve usability
         g->draw_surface(mapData.POIs_data[Id].poi_Surface, mapData.POIs_data[Id].poi_Coordinate, pow(currentZoomLevel, 0.3) * 0.0165);
         //draw the text describing the POI
         if(currentZoomLevel > 165){
         ezgl::point2d poiTextCoordinate(mapData.POIs_data[Id].poi_Coordinate.x, mapData.POIs_data[Id].poi_Coordinate.y - 20 + pow(currentZoomLevel - 165.382, 0.3));
         g->draw_text(poiTextCoordinate, getPOIName(Id));
         }
      }
   }
}


//functon to draw a one way arrow on the street
void drawOneWayArrow(ezgl::renderer *g){
   //set the font size of the arrow
   g->set_font_size(14);
   //set the colour of the arrow to black

   if (mapData.darkMode) {
      g->set_color(ezgl::WHITE);
   } else {
      g->set_color(50, 50, 50);   
   }
   //get the zoom level
   double currentZoomLevel = getZoomLevel(g);
   //get the world frame
   ezgl::rectangle world = g -> get_visible_world();
   //loop through all of the streets and plot one way arrow based on the zoom level
   for(int Id = 0; Id < mapData.street_segments.size(); Id += 10){
   ezgl::point2d midPoint = mapData.street_segments[Id].midpoint[0];
      //if statement to figure out weather the street is one way
      if (world.contains(midPoint.x, midPoint.y)) {
         if(mapData.street_segments[Id].oneWay && currentZoomLevel > 28 &&  currentZoomLevel < 36){
            //set the rotation of the arrow
            g->set_font_size(8);
            g->set_text_rotation(mapData.street_segments[Id].angle[0]);
            //draw the arrow on the map
            g->draw_text(mapData.street_segments[Id].midpoint[0], "—>");
         }
      if(mapData.street_segments[Id].oneWay&& currentZoomLevel > 99 &&  currentZoomLevel < 270){
            //set the rotation of the arrow
            g->set_text_rotation(mapData.street_segments[Id].angle[0]);
            //draw the arrow on the map
            g->draw_text(mapData.street_segments[Id].midpoint[0], "—>");
         }
      }
   }
}
//function to set the feature color
void setFeatureColor(FeatureType featureType, ezgl::renderer *g) {

   // Dark mode colors
   if (mapData.darkMode) {
      if (featureType == PARK) {
         g -> set_color(34, 94, 79, 100); // Dark green
      } else if (featureType == BEACH) {
         g -> set_color(163, 154, 120, 100); // Dark sandy color
      } else if (featureType == LAKE) {
         g -> set_color(43, 108, 133, 100); // Dark blue
      } else if (featureType == RIVER) {
         g -> set_color(43, 108, 133, 100); // Dark blue
      } else if (featureType == ISLAND) {
         g -> set_color(50, 50, 50); // Dark gray
      } else if (featureType == BUILDING) {
         g -> set_color(85, 97, 108, 255); // Darker shade of the original
      } else if (featureType == GREENSPACE) {
         g -> set_color(34, 94, 79, 100); // Dark green, same as PARK
      } else if (featureType == GOLFCOURSE) {
         g -> set_color(34, 94, 79, 100); // Dark green, same as PARK
      } else if (featureType == STREAM) {
         g -> set_color(43, 108, 133, 100); // Dark blue
      } else if (featureType == GLACIER) {
         g -> set_color(132, 124, 128, 100); // Dark icy gray
      } else if (featureType == UNKNOWN) {
         g -> set_color(204, 102, 0); // Darker shade of orange
      }
   } else {
      // Light mode colors
   switch (featureType) {
         case PARK: 
            g -> set_color(204, 240, 215, 100);
            break; 
         case GREENSPACE: 
            g -> set_color(204, 240, 215, 100);
            break; 
         case GOLFCOURSE: 
            g -> set_color(204, 240, 215, 130);
            break;  
         case BEACH:
            g -> set_color(245, 236, 210, 100);
            break;
         case LAKE: 
            g -> set_color(160, 216, 235, 100);
            break;
         case RIVER:
            g -> set_color(160, 216, 235, 100);
            break; 
         case STREAM: 
            g -> set_color(160, 216, 235, 100);
            break;
         case GLACIER:
            g -> set_color(212, 204, 208, 100);
            break;
         case BUILDING:
            g -> set_color(247, 233, 187, 175);
            break;
         case ISLAND:
            g -> set_color(ezgl::WHITE);
            break;
         case UNKNOWN:
            g -> set_color(ezgl::ORANGE);
            break;
         default:
            g -> set_color(ezgl::BLACK);
      }
   } 
}


//function to get the current zoom level
double getZoomLevel(ezgl::renderer *g) {
   // Current world coordinates
   ezgl::rectangle world = g -> get_visible_world();

   // Initial world coordinates
   ezgl::rectangle initial_world({x_from_lon(mapData.min_lon), y_from_lat(mapData.min_lat)},
                                {x_from_lon(mapData.max_lon), y_from_lat(mapData.max_lat)});

   
   double currentWidth = world.width();
   double initialWidth = initial_world.width();
   
   // Set zoom level as the width ratio 
   double ratio = initialWidth / currentWidth;
   return ratio;
}

//function to plot all the features
void plotFeatures(FeatureIdx i, int numFeaturePoints, int areaLimit, ezgl::renderer *g) {
   // If the feature is bigger than a certain area limit
   if(mapData.features[i].featureArea > areaLimit) {
      // If the feature is not a degenerate single point feature, then we draw the features
      if (numFeaturePoints > 1) {
         if (mapData.features[i].all_feature_points[0] == mapData.features[i].all_feature_points[numFeaturePoints - 1]) {
            setFeatureColor(mapData.features[i].featureType, g);
            g -> fill_poly(mapData.features[i].all_feature_points);
         } else {
            setFeatureColor(mapData.features[i].featureType, g);
            g -> set_line_width(2);
            for (int featurePointId = 0; featurePointId < numFeaturePoints - 1; featurePointId++) {  
               g -> draw_line(mapData.features[i].all_feature_points[featurePointId], mapData.features[i].all_feature_points[featurePointId + 1]);
            }
         }
            
      }      
   }
}

//function to plot non water features
void plotNonWaterFeatures(FeatureIdx i, int numFeaturePoints, int areaLimit, ezgl::renderer *g) {
   // Omit features that are greater than a certain area limit
   if(mapData.nonWaterFeatures[i].featureArea > areaLimit) {
      // Printing will noticeably reduce efficiency
      if (numFeaturePoints > 1) {
         if (mapData.nonWaterFeatures[i].all_feature_points[0] == mapData.nonWaterFeatures[i].all_feature_points[numFeaturePoints - 1]) {
            setFeatureColor(mapData.nonWaterFeatures[i].featureType, g);
            g -> fill_poly(mapData.nonWaterFeatures[i].all_feature_points);
         } else {
            setFeatureColor(mapData.nonWaterFeatures[i].featureType, g);
            g->set_line_width(2);
            for (int featurePointId = 0; featurePointId < numFeaturePoints - 1; featurePointId++) {  
               g->draw_line(mapData.nonWaterFeatures[i].all_feature_points[featurePointId], mapData.nonWaterFeatures[i].all_feature_points[featurePointId + 1]);
            }
         }   
      }      
   }   
}

void drawScale(ezgl::renderer *g){

   double screenMeter = g->world_to_screen(ezgl::rectangle(g -> get_visible_world().bottom_left(),1,1)).width();
   double currScale = g->get_visible_world().width();
   double displayLength;
   std::string displayText;
   g->set_coordinate_system(ezgl::SCREEN);
   if ( currScale <= 100 ) {
      displayLength = screenMeter;
      displayText = "1m";
   }
   else if( currScale <= 1000 ){
      displayLength = screenMeter*10;
      displayText = "10m";
   }
   else if( currScale <= 10000 ){
      displayLength = screenMeter*100;
      displayText = "100m";
   }
   else if( currScale <= 100000 ){
      displayLength = screenMeter*1000;
      displayText = "1km";
   }
   else if( currScale <= 1000000 ){
      displayLength = screenMeter*10000;
      displayText = "10km";
   }
   else if( currScale <= 10000000 ){
      displayLength = screenMeter*100000;
      displayText = "100km";
   }
   else{ //should not pass here
      displayLength = screenMeter*1000000;
      displayText = "1000km";
   }
   g->set_line_width(5);
   g->set_font_size(12);
   g->set_color(90,90,90,255);
   int left = g->get_visible_screen().left();
   int top = g->get_visible_screen().top();
   g-> set_text_rotation(0);
   g->draw_text(ezgl::point2d(left + 50, top - 50),"0");
   g->draw_text(ezgl::point2d(left + 50 + displayLength, top - 50),displayText);
   g->draw_line(ezgl::point2d(left + 50, top - 65),ezgl::point2d(left + 50 + displayLength, top - 65));
   g->set_coordinate_system(ezgl::WORLD);
}


//function to set the color of the roads
void setRoadColor(WayType wayType, ezgl::renderer *g) {
   if(mapData.darkMode) {
      if (wayType == NA) {
         g -> set_color(ezgl::BLACK); // Slightly lighter than black for visibility
      } else if (wayType == MOTORWAY) {
         // g -> set_color(52, 86, 116, 255); // Darker blue
         g -> set_color(185, 130, 58, 255); // Darker Orange
      } else if (wayType == TRUNK) {
         g -> set_color(66, 106, 123, 255); // Darker cyan-blue
      } else if (wayType == PRIMARY) {
         g -> set_color(66, 106, 123, 255); // Same as TRUNK for consistency
      } else if (wayType == SECONDARY) {
         g -> set_color(99, 137, 150, 255); // Darker light blue
      } else if (wayType == TERTIARY) { 
         g -> set_color(105, 137, 148, 255); // Darker off-white
      } else if (wayType == UNCLASSIFIED) {
         g -> set_color(105, 137, 148, 255); // Same as TERTIARY
      } else if (wayType == RESIDENTIAL) {
         g -> set_color(105, 137, 148, 255); // Same as TERTIARY
      } else if (wayType == MOTORWAY_LINK) {
         //g -> set_color(52, 86, 116, 255); // Same as MOTORWAY
         g -> set_color(185, 130, 58, 255); // Darker Orange
      } else if (wayType == TRUNK_LINK) {
         g -> set_color(66, 106, 123, 255); // Same as TRUNK
      } else if (wayType == PRIMARY_LINK) {
         g -> set_color(66, 106, 123, 255); // Same as PRIMARY
      } else if (wayType == SECONDARY_LINK) {
         g -> set_color(99, 137, 150, 255); // Same as SECONDARY
      } else if (wayType == TERTIARY_LINK) {
         g -> set_color(105, 137, 148, 255); // Same as TERTIARY
      } else if (wayType == LIVING_STREET) {
         g -> set_color(105, 137, 148, 255); // Same as TERTIARY
      } else if (wayType == SERVICE) {
         g -> set_color(105, 137, 148, 255); // Same as TERTIARY
      } else if (wayType == PEDESTRIAN) {
         g -> set_color(85, 107, 118, 255); // Darker gray-blue
      } else if (wayType == TRACK) {
         g -> set_color(85, 107, 118, 255); // Same as PEDESTRIAN
      } else if (wayType == BUS_GUIDEWAY) {
         g -> set_color(85, 107, 118, 255); // Same as PEDESTRIAN
      } else if (wayType == ESCAPE) {
         g -> set_color(85, 107, 118, 255); // Same as PEDESTRIAN
      } else if (wayType == RACEWAY) {
         g -> set_color(85, 107, 118, 255); // Same as PEDESTRIAN
      } else if (wayType == ROAD) {
         g -> set_color(85, 107, 118, 255); // Same as PEDESTRIAN
      } else if (wayType == BUSWAY) {
         g -> set_color(85, 107, 118, 255); // Same as PEDESTRIAN
      } else if (wayType == FOOTWAY) {
         g -> set_color(75, 97, 108, 255); // Slightly darker for footpaths
      } else if (wayType == BRIDLEWAY) {
         g -> set_color(75, 97, 108, 255); // Same as FOOTWAY
      } else if (wayType == STEPS) {
         g -> set_color(75, 97, 108, 255); // Same as FOOTWAY
      } else if (wayType == CORRIDOR) {
         g -> set_color(75, 97, 108, 255); // Same as FOOTWAY
      } else if (wayType == PATH) {
         g -> set_color(75, 97, 108, 255); // Same as FOOTWAY
      } else if (wayType == VIA_FERRANTA) {
         g -> set_color(75, 97, 108, 255); // Same as FOOTWAY
      } else if (wayType == CYCLEWAY) {
         g -> set_color(75, 97, 108, 255); // Same as FOOTWAY
      } else if (wayType == PROPOSED) {
         g -> set_color(75, 97, 108, 255); // Same as FOOTWAY
      } else if (wayType == CONSTRUCTION) {
         g -> set_color(75, 97, 108, 255); // Same as FOOTWAY
      } 
   } else {
      if (wayType == NA) {
         g -> set_color(ezgl::BLACK);
      } else if (wayType == MOTORWAY) {
         g -> set_color(132,166,196,255);
      } else if (wayType == TRUNK) {
         g -> set_color(166,186,203,255);
      } else if (wayType == PRIMARY) {
         g -> set_color(166,186,203,255);
      } else if (wayType == SECONDARY) {
         g -> set_color(199,217,230,255);
      } else if (wayType == TERTIARY) {
         g -> set_color(205,217,228,255);
      } else if (wayType == UNCLASSIFIED) {
         g -> set_color(205,217,228,255);
      } else if (wayType == RESIDENTIAL) {
         g -> set_color(205,217,228,255);
      } else if (wayType == MOTORWAY_LINK) {
         g -> set_color(132,166,196,255);
      } else if (wayType == TRUNK_LINK) {
         g -> set_color(166,186,203,255);
      } else if (wayType == PRIMARY_LINK) {
         g -> set_color(166,186,203,255);
      } else if (wayType == SECONDARY_LINK) {
         g -> set_color(199,217,230,255);
      } else if (wayType == TERTIARY_LINK) {
         g -> set_color(205,217,228,255);
      } else if (wayType == LIVING_STREET) {
         g -> set_color(205,217,228,255);
      } else if (wayType == SERVICE) {
         g -> set_color(205,217,228,255);
      } else if (wayType == PEDESTRIAN) {
         g -> set_color(205,217,228,255);
      } else if (wayType == TRACK) {
         g -> set_color(205,217,228,255);
      } else if (wayType == BUS_GUIDEWAY) {
         g -> set_color(205,217,228,255);
      } else if (wayType == ESCAPE) {
         g -> set_color(205,217,228,255);
      } else if (wayType == RACEWAY) {
         g -> set_color(205,217,228,255);
      } else if (wayType == ROAD) {
         g -> set_color(205,217,228,255);
      } else if (wayType == BUSWAY) {
         g -> set_color(205,217,228,255);
      } else if (wayType == FOOTWAY) {
         g -> set_color(205,217,228,255);
      } else if (wayType == BRIDLEWAY) {
         g -> set_color(205,217,228,255);
      } else if (wayType == STEPS) {
         g -> set_color(205,217,228,255);
      } else if (wayType == CORRIDOR) {
         g -> set_color(205,217,228,255);
      } else if (wayType == PATH) {
         g -> set_color(205,217,228,255);
      } else if (wayType == VIA_FERRANTA) {
         g -> set_color(205,217,228,255);
      } else if (wayType == CYCLEWAY) {
         g -> set_color(205,217,228,255);
      } else if (wayType == PROPOSED) {
         g -> set_color(205,217,228,255);
      } else if (wayType == CONSTRUCTION) {
         g -> set_color(205,217,228,255);
      } 
   }
}

//function to plot all the streest

void plotStreets(int streetSegID, int pointId, int layerNum, ezgl::renderer *g) {
   WayType wayType = mapData.street_segments[streetSegID].wayType;
   ezgl::rectangle world = g -> get_visible_world();
   ezgl::point2d midPoint = mapData.street_segments[streetSegID].midpoint[0];

   // Only plot the streets if its position is within the current world boundaries
   if(world.contains(midPoint.x, midPoint.y)){
      if (layerNum == 1) {
         if (wayType == MOTORWAY || wayType == MOTORWAY_LINK) {
            setRoadColor(wayType,g);
            g->set_line_width(6);
            g->draw_line(mapData.street_segments[streetSegID].all_curve_points[pointId], mapData.street_segments[streetSegID].all_curve_points[pointId+1]);
         }
      } else if (layerNum == 2) {
         if (wayType == PRIMARY || wayType == PRIMARY_LINK || wayType == TRUNK || wayType == TRUNK_LINK) {
            setRoadColor(wayType,g);
            g->set_line_width(3);
            g->draw_line(mapData.street_segments[streetSegID].all_curve_points[pointId], mapData.street_segments[streetSegID].all_curve_points[pointId+1]);
      }
      } else if (layerNum == 3) {
         if (wayType == SECONDARY || wayType == SECONDARY_LINK) {
            setRoadColor(wayType,g);
            g->set_line_width(3);
            g->draw_line(mapData.street_segments[streetSegID].all_curve_points[pointId], mapData.street_segments[streetSegID].all_curve_points[pointId+1]);
      }
      } else if (layerNum == 4) {
         if (wayType == TERTIARY || wayType == TERTIARY_LINK || wayType == RESIDENTIAL) {
            setRoadColor(wayType,g);
            g->set_line_width(2);
            g->draw_line(mapData.street_segments[streetSegID].all_curve_points[pointId], mapData.street_segments[streetSegID].all_curve_points[pointId+1]);
      }
      } else if (layerNum == 5){
         if (wayType != MOTORWAY && wayType != MOTORWAY_LINK && wayType != PRIMARY && wayType != PRIMARY_LINK && wayType != TRUNK && wayType != TRUNK_LINK && wayType != SECONDARY && wayType != SECONDARY && wayType != SECONDARY_LINK && wayType != TERTIARY && wayType != TERTIARY_LINK && wayType != RESIDENTIAL && wayType != TRUNK_LINK) {
            setRoadColor(wayType,g);
            g->set_line_width(1);
            g->draw_line(mapData.street_segments[streetSegID].all_curve_points[pointId], mapData.street_segments[streetSegID].all_curve_points[pointId+1]);
         }
      }
   }
}


//function to convert string type to way type
WayType stringToWayType(std::string type) {
   // Map string into respective WayType
   if (type == "motorway") {
      return WayType::MOTORWAY;
   } else if (type == "trunk") {
      return WayType::TRUNK;
   } else if (type == "primary") {
      return WayType::PRIMARY;
   } else if (type == "secondary") {
      return WayType::SECONDARY;
   } else if (type == "tertiary") {
      return WayType::TERTIARY;
   } else if (type == "unclassified") {
      return WayType::UNCLASSIFIED;
   } else if (type == "residential") {
      return WayType::RESIDENTIAL;
   } else if (type == "motorway_link") {
      return WayType::MOTORWAY_LINK;
   } else if (type == "trunk_link") {
      return WayType::TRUNK_LINK;
   } else if (type == "primary_link") {
      return WayType::PRIMARY_LINK;
   } else if (type == "secondary_link") {
      return WayType::SECONDARY_LINK;
   } else if (type == "tertiary_link") {
      return WayType::TERTIARY_LINK;
   } else if (type == "living_street") {
      return WayType::LIVING_STREET;
   } else if (type == "service") {
      return WayType::SERVICE;
   } else if (type == "bus_guideway") {
      return WayType::BUS_GUIDEWAY;
   } else if (type == "escape") {
      return WayType::ESCAPE;
   } else if (type == "raceway") {
      return WayType::RACEWAY;
   } else if (type == "road") {
      return WayType::ROAD;
   } else if (type == "busway") {
      return WayType::BUSWAY;
   } else if (type == "footway") {
      return WayType::FOOTWAY;
   } else if (type == "bridleway") {
      return WayType::BRIDLEWAY;
   } else if (type == "steps") {
      return WayType::STEPS;
   } else if (type == "corridor") {
      return WayType::CORRIDOR;
   } else if (type == "path") {
      return WayType::PATH;
   } else if (type == "via_ferranta") {
      return WayType::VIA_FERRANTA;
   } else if (type == "cycleway") {
      return WayType::CYCLEWAY;
   } else if (type == "proposed") {
      return WayType::PROPOSED;
   } else if (type == "construction") {
      return WayType::CONSTRUCTION;
   } else {
      return WayType::NA;
   }
}

int hexToDecimalR(std::string hexString) {
   // Get the string of hex color
   std::string hexStringCopy = hexString;
   // Remove hashtag
   hexStringCopy.erase(0, 1);

   // Pass the string as input and read it out as integers
   std::stringstream ss;
   ss << std::hex << hexStringCopy;
   unsigned int hexValue;
   ss >> hexValue;

   // Extract the first 2 bits to get the color in decimal
   int red = (hexValue >> 16) & 0xFF;

   return red;
}

int hexToDecimalG(std::string hexString) {
   // Get the string of hex color
   std::string hexStringCopy = hexString;
   // Remove hashtag
   hexStringCopy.erase(0, 1);

   // Pass the string as input and read it out as integers
   std::stringstream ss;
   ss << std::hex << hexStringCopy;
   unsigned int hexValue;
   ss >> hexValue;

   // Extract the middle 2 bits to get the color in decimal
   int green = (hexValue >> 8) & 0xFF;

   return green;
}

int hexToDecimalB(std::string hexString) {
   // Get the string of hex color
   std::string hexStringCopy = hexString;
   // Remove hashtag
   hexStringCopy.erase(0, 1);

   // Pass the string as input and read it out as integers
   std::stringstream ss;
   ss << std::hex << hexStringCopy;
   unsigned int hexValue;
   ss >> hexValue;


   // Extract the last 2 bits to get the color in decimal
   int blue = hexValue & 0xFF;

   return blue;
}

void loadBusRoutes() {
   for (unsigned i = 0; i < getNumberOfRelations(); i++) {
      const OSMRelation* currRelation = getRelationByIndex(i);

      for (unsigned j = 0; j < getTagCount(currRelation); j++) {
         std::string key, value;
         std::tie(key, value) = getTagPair(currRelation, j);

         if (key == "route" && value == "bus") {

            std::vector<TypedOSMID> relationMembers = getRelationMembers(currRelation);
            std::vector<std::string> relationMemberRoles = getRelationMemberRoles(currRelation);

            for (int memberID = 0; memberID < relationMembers.size(); memberID++) {

               if (relationMembers[memberID].type() == TypedOSMID::Way) {

                  if (relationMemberRoles[memberID] != "platform") {
                     
                     const OSMWay* busWays = mapData.wayIdToWayMap[relationMembers[memberID]];

                     const std::vector<OSMID> busMembers = getWayMembers(busWays);
                     
                     Bus_Lines line;

                     std::vector<ezgl::point2d> busNodeCoordsXY;

                     for (int nodeNum = 0; nodeNum < busMembers.size(); nodeNum++) {
                        const OSMNode* currNode = mapData.nodeIdToNodeMap[busMembers[nodeNum]];
                        LatLon nodeCoordsLatLon = getNodeCoords(currNode);
                        ezgl::point2d nodeCoordsXY = ezgl::point2d {x_from_lon(nodeCoordsLatLon.longitude()), y_from_lat(nodeCoordsLatLon.latitude())};
                        busNodeCoordsXY.push_back(nodeCoordsXY);
                     }

                     line.wayPositions = busNodeCoordsXY;

                     // for (unsigned k = 0; k < getTagCount(currRelation); k++) {
                     //    std::string innerKey, innerValue;
                     //    std::tie(innerKey, innerValue) = getTagPair(currRelation, k);

                     //    if (innerKey == "colour") {
                     //       line.color = innerValue;
                     //    }
                     // }

                     mapData.bus.push_back(line);

                  }


               }



            }


         }

      }
   }


}


void drawBusRoutes(ezgl::renderer *g) {

   double currZoomLevel = getZoomLevel(g);
   g -> set_color(ezgl::RED);
   if (currZoomLevel > 10) {


      for (int i = 0; i < mapData.bus.size(); ++i) {
         

         for (int j = 0; j < mapData.bus[i].wayPositions.size(); j++) {
            // && size != 0
            if (j + 1 != mapData.bus[i].wayPositions.size()) {
               g->set_line_width(6);
               g->draw_line(mapData.bus[i].wayPositions[j], mapData.bus[i].wayPositions[j+1]);

            }
         
         }
      }


   }



}



void loadBikeRoutes() {
   for (unsigned i = 0; i < getNumberOfRelations(); i++) {
      const OSMRelation* currRelation = getRelationByIndex(i);

      for (unsigned j = 0; j < getTagCount(currRelation); j++) {
         std::string key, value;
         std::tie(key, value) = getTagPair(currRelation, j);

         if (key == "route" && value == "bicycle") {

            std::vector<TypedOSMID> relationMembers = getRelationMembers(currRelation);
            std::vector<std::string> relationMemberRoles = getRelationMemberRoles(currRelation);

            for (int memberID = 0; memberID < relationMembers.size(); memberID++) {

               if (relationMembers[memberID].type() == TypedOSMID::Way) {

                  if (relationMemberRoles[memberID] != "platform") {
                     
                     const OSMWay* bikeRoutes = mapData.wayIdToWayMap[relationMembers[memberID]];

                     const std::vector<OSMID> bikeMembers = getWayMembers(bikeRoutes);
                     
                     Bike_Route line;

                     std::vector<ezgl::point2d> bikeNodeCoordsXY;

                     for (int nodeNum = 0; nodeNum < bikeMembers.size(); nodeNum++) {
                        const OSMNode* currNode = mapData.nodeIdToNodeMap[bikeMembers[nodeNum]];
                        LatLon nodeCoordsLatLon = getNodeCoords(currNode);
                        ezgl::point2d nodeCoordsXY = ezgl::point2d {x_from_lon(nodeCoordsLatLon.longitude()), y_from_lat(nodeCoordsLatLon.latitude())};
                        bikeNodeCoordsXY.push_back(nodeCoordsXY);
                     }

                     line.wayPositions = bikeNodeCoordsXY;

                     // for (unsigned k = 0; k < getTagCount(currRelation); k++) {
                     //    std::string innerKey, innerValue;
                     //    std::tie(innerKey, innerValue) = getTagPair(currRelation, k);

                     //    if (innerKey == "colour") {
                     //       line.color = innerValue;
                     //    }
                     // }

                     mapData.bike.push_back(line);

                  }


               }



            }


         }

      }
   }


}

void drawBikeRoutes(ezgl::renderer *g) {
      double currZoomLevel = getZoomLevel(g);
      g -> set_color(ezgl::GREEN);
      if (currZoomLevel > 10) {


         for (int i = 0; i < mapData.bike.size(); ++i) {
            

            for (int j = 0; j < mapData.bike[i].wayPositions.size(); j++) {
               // && size != 0
               if (j + 1 != mapData.bike[i].wayPositions.size()) {
                  g->set_line_width(6);
                  g->draw_line(mapData.bike[i].wayPositions[j], mapData.bike[i].wayPositions[j+1]);

               }
            
            }
         }


      }



}