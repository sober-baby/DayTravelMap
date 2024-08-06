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
#include <string>
#include "m1HelperFunctions.h"
#include "m3HelperFunctions.h"
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m4.h"
//Program exit codes
constexpr int SUCCESS_EXIT_CODE = 0;        //Everyting went OK
constexpr int ERROR_EXIT_CODE = 1;          //An error occured
constexpr int BAD_ARGUMENTS_EXIT_CODE = 2;  //Invalid command-line usage

//The default map to load if none is specified
std::string default_map_path = "/cad2/ece297s/public/maps/toronto_canada.streets.bin";

// The start routine of your program (main) when you are running your standalone
// mapper program. This main routine is *never called* when you are running 
// ece297exercise (the unit tests) -- those tests have their own main routine
// and directly call your functions in /libstreetmap/src/ to test them.
// Don't write any code in this file that you want run by ece297exerise -- it 
// will not be called!
int main(int argc, char** argv) {

    std::string map_path;
    if(argc == 1) {
        //Use a default map
        map_path = default_map_path;
    } else if (argc == 2) {
        //Get the map from the command line
        map_path = argv[1];
    } else {
        //Invalid arguments
        std::cerr << "Usage: " << argv[0] << " [map_file_path]\n";
        std::cerr << "  If no map_file_path is provided a default map is loaded.\n";
        return BAD_ARGUMENTS_EXIT_CODE;
    }

    //Load the map and related data structures
    bool load_success = loadMap(map_path);
    if(!load_success) {
        std::cerr << "Failed to load map '" << map_path << "'\n";
        return ERROR_EXIT_CODE;
    }

    std::cout << "Successfully loaded map '" << map_path << "'\n";

    //double time = computePathTravelTime(29.15722898780660799, {47882, 20830, 20829, 20828, 20827, 20826, 20825, 20824, 20823, 20822, 20821, 20820, 20819, 222552, 222553, 222554, 175404, 222557, 54698, 54699, 54697, 169406, 170144, 170145, 169282, 169285, 169286, 169284, 169408, 169409, 90311, 90310, 143591, 143590, 143586, 143597, 143594, 167285, 169377, 169378, 169382, 169383, 169379, 169380, 167284, 214201, 214202, 16});
    // std::cout << time << "s" << std::endl;
    //You can now do something with the map data

    std::vector<DeliveryInf> deliveries;
    std::vector<IntersectionIdx> depots;
    float turn_penalty;
    std::vector<CourierSubPath> result_path;
    bool is_legal;
        
    // deliveries = {DeliveryInf(171961, 41792), DeliveryInf(145152, 151088), DeliveryInf(96058, 57692), DeliveryInf(173265, 66308), DeliveryInf(52222, 48968), DeliveryInf(131184, 61922), DeliveryInf(110624, 130404), DeliveryInf(44815, 103959), DeliveryInf(105931, 111575), DeliveryInf(54071, 111889), DeliveryInf(34725, 105882), DeliveryInf(153323, 71800), DeliveryInf(37717, 36213), DeliveryInf(174923, 92754), DeliveryInf(83337, 123016), DeliveryInf(191528, 179887), DeliveryInf(104028, 161047), DeliveryInf(135783, 47419), DeliveryInf(132416, 171471), DeliveryInf(175707, 157213), DeliveryInf(150803, 105756), DeliveryInf(122449, 155253), DeliveryInf(186424, 13106), DeliveryInf(156304, 54603), DeliveryInf(136971, 140068)};
    // depots = {13, 49589};
    // turn_penalty = 30.000000000;
    // deliveries = {DeliveryInf(42566, 168058)};
    // depots = {35927, 36402};
    // turn_penalty = 30.000000000;
    // deliveries = {DeliveryInf(124331, 156932), DeliveryInf(25964, 156932), DeliveryInf(67812, 156932), DeliveryInf(153404, 97799), DeliveryInf(68424, 91419), DeliveryInf(94361, 91419), DeliveryInf(180613, 151301), DeliveryInf(127593, 64272)};
    //     depots = {14187, 6615, 128524};
    //     turn_penalty = 30.000000000;

    // deliveries = {DeliveryInf(51536, 48633), DeliveryInf(116224, 105642), DeliveryInf(37296, 57573), DeliveryInf(36175, 76961), DeliveryInf(36175, 105160), DeliveryInf(58813, 34544), DeliveryInf(36175, 154104), DeliveryInf(116224, 154214)};
    // depots = {34099, 58669, 99147};
    // turn_penalty = 30.000000000;

        //     deliveries = {DeliveryInf(130308, 168964), DeliveryInf(117529, 54933), DeliveryInf(65839, 168964), DeliveryInf(117349, 54933), DeliveryInf(158825, 54933), DeliveryInf(183286, 116918), DeliveryInf(99424, 100534), DeliveryInf(60610, 96438)};
        // depots = {42980, 124042, 76398};
        // turn_penalty = 30.000000000;
        	
    // result_path = travelingCourier(turn_penalty, deliveries, depots);
        

        // is_legal = courier_path_is_legal(deliveries, depots, result_path);
        // CHECK(is_legal);

    drawMap();
    

    // std::vector<int> path = findPathBetweenIntersections(0.00000000000000000, std::make_pair(262542, 329413));
    //     //<= 2491.42845253009591033
    // double time = computePathTravelTime(0.00000000000000000, path);
    // std::cout << time << std::endl;
    // bool sth = bfsPath(1,2);
    // std::cout << sth << std::endl;

    // std::vector<int> streets = findStreetSegmentsOfIntersection(1);
    // std::cout << streets[0] << streets[1] << std::endl;

    //Clean-up the map data and related data structures
    std::cout << "Closing map\n";
    

    // Final close map includes clearing draw surfaces, thereby avoiding memory leaks
    closeMapFinal();

    return SUCCESS_EXIT_CODE;
}
