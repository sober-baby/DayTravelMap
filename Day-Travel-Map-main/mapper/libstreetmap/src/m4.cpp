#include "m4.h"
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
#include <unordered_set>

//declare all functions

// 1. remove duplicate/leave interest; used in when creating interested matrix
std::vector<IntersectionIdx> removeDuplicatesOfThree(std::vector<IntersectionIdx> pickUps, std::vector<IntersectionIdx> dropOffs, std::vector<IntersectionIdx> depots);

// 2.1 Helperfunction to access time matrix
double findTimeBetweenIntersections(IntersectionIdx in1, IntersectionIdx in2, std::vector<IntersectionIdx>& interestedIntersections);
// 2.2 Helperfunction to access path matrix
std::vector<StreetSegmentIdx> findPathBetweenIntersections(IntersectionIdx in1, IntersectionIdx in2, std::vector<IntersectionIdx>& interestedIntersections);

// 3.1 Helperfunction to check leg ; //Assume pickUps and dropOffs are in order, -1 if not found
std::vector<int> findIndexSpecial(IntersectionIdx in1,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs);
// 3.2 Helperfunction to check leg ; //0 is pickup, 1 is dropoff, -1 if not found
int findTypeSpecial(IntersectionIdx in1,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs);
// 3.3 True function that detects leg
bool testValidPath(std::vector<IntersectionIdx> startMiddleEnd, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs );
std::vector<IntersectionIdx> findBestOne(std::vector<std::vector<IntersectionIdx>> fulls, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs,  std::vector<IntersectionIdx>& interestedIntersections);
// 4. Return all permutations of a list/vec; Used for exhaustion
std::vector<std::vector<IntersectionIdx>> findAllPermutation(std::vector<IntersectionIdx> middle);
std::vector<IntersectionIdx> shiftImprove2ff(std::vector<IntersectionIdx>& fullPathNodes, int level,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections);
std::vector<IntersectionIdx> shiftImproveff(std::vector<IntersectionIdx>& fullPathNodes, int level,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections);

// 5. Helperfunction to access time/path via index, simplies down to pure permu of nodes
std::pair<double,  std::vector<std::vector<StreetSegmentIdx>>> findDirectTimePath(std::vector<IntersectionIdx>& full, std::vector<IntersectionIdx>& interestedIntersections);

// 6. Helperfunction that finds the True best path via exhaustion
std::pair<double, std::vector<IntersectionIdx>> findBestTimePathBetter(IntersectionIdx start,IntersectionIdx end, std::vector<IntersectionIdx>& middle, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections);

std::vector<IntersectionIdx> localImprove(std::vector<IntersectionIdx>& fullPathNodes, int level,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections);
std::vector<IntersectionIdx> localImprove2(std::vector<IntersectionIdx>& fullPathNodes, int level,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections);
std::vector<IntersectionIdx> shiftImprove(std::vector<IntersectionIdx>& fullPathNodes, int level,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections);
std::vector<IntersectionIdx> shiftImprove2(std::vector<IntersectionIdx>& fullPathNodes, int level,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections);
std::vector<IntersectionIdx> twoOptPerturbation(std::vector<IntersectionIdx>& fullPathNode, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections);
std::vector<IntersectionIdx> threeOptPerturbation(std::vector<IntersectionIdx>& fullPathNode, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections);
std::vector<IntersectionIdx> UselessOpt(std::vector<IntersectionIdx>& fullPathNode, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections);
std::vector<IntersectionIdx> twoOptPerturbation2(std::vector<IntersectionIdx>& fullPathNode, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections);
std::vector<IntersectionIdx> threeOptPerturbationTrue(std::vector<IntersectionIdx>& fullPathNode, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections);
std::vector<IntersectionIdx> finalOpt(std::vector<IntersectionIdx>& fullPathNodes,std::vector<IntersectionIdx>pickUps, std::vector<IntersectionIdx>dropOffs, std::vector<IntersectionIdx>interestedIntersections);
//return all intersections of two three vectors
std::vector<IntersectionIdx> removeDuplicatesOfThree(std::vector<IntersectionIdx> pickUps, std::vector<IntersectionIdx> dropOffs, std::vector<IntersectionIdx> depots){
   std::unordered_set<IntersectionIdx> s;
   for(int i = 0; i < dropOffs.size(); i++){
      s.insert(dropOffs[i]);
   }
   for(int i = 0; i < pickUps.size(); i++){
      s.insert(pickUps[i]);
   }
   for(int i = 0; i < depots.size(); i++){
      s.insert(depots[i]);
   }
   std::vector<IntersectionIdx> removedVector(s.begin(), s.end());
   return removedVector;

}

extern GlobalVars mapData;



//helper functions for using the matrix avoid using the for loops too many times; @CHENGSIHAN!!!!!!!! HOW CAN YOU USE 
double findTimeBetweenIntersections(IntersectionIdx in1, IntersectionIdx in2, std::vector<IntersectionIdx>& interestedIntersections){

    return mapData.timeMatrix[mapData.intersectionToMatrixIndex[in1]][mapData.intersectionToMatrixIndex[in2]];


}
//helper functions for using the matrix avoid using the for loops too many times
std::vector<StreetSegmentIdx> findPathBetweenIntersections(IntersectionIdx in1, IntersectionIdx in2, std::vector<IntersectionIdx>& interestedIntersections){

    return mapData.pathMatrix[mapData.intersectionToMatrixIndex[in1]][mapData.intersectionToMatrixIndex[in2]];
            
  
}

//Assume pickUps and dropOffs are in order, -1 if not found
std::vector<int> findIndexSpecial(IntersectionIdx in1,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs){
    std::vector<int> re;
    for(int i = 0; i < pickUps.size(); ++i){
        if(pickUps[i] == in1){
            re.push_back(i);
        }
    }
    for(int i = 0; i < dropOffs.size(); ++i){
        if(dropOffs[i] == in1){
            re.push_back(i);
        }
    }
    return re;
}

//0 is pickup, 1 is dropoff, -1 if not found
int findTypeSpecial(IntersectionIdx in1,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs){
    for(int i = 0; i < pickUps.size(); ++i){
        if(pickUps[i] == in1){
            return 0;
        }
    }
    for(int i = 0; i < dropOffs.size(); ++i){
        if(dropOffs[i] == in1){
            return 1;
        }
    }
    return -1;
}

bool testValidPath(std::vector<IntersectionIdx> startMiddleEnd, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs ){
    // startMIddleEnd must be full path
    if(startMiddleEnd[0]!=startMiddleEnd[startMiddleEnd.size()-1]){
        return false;
    }


    for(int i = 0; i<startMiddleEnd.size(); i++){
        if(findTypeSpecial(startMiddleEnd[i],pickUps,dropOffs)== 1){ //dropoff
            std::vector<int> vtargetIndex = findIndexSpecial(startMiddleEnd[i],pickUps,dropOffs);
            bool thisDrop = false;
            for(int k = 0; k<vtargetIndex.size(); k++){
                //as long as any pick up is detected before
                int targetIndex = vtargetIndex[k];
                for(int j = i-1; j>0; j--){
                    if(dropOffs[targetIndex] == startMiddleEnd[j]){
                        break;
                    }
                    if(pickUps[targetIndex] == startMiddleEnd[j]){
                        thisDrop = true;
                        break;
                    }
                }
            }
            if(thisDrop == false){
                return false;
            }
        }
        if(findTypeSpecial(startMiddleEnd[i],pickUps,dropOffs)== 0){ //pickup
            std::vector<int> vtargetIndex = findIndexSpecial(startMiddleEnd[i],pickUps,dropOffs);
            int targetIndex = vtargetIndex[0];
            bool dropped = false;
            for(int j = i; j<startMiddleEnd.size(); j++){
                if(dropOffs[targetIndex] == startMiddleEnd[j]){
                    dropped = true;
                    break;
                }
            }
            if(dropped == false){
                return false;
            }
        }
    }
    return true;
}

/*
bool testValidPath(std::vector<IntersectionIdx> startMiddleEnd, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs) {
    
    // Convert into pick drop
    std::vector<PickDropDepot> deliveryOrder;

    std::unordered_map<int, int> interesectionToIndex;
        // Dropoffs will be inaccurate as it will be repeated
    // std::cout << startMiddleEnd.size() << "ssdaaasdfa" << std::endl;
    if (startMiddleEnd.empty()) {
        return false;
    }

    for (int p = 1; p < startMiddleEnd.size()-1; ++p) {
        // std::cout << startMiddleEnd[p] << "ssdfa" << std::endl;
       interesectionToIndex[startMiddleEnd[p]] = p;
    }

    for (int i = 1; i < startMiddleEnd.size()-1; ++i) {
        PickDropDepot currIntersection;
        currIntersection.intersectionID = startMiddleEnd[i];
        for (int j = 0; j < pickUps.size(); ++j) {
            if (pickUps[j] == startMiddleEnd[i]) {
                currIntersection.isPickUp = true;
                
                break;
            }
        }
        for (int k = 0; k < dropOffs.size(); ++k) {
            if (dropOffs[k] == startMiddleEnd[i]) {
                currIntersection.isDropOff = true;
                currIntersection.respPickUp = pickUps[k];
                break;
            }
        }
        deliveryOrder.push_back(currIntersection);
    }

    // delivery order is now without the two depots
    for (int m = 0; m < deliveryOrder.size(); m++) {
        PickDropDepot currDelivery = deliveryOrder[m];
        
        currDelivery.visited = true;
        if (currDelivery.isDropOff) {
        
            if (!deliveryOrder[interesectionToIndex[currDelivery.respPickUp]].visited) {
                return false;
            }
        }
    }

    return true;
}


bool testValidPath(std::vector<IntersectionIdx> startMiddleEnd, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs) {

    

        // Convert into pick drop
    std::vector<PickDropDepot> deliveryOrder; 


    for (int i = 1; i < startMiddleEnd.size()-1; ++i) {
        PickDropDepot currIntersection;
        currIntersection.intersectionID = startMiddleEnd[i];
        for (int j = 0; j < pickUps.size(); ++j) {
            if (pickUps[j] == startMiddleEnd[i]) {
                currIntersection.isPickUp = true;
                
                break;
            }
        }
        for (int k = 0; k < dropOffs.size(); ++k) {
            if (dropOffs[k] == startMiddleEnd[i]) {
                currIntersection.isDropOff = true;
                break;
            }
        }
        deliveryOrder.push_back(currIntersection);
    }

    for (int i = startMiddleEnd.size()-2; i >= 0; i--) {
        deliveryOrder[i].visited = true;
       if (deliveryOrder[i].isDropOff) {
           // Find all the pick ups that have this dropOff
           std::vector<IntersectionIdx> allPickUps;
           for (int v = 0; v < pickUps.size(); v++) {
               if (pickUps[v] == deliveryOrder[i].intersectionID)) {
                   allPickUps.push_back(pickUps[v]);
               }
           }

           
       }
    }

}
*/
std::vector<std::vector<IntersectionIdx>> findAllPermutation(std::vector<IntersectionIdx> middle){
    std::vector<std::vector<IntersectionIdx>> re;
    if (middle.empty()){
        re.push_back({});
        return re;
    }
    for(int i  = 0; i<middle.size(); ++i){
        std::vector<IntersectionIdx> temp;
        temp.push_back(middle[i]);
        std::vector<IntersectionIdx> rest = middle;
        rest.erase(rest.begin()+i);
        std::vector<std::vector<IntersectionIdx>> inducted = findAllPermutation(rest);
        for(int j = 0; j<inducted.size(); ++j){
            std::vector<IntersectionIdx> pre = temp;
            for(int k = 0; k<inducted[j].size();++k) {
                pre.push_back(inducted[j][k]);
            }
                
            re.push_back(pre);
        }
    }
    return re;
}

std::pair<double,  std::vector<std::vector<StreetSegmentIdx>>> findDirectTimePath(std::vector<IntersectionIdx>& full, std::vector<IntersectionIdx>& interestedIntersections){


    //assume full is a valid input
    double currtime = 0;
    std::vector<std::vector<StreetSegmentIdx>> currPath;
    for(int i = 0; i<full.size()-1;++i){
        // std::cout << full.size() << std::endl;
        double time = findTimeBetweenIntersections(full[i],full[i+1],interestedIntersections);
        currtime += time;
        // std::cout << full[i] << " and " << full[i+1] << std::endl;
        // std::cout << time << std::endl;
        currPath.push_back(findPathBetweenIntersections(full[i],full[i+1],interestedIntersections));
    }
    std::pair<double,  std::vector<std::vector<StreetSegmentIdx>>> pair = std::make_pair(currtime, currPath);
    return std::pair(currtime, currPath);
}


std::pair<double, std::vector<IntersectionIdx>> findBestTimePathBetter(IntersectionIdx start,IntersectionIdx end, std::vector<IntersectionIdx>& middle, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections){
    // std::cout << "Used" << std::endl;
    std::vector<std::vector<IntersectionIdx>> poss = findAllPermutation(middle);
    double besttime = 999999;
    std::vector<std::vector<StreetSegmentIdx>> finalPath;
    std::vector<IntersectionIdx> finalNodes;
    double currtime = 0;

    for(int i = 0; i<poss.size(); i++){
        std::vector<IntersectionIdx> newMiddle = poss[i];
        std::vector<IntersectionIdx> newFull;
        newFull.push_back(start);
        for(int j = 0; j<newMiddle.size(); j++){
            newFull.push_back(newMiddle[j]);
        }
        newFull.push_back(end);
        if(testValidPath(newFull,pickUps,dropOffs)){ //now invest the new path
        
            currtime = findDirectTimePath(newFull,interestedIntersections).first;
            if(currtime < besttime){
                besttime = currtime;
                finalNodes = poss[i];
            }
        }
    }
    return std::make_pair(besttime, finalNodes);
}


// The majority of the code is not yet upgraded(consider double layer/probabilty double layer, etc.)'
// A 2-opt algorithm is applied





// The majority of the code is not yet upgraded(consider double layer/probabilty double layer, etc.)'
// A 2-opt algorithm is applied
std::pair<double,  std::vector<IntersectionIdx>> Jms(int prevCount, IntersectionIdx depotPos, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections) {
    auto start = std::chrono::high_resolution_clock::now(); 

    // Heuristic - Greedy Algorithms
    int numInterestedInter = interestedIntersections.size();
    double finalTime;
    // for (int depotNum = 0; depotNum < depots.size(); ++depotNum) {
    // Search from each depot --- can maybe use parallel for this one?
    IntersectionIdx initPos = depotPos;
    IntersectionIdx currPos = depotPos;
    // Initialize an array of only pickups and dropoffs
    int chosenPos;
    int prevChosen;
    
    std::vector<std::vector<IntersectionIdx>> allPossNodes;
    double trials = 2;
    std::vector<IntersectionIdx> pickUpAndDropOffs = pickUps;
    /*
    for(int i = 0; i< trials; i++){
        pickUpAndDropOffs = pickUps;
        std::vector<IntersectionIdx> fullPathNodes;
        fullPathNodes.push_back(depotPos);
        while (!pickUpAndDropOffs.empty()) {
            

            double bestNextTime = 999999996;
            double secondNextTime = 999999997;
            double thirdNextTime = 999999998;
            double fourthNextTime = 999999999;
            StreetSegmentIdx bestNextNode = -1;
            StreetSegmentIdx secondNextNode = -1;
            StreetSegmentIdx thirdNextNode = -1;
            StreetSegmentIdx fourthNextNode = -1;
            

            IntersectionIdx currPos = depotPos;
            IntersectionIdx nextPos; //the one we look at, just put here preallocate
            double nextTime;
            // Access all the possible next positions
            for (int availPos = 0; availPos < pickUpAndDropOffs.size(); availPos++) {
                // Accessing the cost from current position to the nearest legal position 
                nextPos = pickUpAndDropOffs[availPos]; // next pos
                nextTime = findTimeBetweenIntersections(currPos,nextPos,interestedIntersections);
                if(nextTime <= bestNextTime){
                    fourthNextNode = thirdNextNode;
                    fourthNextTime = thirdNextTime;
                    thirdNextNode = secondNextNode;
                    thirdNextTime = secondNextTime;
                    secondNextNode = bestNextNode;
                    secondNextTime = bestNextTime;
                    bestNextNode = nextPos;
                    bestNextTime = nextTime;
                }
                else if(nextTime <= secondNextTime){
                    fourthNextNode = thirdNextNode;
                    fourthNextTime = thirdNextTime;
                    thirdNextNode = secondNextNode;
                    thirdNextTime = secondNextTime;
                    secondNextNode = nextPos;
                    secondNextTime = nextTime;
                }
                else if(nextTime <= thirdNextTime){
                    fourthNextNode = thirdNextNode;
                    fourthNextTime = thirdNextTime;
                    thirdNextNode = nextPos;
                    thirdNextTime = nextTime;
                }
                 else if(nextTime <= fourthNextTime){
                    fourthNextNode = nextPos;
                    fourthNextTime = nextTime;
                }
            }
            
            IntersectionIdx chosenPos;
            int decision = rand()%10;
            if(decision<10){
                chosenPos = bestNextNode;
            }
            else if (decision<8){
                chosenPos = secondNextNode;
            }
            else if (decision<9){
                chosenPos = thirdNextNode;
            }
            else if (decision<10){
                chosenPos = fourthNextNode;
            }
            if(chosenPos == -1){
                chosenPos = bestNextNode;
            }


            // Update valid positions
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
            fullPathNodes.push_back(chosenPos);
        }
        fullPathNodes.push_back(depotPos);
        allPossNodes.push_back(fullPathNodes);
    }
    */

    
    std::vector<IntersectionIdx> fullPathNodes;
    fullPathNodes.push_back(depotPos);
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


        finalTime += bestNextPosTime;
        fullPathNodes.push_back(chosenPos);


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
                someTime += mapData.timeMatrix[i][j];
            }
        }
    }

    // Need to add from the last intersection back to the depot
    finalTime += someTime;
    fullPathNodes.push_back(depotPos);
    allPossNodes.push_back(fullPathNodes);
    
    
   
    std::vector<IntersectionIdx> trueBestNodePath = allPossNodes[0];
    std::vector<IntersectionIdx> currBestNodePath;
    double trueBestTime = 999999;
    double currBestTime;
    // std::cout <<"firtst!!!!!!"<<allPossNodes.size() <<std::endl;
    /*
    for(int sel = 0; sel<allPossNodes.size(); sel++){
        //printout all nodes everyloop,testing only
        for(int k = 0; k<allPossNodes[sel].size(); k++ ){
            std::cout<< allPossNodes[sel][k]; 
            std::cout<< " "; 
        }
        //std::cout <<"reached!!!!!!"<<allPossNodes[sel].size() <<std::endl;
        currBestNodePath = localImprove(allPossNodes[sel],7, pickUps, dropOffs, interestedIntersections);
        currBestTime = findDirectTimePath(currBestNodePath,interestedIntersections).first;
        if(currBestTime<trueBestTime){
            trueBestTime = currBestTime;
            trueBestNodePath = currBestNodePath;
            
        }
        std::cout <<"reachedend!!!!!!"<<std::endl;
    }
    */
    // fullPathNodes = UselessOpt(fullPathNodes, pickUps, dropOffs, interestedIntersections);


 
    fullPathNodes = twoOptPerturbation(fullPathNodes, pickUps, dropOffs, interestedIntersections);
    fullPathNodes = twoOptPerturbation2(fullPathNodes, pickUps, dropOffs, interestedIntersections);

    
    // fullPathNodes = threeOptPerturbation(fullPathNodes, pickUps, dropOffs, interestedIntersections);
    fullPathNodes = UselessOpt(fullPathNodes, pickUps, dropOffs, interestedIntersections);
    if(fullPathNodes.size()>30){
        if(fullPathNodes.size()>=250){
            while (true) {
                fullPathNodes = shiftImprove2(fullPathNodes,6,pickUps,dropOffs,interestedIntersections);
                fullPathNodes = localImprove(fullPathNodes,4,pickUps,dropOffs,interestedIntersections);
                auto stop = std::chrono::high_resolution_clock::now(); 
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
                if (duration.count() + prevCount> 43) {
                    break;
                }
            }
        }
        else{
            fullPathNodes = localImprove(fullPathNodes,6,pickUps,dropOffs,interestedIntersections);
            fullPathNodes = shiftImprove(fullPathNodes,8,pickUps,dropOffs,interestedIntersections);
            while (true) {
                for (int sps = 3; sps< fullPathNodes.size(); sps+=2){
                    fullPathNodes = shiftImprove2(fullPathNodes,sps,pickUps,dropOffs,interestedIntersections);
                }
                for (int sps = 4; sps< fullPathNodes.size(); sps+=2){
                    fullPathNodes = shiftImprove2(fullPathNodes,sps,pickUps,dropOffs,interestedIntersections);
                }
                fullPathNodes = localImprove(fullPathNodes,5,pickUps,dropOffs,interestedIntersections);
                fullPathNodes = localImprove(fullPathNodes,4,pickUps,dropOffs,interestedIntersections);
                auto stop = std::chrono::high_resolution_clock::now(); 
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
                if (duration.count() + prevCount> 43) {
                    break;
                }
            }
        }
    }
    
    

    
    finalTime = findDirectTimePath(fullPathNodes, interestedIntersections).first;
    // Update depot best time

    
        /*
        //printout all nodes everyloop,testing only
        for(int k = 0; k<fullPathNodes.size(); k++ ){
            std::cout<< fullPathNodes[k]; 
            std::cout<< " "; 
        }
        std::cout << "end66666" << std::endl;
        */
        //ece297exercise 4 --run_test "*Extreme*"
        //ece297exercise 4 --run_test "*Hard*"
    

    /*
    for(int k = 0; k<fullPathNodes.size(); k++ ){
        std::cout<< fullPathNodes[k]; 
        std::cout<< " "; 
    }
    std::cout << "end66666" << std::endl;
    */
    


    /*
    double nfinalTime = findDirectTimePath(fullPathNodes,interestedIntersections).first;
    std::vector<std::vector<StreetSegmentIdx>> nodeFinalPath = findDirectTimePath(fullPathNodes,interestedIntersections).second;
    std::cout << "this best is " << nfinalTime << std::endl;
    return std::make_pair(nfinalTime,nodeFinalPath);
    */
    // Now we use the 2-opt algorithm



    return std::make_pair(finalTime, fullPathNodes);
}



std::vector<CourierSubPath> travelingCourier(const float turn_penalty, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots) {
    auto start = std::chrono::high_resolution_clock::now(); 
        

       


    // Depots: Valid starting position and end positions, we can choose any depots as starting point but it must return to the same spot

    // We will first try exhaustive algorithm
    

    // First, Compute all the paths between all interested intersections by the modified djikstra algorithm

    // Initialize a std vector of intersections we are interested in
    std::vector<IntersectionIdx> pickUps, dropOffs;
    std::vector<IntersectionIdx> interestedIntersections;
    std::vector<IntersectionIdx> pickUpAndDropOffs;
    // This configuration can be changed to grouping by deliveries
    // Guaranteed to have at least one delivery and depot
    
    // We need to get unique intersections
    for (int deliveryNum = 0; deliveryNum < deliveries.size(); ++deliveryNum) {
        pickUps.push_back(deliveries[deliveryNum].pickUp);
        dropOffs.push_back(deliveries[deliveryNum].dropOff);

        // pickUpAndDropOffs.push_back(deliveries[deliveryNum].pickUp);
        // pickUpAndDropOffs.push_back(deliveries[deliveryNum].dropOff);        
    }

    // Remove redundant points for the matrix
    interestedIntersections = removeDuplicatesOfThree(pickUps, dropOffs, depots);


    
    int numInterestedInter = interestedIntersections.size();
    mapData.timeMatrix.resize(numInterestedInter);
    mapData.pathMatrix.resize(numInterestedInter);

    for (int i = 0; i < numInterestedInter; ++i) {
        mapData.timeMatrix[i].resize(numInterestedInter);
        mapData.pathMatrix[i].resize(numInterestedInter);
    }

    #pragma omp parallel for
    for (int interId = 0; interId < interestedIntersections.size(); interId++) {
        // Do multi-djikstra to compute all the path travel time
        std::vector<double> time;
        std::vector<std::vector<StreetSegmentIdx>> path;
        // std::cout<< "multi djistra" << interestedIntersections[interId] << std::endl;
        std::tie(time, path) = multiTargetBfsPath(turn_penalty, interestedIntersections[interId], interestedIntersections);
        mapData.timeMatrix[interId] = time;
        mapData.pathMatrix[interId] = path;
       
    }
    // Now both matrix for mapData is usable


    for (int p = 0; p < interestedIntersections.size(); ++p) {
        mapData.intersectionToMatrixIndex[interestedIntersections[p]] = p;
    }


    // Exhaustive Algorithm
    // Check every possible route, first start at a depot, then we have pick up locations to choose from, then after pick ups we add drop offs to the available positions


    // Heuristic - Greedy Algorithms
    double overallBestTime = 999999999;
    std::vector<IntersectionIdx> finalPathNode;
    int size;
    if (depots.size() > 5) {
        size = 5;
    } else {
        size = depots.size();
    }
    

    auto stop = std::chrono::high_resolution_clock::now(); 
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start); 

    #pragma omp parallel for
    for (int depotNum = 0; depotNum < size; ++depotNum) {

        double depotBestTime;
        std::vector<IntersectionIdx> depotBestPathNode;
        std::tie(depotBestTime, depotBestPathNode) = Jms(duration.count(), depots[depotNum], pickUps, dropOffs, interestedIntersections);

        #pragma omp critical
        if (depotBestTime < overallBestTime) {
            overallBestTime = depotBestTime;
            finalPathNode = depotBestPathNode;
        }

    }

    //Assume we use the path/depot and improve on that 
    //std::cout << "Final Path Time: " <<  findDirectTimePath(finalPathNode, interestedIntersections).first << std::endl;




    // Get courier subpath from list of paths
    std::vector<CourierSubPath> outputCourierPath2;
    for (int i = 0; i < finalPathNode.size() -1 ; ++i) {
        CourierSubPath currPath;
        currPath.intersections = std::make_pair(finalPathNode[i], finalPathNode[i+1]);
        currPath.subpath = findPathBetweenIntersections(finalPathNode[i], finalPathNode[i+1], interestedIntersections);
        outputCourierPath2.push_back(currPath);
    }

    std::vector<CourierSubPath> outputCourierPath;
    // std::vector<std::vector<StreetSegmentIdx>> finalPath;
    // for (int i = 0; i < outputCourierPath2.size(); ++i) {
    //     CourierSubPath currPath = outputCourierPath2[i];
    //     finalPath.push_back(currPath.subpath);
    // }

    // for (int i = 0; i < finalPath.size(); ++i) {
    //     if (finalPath[i].empty()) {
    //         std::cout << "Something" << std::endl;
    //     }
    //     int startPos = finalPath[i][0];
    //     int endPos = finalPath[i][finalPath[i].size()-1];
    //     CourierSubPath currPath;
        
    //     int validIntersectionFrom, validIntersectionTo;

    //     bool isInterStartPos = false;
    //     for (int m = 0; m < numInterestedInter; m++) {
    //         if (mapData.street_segments[startPos].from == interestedIntersections[m]) {
    //             validIntersectionFrom = mapData.street_segments[startPos].from;
    //             isInterStartPos = true;
    //             break;
    //         } 
    //     }

    //     if(!isInterStartPos) {
    //         validIntersectionFrom = mapData.street_segments[startPos].to;
    //     }

    //     bool isInterEndPos = false;
    //     for (int m = 0; m < numInterestedInter; m++) {
    //         if (mapData.street_segments[endPos].from == interestedIntersections[m]) {
    //             validIntersectionTo = mapData.street_segments[endPos].from;
    //             isInterEndPos = true;
    //             break;
    //         }        
    //     }

    //     if(!isInterEndPos) {
    //         validIntersectionTo = mapData.street_segments[endPos].to;
    //     }

    //     // std::cout << "Final Path First Intersection is " << validIntersectionFrom;
        



    //     // std::cout << "Final Path Last Intersection is " << validIntersectionTo ;



    //     currPath.intersections = std::make_pair(validIntersectionFrom, validIntersectionTo);
    //     currPath.subpath = finalPath[i];
        
    //     // May need to clear for test cases?
    //     outputCourierPath.push_back(currPath);
    // }








    // If all possibilities are searched, return empty vector
    return outputCourierPath2;
}




std::vector<std::pair<int, int>> generateUniquePairs(int n) {
    std::vector<std::pair<int, int>> pairs;
    for (int i = 1; i <= n-1; ++i) {
        for (int j = i + 1; j <= n-1; ++j) {
            pairs.emplace_back(i,j);
        }
    }

    return pairs;
}




std::vector<std::vector<IntersectionIdx>> cutIntoPieces(std::vector<IntersectionIdx>& fullPathNode,std::vector<IntersectionIdx>& cuts){
    //assume cuts is ordered and valid, 
    std::vector<std::vector<IntersectionIdx>> re;
    std::vector<IntersectionIdx> temp;
    int count = 0;
    for(int i = 0; i<fullPathNode.size(); i++){
        temp.push_back(fullPathNode[i]);
        if(cuts[count] == i){
            count ++;
            re.push_back(temp);
            temp = {};
        }
    }
    re.push_back(temp);
    return re;
}

std::vector<IntersectionIdx> joinPieces(std::vector<std::vector<IntersectionIdx>> pieces){
    std::vector<IntersectionIdx> re;
    std::vector<IntersectionIdx> currPiece;
    for(int i = 0; i<pieces.size(); i++){
        currPiece = pieces[i];
        for(int j = 0; j < currPiece.size();j++){
            re.push_back(currPiece[j]);
        }
    }
    return re;
}

std::vector<IntersectionIdx> reversePiece(std::vector<IntersectionIdx> piece){
    std::vector<IntersectionIdx> re;
    for(int i = 0; i<piece.size(); i++){
        re.push_back(piece[piece.size()-i-1]);
    }
    return re;
}

std::vector<std::vector<IntersectionIdx>> findAllReverJoins(std::vector<std::vector<IntersectionIdx>> pieces){
    std::vector<std::vector<IntersectionIdx>> re;

    if(pieces.size()==1){
        re.push_back(pieces[0]);
        re.push_back(reversePiece(pieces[0]));
        return re;
    }
    else{
        std::vector<std::vector<IntersectionIdx>> copy = pieces;
        copy.erase(copy.begin());
        std::vector<std::vector<IntersectionIdx>> rest = findAllReverJoins(copy);
        std::vector<std::vector<IntersectionIdx>> temp;
        for(int i=0; i< rest.size(); i++){
            temp = {};
            temp.push_back(pieces[0]);
            temp.push_back(rest[i]);
            re.push_back(joinPieces(temp));
            temp = {};
            temp.push_back(reversePiece(pieces[0]));
            temp.push_back(rest[i]);
            re.push_back(joinPieces(temp));
        }
    }
    return re;
}

std::vector<std::vector<IntersectionIdx>> findAllPermuJoins(std::vector<std::vector<IntersectionIdx>> pieces){
    std::vector<std::vector<IntersectionIdx>> re;

    std::vector<int> order;
    for(int i = 0; i<pieces.size();i++){
        order.push_back(i);
    }
    std::vector<std::vector<int>> possibleOrders = findAllPermutation(order);

    std::vector<std::vector<IntersectionIdx>> temp;
    std::vector<std::vector<IntersectionIdx>> temp2;
    for(int i = 0; i<possibleOrders.size();i++){
        temp = {};
        for(int j = 0; j<pieces.size();j++){
            temp.push_back(pieces[possibleOrders[i][j]]);
        }
        temp2 = findAllReverJoins(temp);
        for(int j = 0; j<temp2.size();j++){
            re.push_back(temp2[j]);
        }
    }

    return re;
}


std::vector<IntersectionIdx> findBestOne(std::vector<std::vector<IntersectionIdx>> fulls, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs,  std::vector<IntersectionIdx>& interestedIntersections){
    double bestTime = 999999999;


    std::vector<IntersectionIdx> bestPath;


    for (int i = 0; i < fulls.size(); ++i){
        /*
        std::vector<IntersectionIdx> fullkk = fulls[i];
        for(int s = 0; s < fullkk.size(); ++s){
            std::cout<<fullkk[s] << " ";
        }
        std::cout << 1 << std::endl;
        */

        //std::cout << "not in?"<<std::endl;
        if(testValidPath(fulls[i], pickUps, dropOffs)){
            /*std::cout << "in"<<std::endl;*/
            double currTime = findDirectTimePath(fulls[i], interestedIntersections).first;
            //std::cout << "in"<<std::endl;
            if(bestTime > currTime){  
                bestTime = currTime;
                bestPath = fulls[i];
            }
        }
    }
    return bestPath;
}

std::vector<IntersectionIdx> findBestSecond(std::vector<std::vector<IntersectionIdx>> fulls, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs,  std::vector<IntersectionIdx>& interestedIntersections){
    double bestTime = 999999999;


    std::vector<IntersectionIdx> bestPath;
    std::vector<std::vector<IntersectionIdx>> bestPathlist;

    for (int i = 0; i < fulls.size(); ++i){
        /*
        std::vector<IntersectionIdx> fullkk = fulls[i];
        for(int s = 0; s < fullkk.size(); ++s){
            std::cout<<fullkk[s] << " ";
        }
        std::cout << 1 << std::endl;
        */

        //std::cout << "not in?"<<std::endl;
        if(testValidPath(fulls[i], pickUps, dropOffs)){
            /*std::cout << "in"<<std::endl;*/
            double currTime = findDirectTimePath(fulls[i], interestedIntersections).first;
            //std::cout << "in"<<std::endl;
            if(bestTime > currTime){  
                bestTime = currTime;
                bestPath = fulls[i];
            }
            if(bestTime* 1.01 > currTime){  
                bestPathlist.push_back(fulls[i]);
            }
        }
    }
    int index = rand()% bestPathlist.size();
    int choose = rand() % 20;
    if(choose == 0){
        return bestPathlist[index];
    }
    return bestPath;
}

/* not used in this semester
std::vector<IntersectionIdx> threeOptPerturbation(std::vector<IntersectionIdx>& fullPathNode, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections) {

    //my code

    
    std::vector<std::vector<IntersectionIdx>> temp2;

    //select 3 pieces to cut, suppose fullnodesppath.len = n+1, cut has n choices
    
    for(int x = 0; x<fullPathNode.size()-1; x++){
        for(int y = x; y<fullPathNode.size()-1; y++){
            for(int z = y; z<fullPathNode.size()-1; z++){
                //ex. x = 3, cut between 3 and 4
                if(x!=y&&y!=z&&z!=x){

                    std::vector<IntersectionIdx> cuts;
                    cuts.push_back(x);
                    cuts.push_back(y);
                    cuts.push_back(z);
                    std::vector<std::vector<IntersectionIdx>> pieces = cutIntoPieces(fullPathNode,cuts);

                    std::vector<std::vector<IntersectionIdx>> temp;
                    
                    temp.push_back(pieces[1]);
                    temp.push_back(pieces[2]);
                    
                    std::vector<std::vector<IntersectionIdx>> middles = findAllPermuJoins(temp);
                    std::vector<std::vector<IntersectionIdx>> wow;
                    std::vector<std::vector<IntersectionIdx>> wows;
                    for(int sp = 0; sp <middles.size(); sp++){
                        wow = {};
                        wow.push_back(pieces[0]);
                        wow.push_back(middles[sp]);
                        wow.push_back(pieces[3]);

                        std::vector<IntersectionIdx> fullkk = joinPieces(wow);
                        
                        wows.push_back(joinPieces(wow));
                    }
                    

                    temp2.push_back(findBestOne(wows, pickUps, dropOffs, interestedIntersections));

                }
            }
        }
    }

    return findBestOne(temp2,pickUps,dropOffs,interestedIntersections);
}
*/
std::vector<IntersectionIdx> threeOptPerturbation(std::vector<IntersectionIdx>& fullPathNode, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections) {

    //my code

    

    //select 3 pieces to cut, suppose fullnodesppath.len = n+1, cut has n choices
    #pragma omp parallel for
    for(int x = 0; x<fullPathNode.size()-1; x+=4){
        for(int y = x+6; y<fullPathNode.size()-1; y+=4){
            for(int z = y+7; z<fullPathNode.size()-1; z+=5){
                //ex. x = 3, cut between 3 and 4
                double current_time = findDirectTimePath(fullPathNode,interestedIntersections).first;
                if(x!=y&&y!=z&&z!=x&&(y-x)>fullPathNode.size()/4&&(z-y)>fullPathNode.size()/4){//extra conditions to limit the time


                    std::vector<IntersectionIdx> temp3;

                    for(int k = 0 ; k<=x; k++){
                        temp3.push_back(fullPathNode[k]);
                    }
                    for(int k =y ; k>=x+1;k--){
                        temp3.push_back(fullPathNode[k]);
                    }
                    for(int k =z ; k>=y+1;k--){
                        temp3.push_back(fullPathNode[k]);
                    }
                    for(int k = z + 1; k<fullPathNode.size(); k++){
                        temp3.push_back(fullPathNode[k]);
                    }
                    temp3 = UselessOpt(temp3, pickUps, dropOffs, interestedIntersections);
                    if (findDirectTimePath(temp3,interestedIntersections).first<current_time && testValidPath(temp3,pickUps,dropOffs)){
                        //std::cout<<findDirectTimePath(temp,interestedIntersections).first<< "yeah"<<current_time<<std::endl;
                        fullPathNode = temp3;
                        current_time = findDirectTimePath(temp3,interestedIntersections).first;
                    }
                    else if (!testValidPath(temp3,pickUps,dropOffs)){
                        //std::cout<<"invalid"<<std::endl;
                    }
                    else{
                        //std::cout<<"not godd"<<std::endl;
                    }


                    std::vector<IntersectionIdx> temp11;

                    for(int k = 0 ; k<=x; k++){
                        temp11.push_back(fullPathNode[k]);
                    }
                    for(int k =z ; k>=y+1;k--){
                        temp11.push_back(fullPathNode[k]);
                    }
                    for(int k =x+1 ; k<=y;k++){
                        temp11.push_back(fullPathNode[k]);
                    }
                    for(int k = z + 1; k<fullPathNode.size(); k++){
                        temp11.push_back(fullPathNode[k]);
                    }
                    temp11 = UselessOpt(temp11, pickUps, dropOffs, interestedIntersections);
                    if (findDirectTimePath(temp11,interestedIntersections).first<current_time && testValidPath(temp11,pickUps,dropOffs)){
                        //std::cout<<findDirectTimePath(temp,interestedIntersections).first<< "yeah"<<current_time<<std::endl;
                        fullPathNode = temp11;
                        current_time = findDirectTimePath(temp11,interestedIntersections).first;
                    }
                    else if (!testValidPath(temp11,pickUps,dropOffs)){
                        //std::cout<<"invalid"<<std::endl;
                    }
                    else{
                        //std::cout<<"not godd"<<std::endl;
                    }

                    std::vector<IntersectionIdx> temp12;

                    for(int k = 0 ; k<=x; k++){
                        temp12.push_back(fullPathNode[k]);
                    }

                    for(int k =y+1 ; k<=z;k++){
                        temp12.push_back(fullPathNode[k]);
                    }
                    for(int k =y ; k>=x+1;k--){
                        temp12.push_back(fullPathNode[k]);
                    }

                    for(int k = z + 1; k<fullPathNode.size(); k++){
                        temp12.push_back(fullPathNode[k]);
                    }
                    temp12 = UselessOpt(temp12, pickUps, dropOffs, interestedIntersections);
                    if (findDirectTimePath(temp12,interestedIntersections).first<current_time && testValidPath(temp12,pickUps,dropOffs)){
                        //std::cout<<findDirectTimePath(temp,interestedIntersections).first<< "yeah"<<current_time<<std::endl;
                        fullPathNode = temp12;
                        current_time = findDirectTimePath(temp12,interestedIntersections).first;
                    }
                    else if (!testValidPath(temp12,pickUps,dropOffs)){
                        //std::cout<<"invalid"<<std::endl;
                    }
                    else{
                        //std::cout<<"not godd"<<std::endl;
                    }

                    std::vector<IntersectionIdx> temp13;

                    for(int k = 0 ; k<=x; k++){
                        temp13.push_back(fullPathNode[k]);
                    }
                    for(int k =z ; k>=y+1;k--){
                        temp13.push_back(fullPathNode[k]);
                    }
                    for(int k =y ; k>=x+1;k--){
                        temp13.push_back(fullPathNode[k]);
                    }
                    for(int k = z + 1; k<fullPathNode.size(); k++){
                        temp13.push_back(fullPathNode[k]);
                    }
                    temp13 = UselessOpt(temp13, pickUps, dropOffs, interestedIntersections);
                    if (findDirectTimePath(temp13,interestedIntersections).first<current_time && testValidPath(temp13,pickUps,dropOffs)){
                        //std::cout<<findDirectTimePath(temp,interestedIntersections).first<< "yeah"<<current_time<<std::endl;
                        fullPathNode = temp13;
                        current_time = findDirectTimePath(temp13,interestedIntersections).first;
                    }
                    else if (!testValidPath(temp13,pickUps,dropOffs)){
                        //std::cout<<"invalid"<<std::endl;
                    }
                    else{
                        //std::cout<<"not godd"<<std::endl;
                    }



                }
            }
        }
    }

    return fullPathNode;
}

std::vector<IntersectionIdx> threeOptPerturbationTrue(std::vector<IntersectionIdx>& fullPathNode, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections) {

    //my code

    //select 3 pieces to cut, suppose fullnodesppath.len = n+1, cut has n choices
    #pragma omp parallel for
    for(int x = 0; x<fullPathNode.size()-1; x+=2){
        for(int y = x+1; y<fullPathNode.size()-1; y+=2){
            for(int z = y+2; z<fullPathNode.size()-1; z+=5){
                //ex. x = 3, cut between 3 and 4
                double current_time = findDirectTimePath(fullPathNode,interestedIntersections).first;
                if(x!=y&&y!=z&&z!=x&&(y-x)>fullPathNode.size()/4&&(z-y)>fullPathNode.size()/4){//extra conditions to limit the time


                    std::vector<IntersectionIdx> temp3;

                    for(int k = 0 ; k<=x; k++){
                        temp3.push_back(fullPathNode[k]);
                    }
                    for(int k =y ; k>=x+1;k--){
                        temp3.push_back(fullPathNode[k]);
                    }
                    for(int k =z ; k>=y+1;k--){
                        temp3.push_back(fullPathNode[k]);
                    }
                    for(int k = z + 1; k<fullPathNode.size(); k++){
                        temp3.push_back(fullPathNode[k]);
                    }
                    temp3 = UselessOpt(temp3, pickUps, dropOffs, interestedIntersections);
                    if (findDirectTimePath(temp3,interestedIntersections).first<current_time && testValidPath(temp3,pickUps,dropOffs)){
                        //std::cout<<findDirectTimePath(temp,interestedIntersections).first<< "yeah"<<current_time<<std::endl;
                        fullPathNode = temp3;
                        current_time = findDirectTimePath(temp3,interestedIntersections).first;
                    }
                    


                    std::vector<IntersectionIdx> temp11;

                    for(int k = 0 ; k<=x; k++){
                        temp11.push_back(fullPathNode[k]);
                    }
                    for(int k =z ; k>=y+1;k--){
                        temp11.push_back(fullPathNode[k]);
                    }
                    for(int k =x+1 ; k<=y;k++){
                        temp11.push_back(fullPathNode[k]);
                    }
                    for(int k = z + 1; k<fullPathNode.size(); k++){
                        temp11.push_back(fullPathNode[k]);
                    }
                    temp11 = UselessOpt(temp11, pickUps, dropOffs, interestedIntersections);
                    if (findDirectTimePath(temp11,interestedIntersections).first<current_time && testValidPath(temp11,pickUps,dropOffs)){
                        //std::cout<<findDirectTimePath(temp,interestedIntersections).first<< "yeah"<<current_time<<std::endl;
                        fullPathNode = temp11;
                        current_time = findDirectTimePath(temp11,interestedIntersections).first;
                    }
                   

                    std::vector<IntersectionIdx> temp12;

                    for(int k = 0 ; k<=x; k++){
                        temp12.push_back(fullPathNode[k]);
                    }

                    for(int k =y+1 ; k<=z;k++){
                        temp12.push_back(fullPathNode[k]);
                    }
                    for(int k =y ; k>=x+1;k--){
                        temp12.push_back(fullPathNode[k]);
                    }

                    for(int k = z + 1; k<fullPathNode.size(); k++){
                        temp12.push_back(fullPathNode[k]);
                    }
                    temp12 = UselessOpt(temp12, pickUps, dropOffs, interestedIntersections);
                    if (findDirectTimePath(temp12,interestedIntersections).first<current_time && testValidPath(temp12,pickUps,dropOffs)){
                        //std::cout<<findDirectTimePath(temp,interestedIntersections).first<< "yeah"<<current_time<<std::endl;
                        fullPathNode = temp12;
                        current_time = findDirectTimePath(temp12,interestedIntersections).first;
                    }
                    

                    std::vector<IntersectionIdx> temp13;

                    for(int k = 0 ; k<=x; k++){
                        temp13.push_back(fullPathNode[k]);
                    }
                    for(int k =z ; k>=y+1;k--){
                        temp13.push_back(fullPathNode[k]);
                    }
                    for(int k =y ; k>=x+1;k--){
                        temp13.push_back(fullPathNode[k]);
                    }
                    for(int k = z + 1; k<fullPathNode.size(); k++){
                        temp13.push_back(fullPathNode[k]);
                    }
                    temp13 = UselessOpt(temp13, pickUps, dropOffs, interestedIntersections);
                    if (findDirectTimePath(temp13,interestedIntersections).first<current_time && testValidPath(temp13,pickUps,dropOffs)){
                        //std::cout<<findDirectTimePath(temp,interestedIntersections).first<< "yeah"<<current_time<<std::endl;
                        fullPathNode = temp13;
                        current_time = findDirectTimePath(temp13,interestedIntersections).first;
                    }
                    else if (!testValidPath(temp13,pickUps,dropOffs)){
                        //std::cout<<"invalid"<<std::endl;
                    }
                    else{
                        //std::cout<<"not godd"<<std::endl;
                    }



                }
            }
        }
    }

    return fullPathNode;
}

std::vector<IntersectionIdx> UselessOpt(std::vector<IntersectionIdx>& fullPathNode, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections){
    // std::vector<int> needDel;
    // for(int i = fullPathNode.size()-1; i>=0; i--){
    //     if(findTypeSpecial(fullPathNode[i],pickUps,dropOffs)== 1){ //dropoff, delete al previous
    //         for(int j = i-1; j>0; j--){
    //             if(fullPathNode[i] == fullPathNode[j]){
    //                 needDel.push_back(j);
    //             }
    //         }
    //     }
    // }
    // std::sort(needDel.begin(),needDel.end(),std::greater<int>());
    // for (int index : needDel){
    //     if(index >= 0 && index < needDel.size()){
    //         fullPathNode.erase(fullPathNode.begin()+index);
    //     } 
    // }

    return fullPathNode;
}


std::vector<IntersectionIdx> twoOptPerturbation2(std::vector<IntersectionIdx>& fullPathNode, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections) {

    //my code

    //select 3 pieces to cut, suppose fullnodesppath.len = n+1, cut has n choices
    #pragma omp parallel for
    for(int x = 1; x<fullPathNode.size()-1; x+=2){
        for(int y = x; y<fullPathNode.size()-1; y+=2){
            double current_time = findDirectTimePath(fullPathNode,interestedIntersections).first;
            if(x!=y){
                std::vector<IntersectionIdx> temp;

                for(int k = 0 ; k<=x;k++){
                    temp.push_back(fullPathNode[k]);
                }
                for(int k =y ; k>=x+1;k--){
                    temp.push_back(fullPathNode[k]);
                }
                for(int k = y + 1; k<fullPathNode.size(); k++){
                    temp.push_back(fullPathNode[k]);
                }
                temp = UselessOpt(temp, pickUps, dropOffs, interestedIntersections);
                
                if (findDirectTimePath(temp,interestedIntersections).first<current_time && testValidPath(temp,pickUps,dropOffs)){
                    //std::cout<<findDirectTimePath(temp,interestedIntersections).first<< "yeah"<<current_time<<std::endl;
                    fullPathNode = temp;
                    current_time = findDirectTimePath(temp,interestedIntersections).first;
                }
                else if (!testValidPath(temp,pickUps,dropOffs)){
                    //std::cout<<"invalid"<<std::endl;
                }
                else{
                    //std::cout<<"not godd"<<std::endl;
                }
                
            }
        }
    }
    return fullPathNode;
}

std::vector<IntersectionIdx> twoOptPerturbation(std::vector<IntersectionIdx>& fullPathNode, std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections) {

    //my code



    //select 3 pieces to cut, suppose fullnodesppath.len = n+1, cut has n choices
    #pragma omp parallel for
    for(int x = 0; x<fullPathNode.size()-1; x+=2){
        for(int y = x; y<fullPathNode.size()-1; y+=2){
            double current_time = findDirectTimePath(fullPathNode,interestedIntersections).first;
            if(x!=y){
                std::vector<IntersectionIdx> temp;

                for(int k = 0 ; k<=x;k++){
                    temp.push_back(fullPathNode[k]);
                }
                for(int k =y ; k>=x+1;k--){
                    temp.push_back(fullPathNode[k]);
                }
                for(int k = y + 1; k<fullPathNode.size(); k++){
                    temp.push_back(fullPathNode[k]);
                }
                temp = UselessOpt(temp, pickUps, dropOffs, interestedIntersections);
                
                if (findDirectTimePath(temp,interestedIntersections).first<current_time && testValidPath(temp,pickUps,dropOffs)){
                    //std::cout<<findDirectTimePath(temp,interestedIntersections).first<< "yeah"<<current_time<<std::endl;
                    fullPathNode = temp;
                    current_time = findDirectTimePath(temp,interestedIntersections).first;
                }
                else if (!testValidPath(temp,pickUps,dropOffs)){
                    //std::cout<<"invalid"<<std::endl;
                }
                else{
                    //std::cout<<"not godd"<<std::endl;
                }
                
            }
        }
    }
    return fullPathNode;
}



std::vector<std::vector<IntersectionIdx>> permuGivenIndex(std::vector<IntersectionIdx>& fullPathNodes, int start, int end){
    //both start and end are inclusive
    std::vector<std::vector<IntersectionIdx>> re;

    std::vector<IntersectionIdx> before;
    std::vector<IntersectionIdx> after;
    std::vector<IntersectionIdx> middle;
    for(int i = 0; i<start; ++i){
        before.push_back(fullPathNodes[i]);
    }
    for(int i = start; i<=end; ++i){
        middle.push_back(fullPathNodes[i]);
    }
    for(int i = end+1; i<fullPathNodes.size(); ++i){
        after.push_back(fullPathNodes[i]);
    }
    std::vector<std::vector<IntersectionIdx>> possMiddle = findAllPermutation(middle);
    for(int i = 0; i<possMiddle.size(); ++i){
        std::vector<IntersectionIdx> curr;
        for(int j = 0; j<before.size(); ++j){
            curr.push_back(before[j]);
        }
        for(int j = 0; j<possMiddle[i].size(); ++j){
            curr.push_back(possMiddle[i][j]);
        }
        for(int j = 0; j<after.size(); ++j){
            curr.push_back(after[j]);
        }
        re.push_back(curr);
    }
    return re;
}

std::vector<IntersectionIdx> localImprove(std::vector<IntersectionIdx>& fullPathNodes, int level,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections){

    int localOptimizeLength = level ; //here is 4 level optimize among 5 points where the middle 3 can be pernumtated
    //std::cout <<"reached4!!!!!!"<<std::endl;
    #pragma omp parallel for
    for(int localOptimizIndex = 0; localOptimizIndex < fullPathNodes.size()-1-localOptimizeLength;localOptimizIndex = localOptimizIndex + localOptimizeLength){//we don't optimize the first and last, since connected to depot
 
        //every time, consider localOptimizIndex +1, +2, ... +L
        //std::cout <<"reached2!!!!!!"<<std::endl;
        double best_time = findDirectTimePath(fullPathNodes, interestedIntersections).first;

        std::vector<std::vector<IntersectionIdx>> newPaths = permuGivenIndex(fullPathNodes,localOptimizIndex+1,localOptimizIndex+localOptimizeLength ); //the orginal is included
        for (int i = 0; i < newPaths.size(); ++i){
        /*
        std::vector<IntersectionIdx> fullkk = fulls[i];
        for(int s = 0; s < fullkk.size(); ++s){
            std::cout<<fullkk[s] << " ";
        }
        std::cout << 1 << std::endl;
        */
        if(testValidPath(newPaths[i], pickUps, dropOffs)){
            /*std::cout << "in"<<std::endl;*/
            double thisTime = findDirectTimePath(newPaths[i], interestedIntersections).first;
            if(best_time > thisTime){   
                fullPathNodes = newPaths[i];
                best_time = thisTime;
            }
        }
        }
    }

    localOptimizeLength = level - 1 ; //here is 4 level optimize among 5 points where the middle 3 can be pernumtated
    //std::cout <<"reached4!!!!!!"<<std::endl;
    #pragma omp parallel for
    for(int localOptimizIndex = 0; localOptimizIndex < fullPathNodes.size()-1-localOptimizeLength;localOptimizIndex = localOptimizIndex + localOptimizeLength){//we don't optimize the first and last, since connected to depot
 
        //every time, consider localOptimizIndex +1, +2, ... +L
        //std::cout <<"reached2!!!!!!"<<std::endl;
        double best_time = findDirectTimePath(fullPathNodes, interestedIntersections).first;

        std::vector<std::vector<IntersectionIdx>> newPaths = permuGivenIndex(fullPathNodes,localOptimizIndex+1,localOptimizIndex+localOptimizeLength ); //the orginal is included
        for (int i = 0; i < newPaths.size(); ++i){
        /*
        std::vector<IntersectionIdx> fullkk = fulls[i];
        for(int s = 0; s < fullkk.size(); ++s){
            std::cout<<fullkk[s] << " ";
        }
        std::cout << 1 << std::endl;
        */
        if(testValidPath(newPaths[i], pickUps, dropOffs)){
            /*std::cout << "in"<<std::endl;*/
            double thisTime = findDirectTimePath(newPaths[i], interestedIntersections).first;
            if(best_time > thisTime){   
                fullPathNodes = newPaths[i];
                best_time = thisTime;
            }
        }
        }
    }
    localOptimizeLength = level - 1 ; //here is 4 level optimize among 5 points where the middle 3 can be pernumtated
    //std::cout <<"reached4!!!!!!"<<std::endl;
    #pragma omp parallel for
    for(int localOptimizIndex = 0; localOptimizIndex < fullPathNodes.size()-1-localOptimizeLength;localOptimizIndex = localOptimizIndex + localOptimizeLength){//we don't optimize the first and last, since connected to depot
 
        //every time, consider localOptimizIndex +1, +2, ... +L
        //std::cout <<"reached2!!!!!!"<<std::endl;
        double best_time = findDirectTimePath(fullPathNodes, interestedIntersections).first;

        std::vector<std::vector<IntersectionIdx>> newPaths = permuGivenIndex(fullPathNodes,localOptimizIndex+1,localOptimizIndex+localOptimizeLength ); //the orginal is included
        for (int i = 0; i < newPaths.size(); ++i){
        /*
        std::vector<IntersectionIdx> fullkk = fulls[i];
        for(int s = 0; s < fullkk.size(); ++s){
            std::cout<<fullkk[s] << " ";
        }
        std::cout << 1 << std::endl;
        */
        if(testValidPath(newPaths[i], pickUps, dropOffs)){
            /*std::cout << "in"<<std::endl;*/
            double thisTime = findDirectTimePath(newPaths[i], interestedIntersections).first;
            if(best_time > thisTime){   
                fullPathNodes = newPaths[i];
                best_time = thisTime;
            }
        }
        }
    }
    return fullPathNodes;
}


std::vector<IntersectionIdx> localImprove2(std::vector<IntersectionIdx>& fullPathNodes, int level,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections){

    int localOptimizeLength = level ; //here is 4 level optimize among 5 points where the middle 3 can be pernumtated
    //std::cout <<"reached4!!!!!!"<<std::endl;
    for(int localOptimizIndex = 0; localOptimizIndex+localOptimizeLength < fullPathNodes.size()-1;localOptimizIndex++){//we don't optimize the first and last, since connected to depot
 
        //every time, consider localOptimizIndex +1, +2, ... +L
        //std::cout <<"reached2!!!!!!"<<std::endl;
        double best_time = findDirectTimePath(fullPathNodes, interestedIntersections).first;

        std::vector<std::vector<IntersectionIdx>> newPaths = permuGivenIndex(fullPathNodes,localOptimizIndex+1,localOptimizIndex+localOptimizeLength ); //the orginal is included
        for (int i = 0; i < newPaths.size(); ++i){

        if(testValidPath(newPaths[i], pickUps, dropOffs)){
            /*std::cout << "in"<<std::endl;*/
            double thisTime = findDirectTimePath(newPaths[i], interestedIntersections).first;
            if(best_time > thisTime){   
                fullPathNodes = newPaths[i];
                best_time = thisTime;
            }
        }
        }
    }
    return fullPathNodes;
}

std::vector<std::vector<IntersectionIdx>> shiftGivenIndex(std::vector<IntersectionIdx>&fullPathNodes, int start, int end ){
    //provides all possibilities that shifts the [start:end+1] to a new position


    std::vector<std::vector<IntersectionIdx>> re;
    std::vector<IntersectionIdx> middle;

    //first push middle
    for(int i = start; i<end+1; ++i){
        middle.push_back(fullPathNodes[i]);
    }

    //this is not optimized

    //if shift to the left
    for(int shiftindex = -(fullPathNodes.size()+start-end); shiftindex<0; ++shiftindex){
        std::vector<IntersectionIdx> temp;
        for(int i = 0; i<start+shiftindex; ++i){
            temp.push_back(fullPathNodes[i]);
        }
        for(int i = 0; i<middle.size(); ++i){
            temp.push_back(middle[i]);
        }
        for(int i = start+shiftindex; i<fullPathNodes.size(); ++i){
            if(i>=start && i<=end){
                continue;
            }
            temp.push_back(fullPathNodes[i]);
        }
        re.push_back(temp);
    }
    //if shift to the left
    for(int shiftindex = 0; shiftindex<fullPathNodes.size()+start-end; ++shiftindex){
        std::vector<IntersectionIdx> temp2;
        for(int i = 0; i<start; ++i){         //first part
            temp2.push_back(fullPathNodes[i]);
        }
        for(int i = end+1; i<end+shiftindex+1; ++i){ //the target part
            temp2.push_back(fullPathNodes[i]);
        }
        for(int i = 0; i<middle.size(); ++i){ //the target part
            temp2.push_back(middle[i]);
        }
        for(int i = end+shiftindex+1; i<fullPathNodes.size(); ++i){ //the rest
            temp2.push_back(fullPathNodes[i]);
        }
        re.push_back(temp2);
    }

    return re;
}

std::vector<IntersectionIdx> shiftImprove(std::vector<IntersectionIdx>& fullPathNodes, int level,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections){


    int localOptimizIndex = 0;
    int localOptimizeLength = level ; //the level should be paramaterized
    
    for(int localOptimizIndex = 0; localOptimizIndex+localOptimizeLength < fullPathNodes.size()-1;localOptimizIndex++) { //we don't optimize the first and last, since connected to depot
        //every time, consider localOptimizIndex +1, +2, ... +L
        //std::cout <<"reached2!!!!!!"<<std::endl;
        std::vector<std::vector<IntersectionIdx>> newPaths = shiftGivenIndex(fullPathNodes,localOptimizIndex+1,localOptimizIndex+localOptimizeLength ); //the orginal is included
        fullPathNodes = findBestOne(newPaths,pickUps,dropOffs,interestedIntersections);

    }
    /* if time, consider re-doing with lower level
    int localOptimizIndex = 0;
    int localOptimizeLength = level - 1 ; //the level should be paramaterized
    while(localOptimizIndex+localOptimizeLength < fullPathNodes.size()-1){ //we don't optimize the first and last, since connected to depot
        //every time, consider localOptimizIndex +1, +2, ... +L
        //std::cout <<"reached2!!!!!!"<<std::endl;
        std::vector<std::vector<IntersectionIdx>> newPaths = shiftGivenIndex(fullPathNodes,localOptimizIndex+1,localOptimizIndex+localOptimizeLength ); //the orginal is included
        fullPathNodes = findBestOne(newPaths,pickUps,dropOffs,interestedIntersections);
        localOptimizIndex ++;
    }
    */

    return fullPathNodes;
}


std::vector<IntersectionIdx> shiftImproveff(std::vector<IntersectionIdx>& fullPathNodes, int level,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections){


    int localOptimizIndex = 0;
    int localOptimizeLength = level ; //the level should be paramaterized
    
    for(int localOptimizIndex = 0; localOptimizIndex+localOptimizeLength < fullPathNodes.size()-1;localOptimizIndex++) { //we don't optimize the first and last, since connected to depot
        //every time, consider localOptimizIndex +1, +2, ... +L
        //std::cout <<"reached2!!!!!!"<<std::endl;
        std::vector<std::vector<IntersectionIdx>> newPaths = shiftGivenIndex(fullPathNodes,localOptimizIndex+1,localOptimizIndex+localOptimizeLength ); //the orginal is included
        fullPathNodes = findBestSecond(newPaths,pickUps,dropOffs,interestedIntersections);

    }
    /* if time, consider re-doing with lower level
    int localOptimizIndex = 0;
    int localOptimizeLength = level - 1 ; //the level should be paramaterized
    while(localOptimizIndex+localOptimizeLength < fullPathNodes.size()-1){ //we don't optimize the first and last, since connected to depot
        //every time, consider localOptimizIndex +1, +2, ... +L
        //std::cout <<"reached2!!!!!!"<<std::endl;
        std::vector<std::vector<IntersectionIdx>> newPaths = shiftGivenIndex(fullPathNodes,localOptimizIndex+1,localOptimizIndex+localOptimizeLength ); //the orginal is included
        fullPathNodes = findBestOne(newPaths,pickUps,dropOffs,interestedIntersections);
        localOptimizIndex ++;
    }
    */

    return fullPathNodes;
}


std::vector<IntersectionIdx> shiftImprove2(std::vector<IntersectionIdx>& fullPathNodes, int level,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections){


    int localOptimizIndex = 0;
    int localOptimizeLength = level ; //the level should be paramaterized
    for(int localOptimizIndex = 0; localOptimizIndex+localOptimizeLength < fullPathNodes.size()-1;localOptimizIndex += localOptimizeLength){ //we don't optimize the first and last, since connected to depot
        //every time, consider localOptimizIndex +1, +2, ... +L
        //std::cout <<"reached2!!!!!!"<<std::endl;
        std::vector<std::vector<IntersectionIdx>> newPaths = shiftGivenIndex(fullPathNodes,localOptimizIndex+1,localOptimizIndex+localOptimizeLength ); //the orginal is included
        fullPathNodes = findBestOne(newPaths,pickUps,dropOffs,interestedIntersections);
    }
    /* if time, consider re-doing with lower level
    int localOptimizIndex = 0;
    int localOptimizeLength = level - 1 ; //the level should be paramaterized
    while(localOptimizIndex+localOptimizeLength < fullPathNodes.size()-1){ //we don't optimize the first and last, since connected to depot
        //every time, consider localOptimizIndex +1, +2, ... +L
        //std::cout <<"reached2!!!!!!"<<std::endl;
        std::vector<std::vector<IntersectionIdx>> newPaths = shiftGivenIndex(fullPathNodes,localOptimizIndex+1,localOptimizIndex+localOptimizeLength ); //the orginal is included
        fullPathNodes = findBestOne(newPaths,pickUps,dropOffs,interestedIntersections);
        localOptimizIndex = localOptimizIndex + localOptimizeLength;
    }
    */

    return fullPathNodes;
}


std::vector<IntersectionIdx> shiftImprove2ff(std::vector<IntersectionIdx>& fullPathNodes, int level,std::vector<IntersectionIdx>& pickUps, std::vector<IntersectionIdx>& dropOffs, std::vector<IntersectionIdx>& interestedIntersections){


    int localOptimizIndex = 0;
    int localOptimizeLength = level ; //the level should be paramaterized
    for(int localOptimizIndex = 0; localOptimizIndex+localOptimizeLength < fullPathNodes.size()-1;localOptimizIndex += localOptimizeLength){ //we don't optimize the first and last, since connected to depot
        //every time, consider localOptimizIndex +1, +2, ... +L
        //std::cout <<"reached2!!!!!!"<<std::endl;
        std::vector<std::vector<IntersectionIdx>> newPaths = shiftGivenIndex(fullPathNodes,localOptimizIndex+1,localOptimizIndex+localOptimizeLength ); //the orginal is included
        fullPathNodes = findBestSecond(newPaths,pickUps,dropOffs,interestedIntersections);
    }
    /* if time, consider re-doing with lower level
    int localOptimizIndex = 0;
    int localOptimizeLength = level - 1 ; //the level should be paramaterized
    while(localOptimizIndex+localOptimizeLength < fullPathNodes.size()-1){ //we don't optimize the first and last, since connected to depot
        //every time, consider localOptimizIndex +1, +2, ... +L
        //std::cout <<"reached2!!!!!!"<<std::endl;
        std::vector<std::vector<IntersectionIdx>> newPaths = shiftGivenIndex(fullPathNodes,localOptimizIndex+1,localOptimizIndex+localOptimizeLength ); //the orginal is included
        fullPathNodes = findBestOne(newPaths,pickUps,dropOffs,interestedIntersections);
        localOptimizIndex = localOptimizIndex + localOptimizeLength;
    }
    */

    return fullPathNodes;
}