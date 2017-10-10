//
// Copyright (C) 2014-2016 Michele Segata <segata@ccs-labs.org>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "PlatoonsTrafficManager.h"

Define_Module(PlatoonsTrafficManager);

void PlatoonsTrafficManager::initialize(int stage) {

	TraCIBaseTrafficManager::initialize(stage);

	if (stage == 0) {

		nCars = par("nCars").longValue();
		platoonSize = par("platoonSize").longValue();
		nLanes = par("nLanes").longValue();
		platoonInsertTime = SimTime(par("platoonInsertTime").doubleValue());
		platoonInsertSpeed = par("platoonInsertSpeed").doubleValue();
		platoonInsertDistance = par("platoonInsertDistance").doubleValue();
		platoonInsertHeadway = par("platoonInsertHeadway").doubleValue();
		platoonLeaderHeadway = par("platoonLeaderHeadway").doubleValue();
		platooningVType = par("platooningVType").stdstringValue();
		insertPlatoonMessage = new cMessage("");
		scheduleAt(platoonInsertTime, insertPlatoonMessage);

	}

}

void PlatoonsTrafficManager::scenarioLoaded() {
	automated.id = findVehicleTypeIndex(platooningVType);
	automated.lane = -1;
	automated.position = 0;
	automated.speed = platoonInsertSpeed/3.6;
}

void PlatoonsTrafficManager::handleSelfMsg(cMessage *msg) {

	TraCIBaseTrafficManager::handleSelfMsg(msg);

	if (msg == insertPlatoonMessage) {
		insertPlatoons();
	}

}

void PlatoonsTrafficManager::insertPlatoons() {

	//vector of all the routes ids
	std::vector<std::string> routeIds;
	std::list<std::string> routes = commandInterface->getRouteIds();
	EV << "Having currently " << routes.size() << " routes in the scenario" << std::endl;
	for (std::list<std::string>::const_iterator i = routes.begin(); i != routes.end(); ++i) {
		std::string routeId = *i;
		EV << routeId << std::endl;
		routeIds.push_back(routeId);
		std::cout<<"platoonsTrafficManager: "<< routeId<<endl;
		std::list<std::string> routeEdges = commandInterface->route(routeId).getRoadIds();
		std::string firstEdge = *(routeEdges.begin());
		EV << "First Edge of route " << routeId << " is " << firstEdge << std::endl;
		routeStartLaneIds[routeId] = laneIdsOnEdge[firstEdge];
	}


	//compute intervehicle distance
	double distance = platoonInsertSpeed / 3.6 * platoonInsertHeadway + platoonInsertDistance;
	double platoonDistance = platoonInsertSpeed / 3.6 * platoonLeaderHeadway;
	//total length for one lane
	double totalLength = nCars + (nCars-1) * platoonDistance;
	//number of highway lanes
	//TODO: take it directly
	int maxL = 3;
	int routeNumber = routeIds.size();
	//for each lane, we create an offset to have misaligned platoons
	double *laneOffset = new double[nCars];
	for (int l = 0; l < nCars; l++)
		laneOffset[l] = uniform(0, 10);


	double currentPos = totalLength;
	int l = 0;
	int routeCounter = 0;

	for(int i = 0; i < nCars; i++){
		std::string route = routeIds[routeCounter];
		std::string str1 = route.substr (0,5);
		if (str1.compare("begin") != 0){
			//automated.speed = 20;
			std::cout<<"route:"<<route<<"this one would be go slower"<<endl;
		}
		automated.position = currentPos - laneOffset[i];
		std::cout<<"position: "<<automated.position<<",currentPos:"<<currentPos<<", laneoffset: "<<laneOffset[i]<<",";
		automated.lane = l;
		addVehicleToQueue(0 /*routeCounter*/, automated);
		platoons.push_back(platoon());
		platoons.back().push_back(i);
		currentPos -= (4 + distance);

		l < maxL? l++ : l = 0;
		routeCounter < routeNumber-1? routeCounter++ : routeCounter = 0;
		std::cout<<"l:"<<l<<endl;
	}


	printMatrix();
	delete [] laneOffset;

}

void PlatoonsTrafficManager::insertFollower(int followerId, int leaderId){
	int j = 0;
	unsigned int i = 0;
	for(; i < platoons.size(); i++){
		if ( leaderId == platoons[i][j]){
			platoons[i].push_back(followerId);
		}
	}
}

void PlatoonsTrafficManager::printMatrix(){
	std::cout << endl;
	for(std::vector<std::vector<int> >::iterator it = platoons.begin(); it != platoons.end(); ++it){
		for(std::vector<int>::iterator jt = it->begin(); jt != it->end(); ++jt){
			std::cout << *jt;
		}
		std::cout << endl;
	}
}

bool PlatoonsTrafficManager::isLeader(unsigned int myId){
    bool leader = (myId < platoons.size())? true : false;;

    return leader;
}

platoon& PlatoonsTrafficManager::getLeaderInfo(int myId){
	int index = -1;
	for (unsigned int i = 0; i < platoons.size(); i++){
		if (platoons[i][0]==myId){
			index = i;;
		}
	}
	return platoons[index];
}


void PlatoonsTrafficManager::getFollowerInfo(int myId, int& leaderId, int& frontVehicle, int& backVehicle){
	for(unsigned int i = 0; i < platoons.size(); i++){
		for(unsigned int j = 0; j < platoons[i].size(); j++){
			if (platoons[i][j] == myId){
				leaderId = platoons[i][0];
				frontVehicle = platoons[i][j-1];
				backVehicle = platoons [i][j+1];
				return;
			}
		}
	}
}

void PlatoonsTrafficManager::finish() {
	TraCIBaseTrafficManager::finish();
	if (insertPlatoonMessage) {
		cancelAndDelete(insertPlatoonMessage);
		insertPlatoonMessage = 0;
	}
}
