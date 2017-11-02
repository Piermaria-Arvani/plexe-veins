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

		insertVehicleFromBegin = new cMessage("insert from begin");
		insertVehicleInRamps = new cMessage("insert in ramps");
		insertVehicle = new cMessage("insert");

		recordNcars = new cMessage ("record cars number");
		scheduleAt(simTime() + SimTime(60), recordNcars);

		carNumber.setName("carsNumber");
		exitedCarsNumber.setName("exitedCarsNumber");

		distributionInsertionTime = &par("distributionInsertionTime");
	}

}

void PlatoonsTrafficManager::recordCarsNumber(){
	carNumber.record(carsCounter);
	exitedCarsNumber.record(exitedCars);
	std::cout<<"cars number = "<<carsCounter<<"exitedCars"<<exitedCars<<endl;
	scheduleAt(simTime() + SimTime(60), recordNcars);
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
	}else if (msg == insertVehicleInRamps){
		insertVehiclesInRamps();
	}else if(msg == insertVehicleFromBegin){
		insertVehiclesFromBegin();
	}else if(msg == recordNcars){
		recordCarsNumber();
	}else if(msg == insertVehicle){
		insertVehicles();
	}

}

void PlatoonsTrafficManager::insertPlatoons() {


	std::list<std::string> routes = commandInterface->getRouteIds();
	EV << "Having currently " << routes.size() << " routes in the scenario" << std::endl;
	for (std::list<std::string>::const_iterator i = routes.begin(); i != routes.end(); ++i) {
		std::string routeId = *i;
		EV << routeId << std::endl;
		routeIds.push_back(routeId);
		std::list<std::string> routeEdges = commandInterface->route(routeId).getRoadIds();
		std::string firstEdge = *(routeEdges.begin());
		EV << "First Edge of route " << routeId << " is " << firstEdge << std::endl;
		routeStartLaneIds[routeId] = laneIdsOnEdge[firstEdge];
	}

	routeNumber = routeIds.size() -1 ;
	double time = 1/distributionInsertionTime->doubleValue();
	scheduleAt(simTime() + SimTime(time), insertVehicle);


	/*for(int i = 0; i < nCars; i++){
		std::string route = routeIds[routeCounter];
		std::string str1 = route.substr (0,5);
		if (str1.compare("begin") != 0 && route.compare("platoon_route") != 0){
			std::string temp = route.substr (8,1); // TODO: take in consideration integers higher than 9
			unsigned int index = atoi(temp.c_str());
			std::pair<int, int> pair_temp = std::make_pair(i,routeCounter);
			if(vehiclesToBeInsertedInRamps.size() <= index){
				vehiclesToBeInsertedInRamps.push_back(vehiclesPerRamp ());
				vehiclesToBeInsertedInRamps.back().push_back(pair_temp);
			}else{
				vehiclesToBeInsertedInRamps[index].push_back(pair_temp);
			}
		}else{
			std::pair<int, int> pair_temp = std::make_pair(i,routeCounter);
			vehiclesToBeInsertedFromBegin.push_back(pair_temp);
		}

		routeCounter = (int)cComponent::uniform(0, routeNumber);
	}

	if(vehiclesToBeInsertedFromBegin.size()>0){
		double time = 1/distributionInsertionTime->doubleValue();
		if(time < 0.5)
			time = 0.5;
		scheduleAt(simTime() + SimTime(time), insertVehicleFromBegin);
	}

	if(vehiclesToBeInsertedInRamps.size()>0){
		double time = 1/distributionInsertionTime->doubleValue();
		if(time < 0.5)
			time = 0.5;
		scheduleAt(simTime() + SimTime(time), insertVehicleInRamps);
	}*/


}

void PlatoonsTrafficManager::insertVehicles(){
	int routeCounter = (int) round(uniform(0, routeNumber));
	std::string route = routeIds[routeCounter];
	std::string str1 = route.substr (0,5);
	if (str1.compare("begin") != 0 && route.compare("platoon_route") != 0){
		automated.position = 0;
		automated.speed = 20;
		automated.lane = 0;
		addVehicleToQueue(routeCounter, automated);
		platoons.push_back(platoon());
		platoons.back().push_back(carsCounter);
	}else{
		automated.position = 4;
		automated.lane = (int)round(uniform(0,2)); //TODO: take the max number of lanes from sumo
		addVehicleToQueue(routeCounter, automated);
		platoons.push_back(platoon());
		platoons.back().push_back(carsCounter);
	}
	carsCounter++;

	if(carsCounter < nCars){
		double time = 1/distributionInsertionTime->doubleValue();
		scheduleAt(simTime() + SimTime(time), insertVehicle);
	}
}

void PlatoonsTrafficManager::insertVehiclesFromBegin(){
	if(vehiclesToBeInsertedFromBegin.size() > counterVehiclesFromBegin){
		automated.position = 4;
		automated.lane = (int)round(uniform(0,2)); //TODO: take the max number of lanes from sumo
		addVehicleToQueue(vehiclesToBeInsertedFromBegin[counterVehiclesFromBegin].second, automated);
		platoons.push_back(platoon());
		platoons.back().push_back(carsCounter);

		carsCounter++;
		counterVehiclesFromBegin++;

		double time = 1/distributionInsertionTime->doubleValue();
		if(time < 0.5)
			time = 0.5;
		scheduleAt(simTime() + SimTime(time), insertVehicleFromBegin);
	}
}

void PlatoonsTrafficManager::insertVehiclesInRamps(){
	bool noVehicles = true;
	for(unsigned int i = 0; i < vehiclesToBeInsertedInRamps.size(); i++){
		if(!vehiclesToBeInsertedInRamps[i].empty() && vehiclesToBeInsertedInRamps[i].size() > counter){
			automated.position = 0;
			automated.speed = 20;
			automated.lane = 0;
			addVehicleToQueue(vehiclesToBeInsertedInRamps[i][counter].second, automated);
			platoons.push_back(platoon());
			platoons.back().push_back(carsCounter);
			carsCounter++;

			noVehicles = false;
		}
	}
	if(!noVehicles){
		counter++;
		double time = 1/distributionInsertionTime->doubleValue();
		if(time < 0.5)
			time = 0.5;
		scheduleAt(simTime() + SimTime(time), insertVehicleInRamps);
	}
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

int PlatoonsTrafficManager::getRouteNumber(std::string routeId){
	int routeNumber = -1;
	for(unsigned int i = 0; i < routeIds.size(); i++){
		if(routeId ==  routeIds[i])
			routeNumber = i;
	}

	return routeNumber;
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

void PlatoonsTrafficManager::increaseExitedCars(){
	exitedCars++;
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
	if (insertVehicleInRamps) {
		cancelAndDelete(insertVehicleInRamps);
		insertVehicleInRamps = 0;
	}
	if (insertVehicleFromBegin) {
		cancelAndDelete(insertVehicleFromBegin);
		insertVehicleFromBegin = 0;
	}
	if (recordNcars) {
		cancelAndDelete(recordNcars);
		recordNcars = 0;
	}
}
