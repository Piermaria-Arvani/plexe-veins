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

	//compute intervehicle distance
	double distance = platoonInsertSpeed / 3.6 * platoonInsertHeadway + platoonInsertDistance;
	//total number of platoons per lane
	int nPlatoons = nCars / platoonSize / nLanes;
	//length of 1 platoon
	double platoonLength = platoonSize * 4 + (platoonSize - 1) * distance;
	//inter-platoon distance
	double platoonDistance = platoonInsertSpeed / 3.6 * platoonLeaderHeadway;
	//total length for one lane
	double totalLength = nPlatoons * platoonLength + (nPlatoons - 1) * platoonDistance;

	//for each lane, we create an offset to have misaligned platoons
	double *laneOffset = new double[nLanes];
	for (int l = 0; l < nLanes; l++)
		laneOffset[l] = uniform(0, 20);

	double currentPos = totalLength;
	int currentCar = 0;
	for (int i = 0; i < nCars/nLanes; i++) {
		for (int l = 0; l < nLanes; l++) {
			automated.position = currentPos + laneOffset[l];
			automated.lane = l;
			addVehicleToQueue(0, automated);
			if ( currentCar == 0){
				//inserts a new leader
				platoons.push_back(platoon());
				platoons.back().push_back(i);
			}else{
				//inserts a vehicle into the leader vector
				insertFollower(i,(i%nLanes));
			}
		}
		currentCar++;
		if (currentCar == platoonSize) {
			currentCar = 0;
			//add inter platoon gap
			currentPos -= (platoonDistance + 4);
		}
		else {
			//add intra platoon gap
			currentPos -= (4 + distance);
		}
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
