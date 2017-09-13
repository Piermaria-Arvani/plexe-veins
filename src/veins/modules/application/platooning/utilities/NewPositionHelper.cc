#include "veins/modules/application/platooning/utilities/NewPositionHelper.h"

Define_Module(NewPositionHelper);

void NewPositionHelper::initialize(int stage) {

	BasePositionHelper::initialize(stage);
	if (stage == 0) {
		nCars = par("nCars").longValue();
		myId = getIdFromExternalId(getExternalId());
		mySumoId = getExternalId();
	}
}

void NewPositionHelper::setPlatoon(platoon p){
	this->p.clear();
	for(unsigned i = 0; i < p.size(); i++){
		this->p.push_back(p[i]);
	}
}

void NewPositionHelper::insertFollower(int id){
	p.push_back(id);
}

void NewPositionHelper::removeFollower(int id){
	for(unsigned i = 0; i < p.size(); i++){
		if (p[i] == id){
			p.erase (p.begin() + i);
		}
	}
}

platoon& NewPositionHelper::getPlatoon(){
	return p;
}

int NewPositionHelper::getLastVehicle (){
	return p.back();
}

int NewPositionHelper::getPlatoonSize (){
	return p.size();
}

int NewPositionHelper::getFrontVehicle(){
	int frontVehicle = -1;
	for(unsigned i = 0; i < p.size(); i++){
		if (p[i] == myId){
			frontVehicle = p[i-1];
		}
	}
	return frontVehicle;
}
bool NewPositionHelper::isLeader(int id){
	if (p.size() == 0){
		return false;
	}
	return id == p[0];
}

void NewPositionHelper::setLeader(int id, std::string sumoId){
	leaderId = id;
	myLeaderSumoId = sumoId;
}

void NewPositionHelper::setFrontVehicle(int id, std::string sumoId){
	frontId = id;
	myFrontVehicleSumoId = sumoId;
}

std::string NewPositionHelper::getLeaderSumoId(){
	return myLeaderSumoId;
}

std::string NewPositionHelper::getFrontVehicleSumoId(){
	return myFrontVehicleSumoId;
}


int NewPositionHelper::getIdFromExternalId(std::string externalId) {
	int dotIndex = externalId.find_last_of('.');
	std::string strId = externalId.substr(dotIndex + 1);
	return strtol(strId.c_str(), 0, 10);
}

bool NewPositionHelper::isInSamePlatoon(int vehicleId) {
	return true;
}

void NewPositionHelper::finish() {
	BasePositionHelper::finish();
}
