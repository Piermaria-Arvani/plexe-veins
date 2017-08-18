#include "CreatePlatoonTrafficManager.h"

Define_Module(CreatePlatoonTrafficManager);

void CreatePlatoonTrafficManager::initialize(int stage) {

	PlatoonsTrafficManager::initialize(stage);

	if (stage == 0) {
		insertJoinerMessage = new cMessage("");
		scheduleAt(platoonInsertTime + SimTime(5), insertJoinerMessage);
		message = new cMessage("");
		scheduleAt(platoonInsertTime + SimTime(10), message);
	}

}

void CreatePlatoonTrafficManager::handleSelfMsg(cMessage *msg) {

	PlatoonsTrafficManager::handleSelfMsg(msg);

	if (msg == insertJoinerMessage) {
		for(int i = 0; i < 2; i++)
			insertJoiner();
	}
	if (msg == message) {
			for(int i = 0; i < 2; i++)
				insertJoiner();
		}

}

void CreatePlatoonTrafficManager::insertJoiner() {
	automated.position = 0;
	automated.lane = 2;
	addVehicleToQueue(0, automated);
}

void CreatePlatoonTrafficManager::finish() {
	PlatoonsTrafficManager::finish();
	cancelAndDelete(insertJoinerMessage);
	insertJoinerMessage = 0;
}
