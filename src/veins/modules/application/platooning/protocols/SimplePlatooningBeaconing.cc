//
// Copyright (c) 2012-2016 Michele Segata <segata@ccs-labs.org>
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

#include "SimplePlatooningBeaconing.h"

Define_Module(SimplePlatooningBeaconing)

void SimplePlatooningBeaconing::initialize(int stage) {
	BaseProtocol::initialize(stage);

	if (stage == 0) {
		//random start time
		SimTime beginTime = SimTime(uniform(0.001, beaconingInterval));
		scheduleAt(simTime() + beaconingInterval + beginTime, sendBeacon);
		positionHelper = FindModule<BasePositionHelper*>::findSubModule(getParentModule());
		newPositionHelper = FindModule<NewPositionHelper*>::findSubModule(getParentModule());
	}
}

void SimplePlatooningBeaconing::handleSelfMsg(cMessage *msg) {

	BaseProtocol::handleSelfMsg(msg);

	if (msg == sendBeacon) {
		//sendPlatooningMessage(-1);

		if(positionHelper->getLeaderId() != -1 && !positionHelper->isLeader()){
			//getting leader data directly from sumo
			std::string leaderSumoId = newPositionHelper->getLeaderSumoId();
			double speed, acceleration, controllerAcceleration, sumoPosX, sumoPosY, sumoTime;
			traci->vehicle(leaderSumoId).getVehicleData(speed, acceleration, controllerAcceleration, sumoPosX, sumoPosY, sumoTime);
			traciVehicle->setPlatoonLeaderData(speed, acceleration, sumoPosX, sumoPosY, sumoTime);
			traciVehicle->setControllerFakeData(0, -1, 0, speed, acceleration);


			if (traci->vehicle(leaderSumoId).getLaneIndex() != traciVehicle->getLaneIndex()){
				traciVehicle->setFixedLane(traci->vehicle(leaderSumoId).getLaneIndex());
				positionHelper->setPlatoonLane(traci->vehicle(leaderSumoId).getLaneIndex());
			}
			//getting front vehicle data directly from sumo
			std::string frontVehicleSumoId = newPositionHelper->getFrontVehicleSumoId();
			traci->vehicle(frontVehicleSumoId).getVehicleData(speed, acceleration, controllerAcceleration, sumoPosX, sumoPosY, sumoTime);
			traciVehicle->setPrecedingVehicleData(speed, acceleration, sumoPosX, sumoPosY, sumoTime);
			//get front vehicle position
			Coord frontPosition(sumoPosX, sumoPosY, 0);
			//get my position
			Veins::TraCICoord traciPosition = mobility->getManager()->omnet2traci(mobility->getCurrentPosition());
			Coord position(traciPosition.x, traciPosition.y);
			//compute distance (-4 because of vehicle length)
			double distance = position.distance(frontPosition) - 4;
			traciVehicle->setControllerFakeData(distance, speed, acceleration, -1, 0);

			struct Plexe::VEHICLE_DATA frontVehicleData;
			frontVehicleData.index = newPositionHelper->getPlatoon().size()-1;
			frontVehicleData.acceleration = acceleration;
			//for now length is fixed to 4 meters. TODO: take it from sumo
			frontVehicleData.length = 4;
			frontVehicleData.positionX = sumoPosX;
			frontVehicleData.positionY = sumoPosY;
			frontVehicleData.speed =speed;
			frontVehicleData.time = sumoTime;
			//send information to CACC
			traciVehicle->setGenericInformation(CC_SET_VEHICLE_DATA, &frontVehicleData, sizeof(struct Plexe::VEHICLE_DATA));


		}
		scheduleAt(simTime() + beaconingInterval, sendBeacon);
	}


}

void SimplePlatooningBeaconing::messageReceived(PlatooningBeacon *pkt, UnicastMessage *unicast) {
	//nothing to do for static beaconing
}

SimplePlatooningBeaconing::SimplePlatooningBeaconing()
{}

SimplePlatooningBeaconing::~SimplePlatooningBeaconing()
{}

void SimplePlatooningBeaconing::finish(){
	BaseProtocol::finish();
}
