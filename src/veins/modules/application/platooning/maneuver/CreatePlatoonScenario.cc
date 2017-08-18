#include "veins/modules/application/platooning/maneuver/CreatePlatoonScenario.h"

Define_Module(CreatePlatoonScenario);

void CreatePlatoonScenario::initialize(int stage) {

	BaseScenario::initialize(stage);

	if (stage == 0) {

		leaderState = LS_INIT;
		joinerState = JS_INIT;
		followerState = FS_INIT;
	}

	if (stage == 1) {

		//lane where the platoon is driving
		int platoonLane = 0;

		prepareManeuverCars(platoonLane);

		protocol = FindModule<BaseProtocol*>::findSubModule(getParentModule());

		//connect maneuver application to protocol
		protocol->registerApplication(MANEUVER_TYPE, gate("lowerLayerIn"), gate("lowerLayerOut"), gate("lowerControlIn"));
		//we are also interested in receiving beacons: the joiner must compute
		//its distance to the front vehicle while approaching it
		protocol->registerApplication(BaseProtocol::BEACON_TYPE, gate("lowerLayerIn"), gate("lowerLayerOut"), gate("lowerControlIn"));

		newPositionHelper = FindModule<NewPositionHelper*>::findSubModule(getParentModule());



	}

}

void CreatePlatoonScenario::prepareManeuverCars(int platoonLane) {

	switch (positionHelper->getId()) {

		case 0: {
			//this is the car that will be the leader
			traciVehicle->setCruiseControlDesiredSpeed(100.0 / 3.6);
			traciVehicle->setActiveController(Plexe::ACC);
			traciVehicle->setFixedLane(platoonLane);
			role = LEADER;

			positionHelper->setLeaderId(positionHelper->getId());
			positionHelper->setIsLeader(true);
			positionHelper->setPlatoonLane(platoonLane);
			positionHelper->setPlatoonId(positionHelper->getId());

			vehicleData.joinerId = -1;
			vehicleData.speed = 100/3.6;
			vehicleData.formation.push_back(0);

			//schedule the start of the maneuver
			startManeuver = new cMessage();
			scheduleAt(simTime() + SimTime(10), startManeuver);

			break;
		}

		case 1:
		case 2:
		case 3:
		case 4: {
			//at the beginning all other cars are free agents
			traciVehicle->setCruiseControlDesiredSpeed(100/3.6);
			traciVehicle->setFixedLane(2);
			traciVehicle->setActiveController(Plexe::ACC);
			role = JOINER;

			positionHelper->setLeaderId(-1);
			positionHelper->setPlatoonId(-1);
			positionHelper->setFrontId(-1);
			positionHelper->setIsLeader(false);
			positionHelper->setPlatoonLane(-1);

			vehicleData.speed = 100/3.6;

			break;
		}
	}
}

void CreatePlatoonScenario::finish() {

	if (startManeuver) {
		cancelAndDelete(startManeuver);
		startManeuver = 0;
	}

	BaseScenario::finish();

}

ManeuverMessage *CreatePlatoonScenario::generateMessage() {
	ManeuverMessage *msg = new ManeuverMessage();
	msg->setVehicleId(positionHelper->getId());
	msg->setPlatoonId(positionHelper->getPlatoonId());
	msg->setPlatoonLane(positionHelper->getPlatoonLane());
	msg->setPlatoonSpeed(vehicleData.speed);
	return msg;
}

void CreatePlatoonScenario::handleSelfMsg(cMessage *msg) {

	//this takes car of feeding data into CACC and reschedule the self message
	BaseScenario::handleSelfMsg(msg);

	if (msg == startManeuver)
		handleLeaderMsg(msg);

}

void CreatePlatoonScenario::handleLowerMsg(cMessage *msg) {
	switch (role) {
		case LEADER:
			handleLeaderMsg(msg);
			break;
		case FOLLOWER:
			handleFollowerMsg(msg);
			break;
		case JOINER:
			handleJoinerMsg(msg);
			break;
		default:
			ASSERT(false);
			break;
	};

}

void CreatePlatoonScenario::sendUnicast(cPacket *msg, int destination) {
	UnicastMessage *unicast = new UnicastMessage("", MANEUVER_TYPE);
	unicast->setDestination(destination);
	unicast->setChannel(Channels::CCH);
	unicast->encapsulate(msg);
	sendDown(unicast);
}
void CreatePlatoonScenario::sendLeaderProposal(){
	ManeuverMessage *toSend;
	toSend = generateMessage();
	toSend->setMessageType(LM_PROPOSE_AS_LEADER);
	toSend->setPlatoonSpeed(vehicleData.speed);
	toSend->setPlatoonId(positionHelper->getId());
	toSend->setPlatoonLane(positionHelper->getPlatoonLane());
	sendUnicast(toSend, -1);
}


void CreatePlatoonScenario::handleLeaderMsg(cMessage *msg) {

	//this message can be a self message, or a unicast message
	//with an encapsulated beacon or maneuver message
	ManeuverMessage *maneuver = 0;
	cPacket *encapsulated = 0;
	//maneuver message to be sent, if needed
	ManeuverMessage *toSend;

	//first check if this is a unicast message, and in case if it is a beacon or a maneuver
	UnicastMessage *unicast = dynamic_cast<UnicastMessage *>(msg);
	if (unicast) {
		encapsulated = unicast->decapsulate();
		maneuver = dynamic_cast<ManeuverMessage *>(encapsulated);
	}

	//check current leader status
	switch(leaderState) {
		case LS_INIT: {
			//notify to other cars the will to be the leader
			sendLeaderProposal();
			leaderState = LS_LEADING;
			break;
		}
		case LS_LEADING: {
			sendLeaderProposal();
			//when getting a message, and being in the LEADING state, we need
			//to check if this is a join request. if not just ignore it
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
				if (maneuver->getMessageType() == JM_REQUEST_JOIN) {
					vehicleData.joiners.push(maneuver->getVehicleId());
				}
			}
			if (!vehicleData.joiners.empty()){
				toSend = generateMessage();
				toSend->setMessageType(LM_MOVE_IN_POSITION);
				//this will be the front vehicle for the car which will join
				int frontVehicle = newPositionHelper->getLastVehicle() ;
				toSend->setFrontVehicleId(frontVehicle);
				//save some data. who is joining?
				vehicleData.joinerId = vehicleData.joiners.front();
				//send a positive ack to the joiner
				sendUnicast(toSend, vehicleData.joinerId);

				leaderState = LS_WAIT_JOINER_IN_POSITION;
			}
			break;
		}
		case LS_WAIT_JOINER_IN_POSITION: {
			sendLeaderProposal();
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
				//the joiner is now in position and is ready to join
				if (maneuver->getMessageType() == JM_IN_POSITION) {

					//tell him to join the platoon
					toSend = generateMessage();
					toSend->setMessageType(LM_JOIN_PLATOON);
					sendUnicast(toSend, vehicleData.joinerId);

					leaderState = LS_WAIT_JOINER_TO_JOIN;
				}
				else if (maneuver->getMessageType() == JM_REQUEST_JOIN) {
					vehicleData.joiners.push(maneuver->getVehicleId());
				}
			}
			break;
		}
		case LS_WAIT_JOINER_TO_JOIN: {
			sendLeaderProposal();
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
				//the joiner has joined the platoon
				if (maneuver->getMessageType() == JM_IN_PLATOON) {
					//add the joiner to the list of vehicles in the platoon
					vehicleData.formation.push_back(vehicleData.joinerId);
					vehicleData.joiners.pop();
					toSend = generateMessage();
					toSend->setMessageType(LM_UPDATE_FORMATION);
					toSend->setPlatoonFormationArraySize(vehicleData.formation.size());
					for (unsigned int i = 0; i < vehicleData.formation.size(); i++) {
						toSend->setPlatoonFormation(i, vehicleData.formation[i]);
					}
					//send to all vehicles
					sendUnicast(toSend, -1);

					newPositionHelper->insertFollower(vehicleData.joinerId);
					leaderState = LS_LEADING;
				}
				else if (maneuver->getMessageType() == JM_REQUEST_JOIN) {
					vehicleData.joiners.push(maneuver->getVehicleId());
				}
			}
			break;
		}
	}

	if (encapsulated) {
		delete encapsulated;
	}
	if (unicast) {
		delete unicast;
	}

}

void CreatePlatoonScenario::handleJoinerMsg(cMessage *msg) {
	//this message can be a self message, or a unicast message
	//with an encapsulated beacon or maneuver message
	ManeuverMessage *maneuver = 0;
	PlatooningBeacon *beacon = 0;
	cPacket *encapsulated = 0;
	//maneuver message to be sent, if needed
	ManeuverMessage *toSend;

	//first check if this is a unicast message, and in case if it is a beacon or a maneuver
	UnicastMessage *unicast = dynamic_cast<UnicastMessage *>(msg);
	if (unicast) {
		encapsulated = unicast->decapsulate();
		maneuver = dynamic_cast<ManeuverMessage *>(encapsulated);
		beacon = dynamic_cast<PlatooningBeacon *>(encapsulated);
	}

	//check current joiner status
	switch(joinerState) {

		//init state, just move to the idle state
		case JS_INIT: {
			joinerState = JS_IDLE;
			break;
		}

		case JS_IDLE: {
			//analyze the leader's proposal, then ask for joining
			if (maneuver && maneuver->getMessageType() == LM_PROPOSE_AS_LEADER) {
				positionHelper->setPlatoonId(maneuver->getPlatoonId());
				positionHelper->setLeaderId(maneuver->getPlatoonId());
				positionHelper->setPlatoonLane(maneuver->getPlatoonLane());
				toSend = generateMessage();
				toSend->setMessageType(JM_REQUEST_JOIN);
				sendUnicast(toSend, positionHelper->getLeaderId());
				joinerState = JS_WAIT_REPLY;
			}
			break;
		}

		case JS_WAIT_REPLY: {
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId() && maneuver->getMessageType()!=LM_PROPOSE_AS_LEADER) {

				//if the leader told us to move in position, we can start approaching the platoon
				if (maneuver->getMessageType() == LM_MOVE_IN_POSITION) {
					//save some data about the platoon
					positionHelper->setFrontId(maneuver->getFrontVehicleId());
					vehicleData.joinLane = maneuver->getPlatoonLane();

					//check for correct lane. if not in correct lane, change it
					int currentLane = traciVehicle->getLaneIndex();
					if (currentLane != vehicleData.joinLane) {
						traciVehicle->setFixedLane(vehicleData.joinLane);
					}

					//activate faked CACC. this way we can approach the front car using data obtained through GPS
					traciVehicle->setCACCConstantSpacing(15);
					//we have no data so far, so for the moment just initialize with some fake data
					traciVehicle->setControllerFakeData(15, vehicleData.speed, 0, vehicleData.speed, 0);
					//set a CC speed higher than the platoon speed to approach it
					traciVehicle->setCruiseControlDesiredSpeed(vehicleData.speed + 30/3.6);
					traciVehicle->setActiveController(Plexe::FAKED_CACC);

					joinerState = JS_MOVE_IN_POSITION;
				}
			}
			break;
		}

		case JS_MOVE_IN_POSITION: {

			//if we get data, just feed the fake CACC
			if (beacon && beacon->getVehicleId() == positionHelper->getFrontId()) {
				//get front vehicle position
				Coord frontPosition(beacon->getPositionX(), beacon->getPositionY(), 0);
				//get my position
				Veins::TraCICoord traciPosition = mobility->getManager()->omnet2traci(mobility->getCurrentPosition());
				Coord position(traciPosition.x, traciPosition.y);
				//compute distance (-4 because of vehicle length)
				double distance = position.distance(frontPosition) - 4;
				//if we are in position, tell the leader about that
				if (distance < 16) {
					toSend = generateMessage();
					toSend->setMessageType(JM_IN_POSITION);
					sendUnicast(toSend, positionHelper->getLeaderId());
					joinerState = JS_WAIT_JOIN;
				}
			}
			break;
		}

		case JS_WAIT_JOIN: {

			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId() && maneuver->getMessageType()!=LM_PROPOSE_AS_LEADER ) {

				//if we get confirmation from the leader, switch from faked CACC to real CACC
				if (maneuver->getMessageType() == LM_JOIN_PLATOON) {
					traciVehicle->setActiveController(Plexe::CACC);
					//set spacing to 5 meters to get close to the platoon
					traciVehicle->setCACCConstantSpacing(5);
				}
				//tell the leader that we're now in the platoon
				toSend = generateMessage();
				toSend->setMessageType(JM_IN_PLATOON);
				sendUnicast(toSend, positionHelper->getLeaderId());
				joinerState = JS_FOLLOW;
			}
			break;
		}

		case JS_FOLLOW: {
			//we're now following. if we get an update of the formation, change it accordingly
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId() && maneuver->getMessageType()!=LM_PROPOSE_AS_LEADER) {
				if (maneuver->getMessageType() == LM_UPDATE_FORMATION) {
					vehicleData.formation.clear();
					for (unsigned int i = 0; i < maneuver->getPlatoonFormationArraySize(); i++) {
						vehicleData.formation.push_back(maneuver->getPlatoonFormation(i));
					}
				}
			}
			break;
		}
	}

	if (encapsulated) {
		delete encapsulated;
	}
	if (unicast) {
		delete unicast;
	}

}

void CreatePlatoonScenario::handleFollowerMsg(cMessage *msg) {

	//this message can be a self message, or a unicast message
	//with an encapsulated beacon or maneuver message
	ManeuverMessage *maneuver = 0;
	cPacket *encapsulated = 0;

	//first check if this is a unicast message, and in case if it is a beacon or a maneuver
	UnicastMessage *unicast = dynamic_cast<UnicastMessage *>(msg);
	if (unicast) {
		encapsulated = unicast->decapsulate();
		maneuver = dynamic_cast<ManeuverMessage *>(encapsulated);
	}

	//check current follower status
	switch(followerState) {

	case FS_INIT: {
		followerState = FS_FOLLOW;
		break;
	}

	case FS_FOLLOW: {
		if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
			if (maneuver->getMessageType() == LM_UPDATE_FORMATION) {
				vehicleData.formation.clear();
				for (unsigned int i = 0; i < maneuver->getPlatoonFormationArraySize(); i++) {
					vehicleData.formation.push_back(maneuver->getPlatoonFormation(i));
				}
			}
		}
		break;
	}

	}

	if (encapsulated) {
		delete encapsulated;
	}
	if (unicast) {
		delete unicast;
	}

}

void CreatePlatoonScenario::handleLowerControl(cMessage *msg) {
	//lower control message
	UnicastProtocolControlMessage *ctrl = 0;

	ctrl = dynamic_cast<UnicastProtocolControlMessage *>(msg);
	//TODO: check for double free corruption
	if (ctrl) {
		delete ctrl;
	}
	else {
		delete msg;
	}

}
