#include "veins/modules/application/platooning/maneuver/HighwayScenario.h"

Define_Module(HighwayScenario);

void HighwayScenario::initialize(int stage) {

	BaseScenario::initialize(stage);

	if (stage == 0) {
		vehicleState = V_INIT;
		startManeuver = new cMessage("start maneuver");
		movingInPosition = new cMessage("moving in position");
		detectInFrontVehicle = new cMessage("detect in front vehicle");
	}

	if (stage == 1) {

		prepareManeuverCars();

		protocol = FindModule<BaseProtocol*>::findSubModule(getParentModule());

		//connect maneuver application to protocol
		protocol->registerApplication(MANEUVER_TYPE, gate("lowerLayerIn"), gate("lowerLayerOut"), gate("lowerControlIn"));
		//we are also interested in receiving beacons: the joiner must compute
		//its distance to the front vehicle while approaching it
		protocol->registerApplication(BaseProtocol::BEACON_TYPE, gate("lowerLayerIn"), gate("lowerLayerOut"), gate("lowerControlIn"));

		newPositionHelper = FindModule<NewPositionHelper*>::findSubModule(getParentModule());

	}

}

int HighwayScenario::determineRole(){
	int id = positionHelper->getId();
	if(id % 5 == 0){
		return 0;
	}else if ( id % 3 == 0){
		return 2;
	}else {
		return 1;
	}
}

void HighwayScenario::prepareManeuverCars() {
	std::cout<<"settaggio vettura "<< positionHelper->getId()<<endl;
	int temp = determineRole();
	role = JOIN_ROLE(temp);
	std::cout<<" ruolo"<< role<<endl;
	//int lane = positionHelper->getId()%4;
	int lane = traciVehicle->getLaneIndex();
	std::cout<<" lane"<< lane<<endl;
	vehicleData.minSpeed = (100/3.6) - 10;
	vehicleData.maxSpeed = (100/3.6) + 10 ;
	vehicleData.speed = (100/3.6) - (positionHelper->getId()%3);
	std::cout<<"speed "<<vehicleData.speed<<endl;
	//startManeuver = new cMessage();
	scheduleAt(simTime() + SimTime(5), startManeuver);

	switch (role){

		case LEADER_OR_JOINER:
		case LEADER:{
			traciVehicle->setCruiseControlDesiredSpeed(vehicleData.speed);
			traciVehicle->setActiveController(Plexe::ACC);
			traciVehicle->setFixedLane(lane);

			positionHelper->setLeaderId(positionHelper->getId());
			positionHelper->setIsLeader(true);
			positionHelper->setPlatoonLane(lane);
			positionHelper->setPlatoonId(positionHelper->getId());

			vehicleData.joinerId = -1;
			vehicleData.actualLane = lane;


			break;
		}
		case JOINER:{
			traciVehicle->setCruiseControlDesiredSpeed(vehicleData.speed);
			traciVehicle->setFixedLane(lane);
			traciVehicle->setActiveController(Plexe::ACC);

			positionHelper->setLeaderId(-1);
			positionHelper->setPlatoonId(-1);
			positionHelper->setFrontId(-1);
			positionHelper->setIsLeader(false);
			positionHelper->setPlatoonLane(-1);

			vehicleData.actualLane = lane;

			break;
		}
	}


}

void HighwayScenario::finish() {

	cancelAndDelete(startManeuver);
	cancelAndDelete(detectInFrontVehicle);
	cancelAndDelete(movingInPosition);
	startManeuver = 0;
	detectInFrontVehicle = 0;
	movingInPosition = 0;

	BaseScenario::finish();

}

ManeuverMessage *HighwayScenario::generateMessage() {
	ManeuverMessage *msg = new ManeuverMessage();
	msg->setVehicleId(positionHelper->getId());
	msg->setPlatoonId(positionHelper->getPlatoonId());
	msg->setPlatoonLane(positionHelper->getPlatoonLane());
	msg->setPlatoonSpeed(vehicleData.speed);
	return msg;
}

void HighwayScenario::handleSelfMsg(cMessage *msg) {

	//this takes car of feeding data into CACC and reschedule the self message
	BaseScenario::handleSelfMsg(msg);

	if (msg == startManeuver || msg == movingInPosition || msg == detectInFrontVehicle)
		handleVehicleMsg(msg);

}

void HighwayScenario::handleLowerMsg(cMessage *msg) {
	handleVehicleMsg(msg);
}

void HighwayScenario::sendUnicast(cPacket *msg, int destination) {
	UnicastMessage *unicast = new UnicastMessage("", MANEUVER_TYPE);
	unicast->setDestination(destination);
	unicast->setChannel(Channels::CCH);
	unicast->encapsulate(msg);
	sendDown(unicast);
}

void HighwayScenario::sendLeaderProposal(){
	std::cout<<"leader "<<positionHelper->getId()<<" send its propo"<<endl;
	ManeuverMessage *toSend;
	Coord veinsPosition = mobility->getPositionAt(simTime());
	//transform veins position into sumo position
	Veins::TraCICoord coords = mobility->getManager()->omnet2traci(veinsPosition);
	Coord position(coords.x, coords.y, 0);
	//in order to find last vehicle position
	double posX = position.x - (newPositionHelper->getPlatoonSize()*9);
	toSend = generateMessage();
	toSend->setPosX(posX);
	toSend->setPlatoonSpeed(vehicleData.speed);
	toSend->setMessageType(LM_PROPOSE_AS_LEADER);
	std::string str = newPositionHelper->getSumoId() ;
	const char* c = str.c_str();
	toSend->setSumoId(c);
	if (role == LEADER_OR_JOINER){
		toSend->setPlatoonLane(vehicleData.actualLane);
		toSend->setPlatoonId(positionHelper->getId());
	}
	sendUnicast(toSend, -1);
}

void HighwayScenario::sendNegativeAck(int destination, int type){
	ManeuverMessage *toSend;
	toSend = generateMessage();
	toSend->setMessageType(type);
	sendUnicast(toSend, destination);
}

void HighwayScenario::insertNewJoiner(int joinerId, double hisPosX){
	Coord veinsPosition = mobility->getPositionAt(simTime());
	//transform veins position into sumo position
	Veins::TraCICoord coords = mobility->getManager()->omnet2traci(veinsPosition);
	Coord position(coords.x, coords.y, 0);
	//in order to find last vehicle position
	double posX = position.x + (newPositionHelper->getPlatoonSize()*9);

	double distance = posX - hisPosX;

	JOINER_DATA temp;
	temp.id = joinerId;
	temp.distance = distance;
	vehicleData.joiners.push_back(temp);

}

void HighwayScenario::leaderDetectsInFrontVehicles(){
	double distance, relativeSpeed;
	traciVehicle->getRadarMeasurements(distance, relativeSpeed);
	std::cout<<"detecting vehicles"<<endl;
	if(distance != -1 && distance < 50 && relativeSpeed < 0){
		int nextLane = vehicleData.actualLane < 3? vehicleData.actualLane + 1 : vehicleData.actualLane - 1;
		int direction = vehicleData.actualLane > nextLane? -1 : 1;
		bool blocked = false;
		std::vector<std::string> temp = newPositionHelper->getSumoIds();
		std::vector<char*> sumoIds;
		for(unsigned int j = 0; j < temp.size(); j++){
			char* s = new char [temp[j].size() +1];
			memcpy(s, temp[j].c_str(), temp[j].size()+1);
			sumoIds.push_back(s);
		}
		if(traciVehicle->couldChangeLane(direction)){
			for(unsigned int j = 1; (j < sumoIds.size()) && !blocked; j++){
				std::cout<<"first for nested, blocked="<<blocked<<", j ="<<j<<endl;
				if(!(traci->vehicle(sumoIds[j]).couldChangeLane(direction)))
					blocked = true;
			}
		}else{
			blocked = true;
		}
		if(!blocked){
			for(unsigned int k = 0; k < sumoIds.size(); k++){
				traci->vehicle(sumoIds[k]).setFixedLane(vehicleData.actualLane + direction);
			}
			vehicleData.actualLane += direction;
			positionHelper->setPlatoonLane(vehicleData.actualLane);
		}else{
			traciVehicle->setCruiseControlDesiredSpeed(vehicleData.speed - abs(relativeSpeed));
		}
	}
}

void HighwayScenario::joinerDetectsInFrontVehicles(){
	double distance, relativeSpeed;
	traciVehicle->getRadarMeasurements(distance, relativeSpeed);
	if(distance != -1 && distance < 50 && relativeSpeed < 0){
		int nextLane = vehicleData.actualLane < 3? vehicleData.actualLane + 1 : vehicleData.actualLane - 1;
		int direction = vehicleData.actualLane > nextLane? -1 : 1;
		bool blocked = false;
		if(!traciVehicle->couldChangeLane(direction)){
			blocked = true;
		}

		if(!blocked){
			vehicleData.actualLane = nextLane;
			traciVehicle->setFixedLane(nextLane);
		}else{
			traciVehicle->setCruiseControlDesiredSpeed(30);
		}

		if(role == LEADER_OR_JOINER)
			positionHelper->setPlatoonLane(vehicleData.actualLane);
	}
}

void HighwayScenario::changeState(VEHICLE_STATES state){
	vehicleState = state;
}

void HighwayScenario::resetJoiner(){
	positionHelper->setPlatoonId(-1);
	positionHelper->setLeaderId(-1);
	positionHelper->setPlatoonLane(-1);
	newPositionHelper->setFrontVehicle(-1, "");
	newPositionHelper->setLeader(-1,"");
	if (role == JOINER){
		changeState(JS_WILL_TO_BE_FOLLOWER);
	}else{
		changeState(LJS_WILL_TO_BE_LEADER_OR_FOLLOWER);
	}

	if(movingInPosition->isScheduled())
		cancelEvent(movingInPosition);

	if(!detectInFrontVehicle->isScheduled()){
		//detectInFrontVehicle = new cMessage();
		scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
	}
	timer = 0;
}

void HighwayScenario::returnToLeadingState(){
	positionHelper->setLeaderId(positionHelper->getId());
	newPositionHelper->setLeader(positionHelper->getId(),newPositionHelper->getSumoId());
	positionHelper->setIsLeader(true);
	positionHelper->setPlatoonLane(vehicleData.actualLane);
	positionHelper->setPlatoonId(positionHelper->getId());
	if(movingInPosition->isScheduled())
		cancelEvent(movingInPosition);
	changeState(LS_LEADING);
}

void HighwayScenario::removeVehicleFromJoiners(int id){
	bool find = false;
	for(unsigned int i = 0; i < vehicleData.joiners.size() && find == false; i++){
		if(vehicleData.joiners[i].id == id){
			find = true;
			vehicleData.joiners.erase(vehicleData.joiners.begin() + i);
		}
	}
}

void HighwayScenario::leaderSendMoveOrLeading(){
	if (!vehicleData.joiners.empty()){
		ManeuverMessage *toSend;
		toSend = generateMessage();
		toSend->setMessageType(LM_MOVE_IN_POSITION);
		std::string temp = newPositionHelper->getSumoId() ;
		const char* t = temp.c_str();
		toSend->setSumoId(t);
		//this will be the front vehicle for the car which will join
		int frontVehicle = newPositionHelper->getLastVehicle();
		toSend->setFrontVehicleId(frontVehicle);
		std::string str = newPositionHelper->getLastVehicleSumoId() ;
		const char* c = str.c_str();
		toSend->setFrontVehicleSumoId(c);
		toSend->setPlatoonSpeed(vehicleData.speed);
		int i = -1;
		double distance = 10000;
		for (unsigned int j = 0; j < vehicleData.joiners.size(); j++){
			if(vehicleData.joiners[j].distance < distance){
				distance = vehicleData.joiners[j].distance;
				i = j;
			}
		}
		//who is joining?
		vehicleData.joinerId = vehicleData.joiners[i].id;
		vehicleData.joiners.erase(vehicleData.joiners.begin() + i);
		std::cout<<"leader "<<positionHelper->getId()<<" sends a move in position to "<< vehicleData.joinerId<<endl;
		//send a positive ack to the joiner
		sendUnicast(toSend, vehicleData.joinerId);

		changeState(LS_WAIT_JOINER_IN_POSITION);

	}else{
		if(newPositionHelper->getPlatoonSize() == 1 && role == LEADER_OR_JOINER)
			resetJoiner();
		else if(vehicleState != LS_LEADING)
			changeState(LS_LEADING);
	}
}

void HighwayScenario::handleVehicleMsg(cMessage *msg) {

	//this message can be a self message, or a unicast message
	//with an encapsulated maneuver message
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
	switch(vehicleState) {
		case V_INIT:{
			std::cout<<" vehicle init "<<positionHelper->getId()<<endl;
			timer = 0;
			if(role == LEADER){
				changeState(LS_LEADING);
				//detectInFrontVehicle = new cMessage();
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
			}else if (role == JOINER){
				changeState(JS_WILL_TO_BE_FOLLOWER);
				//detectInFrontVehicle = new cMessage();
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
			}else if (role == LEADER_OR_JOINER){
				changeState(LJS_WILL_TO_BE_LEADER_OR_FOLLOWER);
				//detectInFrontVehicle = new cMessage();
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
			}

			break;
		}
		case LS_LEADING: {
			std::cout<<"vehicle "<<positionHelper->getId()<<" leading"<<endl;

			if(msg == startManeuver){
				sendLeaderProposal();
			}

			if(msg == detectInFrontVehicle){
				leaderDetectsInFrontVehicles();
				//detectInFrontVehicle = new cMessage();
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
				timer ++;
				if(timer == 5){
					sendLeaderProposal();
					timer = 0;
				}
			}

			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
				if (maneuver->getMessageType() == JM_REQUEST_JOIN) {
					std::cout<<"leader gets a join request from "<<maneuver->getVehicleId()<<endl;
					toSend = generateMessage();
					toSend->setMessageType(LM_MOVE_IN_POSITION);
					std::string temp = newPositionHelper->getSumoId() ;
					const char* t = temp.c_str();
					toSend->setSumoId(t);
					//this will be the front vehicle for the car which will join
					int frontVehicle = newPositionHelper->getLastVehicle();
					toSend->setFrontVehicleId(frontVehicle);
					std::string str = newPositionHelper->getLastVehicleSumoId() ;
					const char* c = str.c_str();
					toSend->setFrontVehicleSumoId(c);
					toSend->setPlatoonSpeed(vehicleData.speed);

					//who is joining?
					vehicleData.joinerId = maneuver->getVehicleId();
					std::cout<<"leader "<<positionHelper->getId()<<" sends a move in position to "<< vehicleData.joinerId<<endl;
					//send a positive ack to the joiner
					sendUnicast(toSend, vehicleData.joinerId);

					timeoutTimer = 0;
					changeState(LS_WAIT_JOINER_IN_POSITION);
				}else if(maneuver->getMessageType() == JM_ABORT_MANEUER){
					removeVehicleFromJoiners(maneuver->getVehicleId());
				}else{
					sendNegativeAck(vehicleData.joinerId, LM_ABORT_MANEUVER);
				}
			}

			else if(maneuver && maneuver->getMessageType() == LM_PROPOSE_AS_LEADER && role == LEADER_OR_JOINER && maneuver->getVehicleId() != positionHelper->getId()){
				std::cout<<"leader "<<positionHelper->getId()<<" receive a leader propo"<<endl;
				Coord veinsPosition = mobility->getPositionAt(simTime());
				Veins::TraCICoord coords = mobility->getManager()->omnet2traci(veinsPosition);
				Coord position(coords.x, coords.y, 0);
				int myPosX = position.x;
				if (maneuver->getPlatoonSpeed() >= vehicleData.minSpeed && maneuver->getPlatoonSpeed()<= vehicleData.maxSpeed && (myPosX + 10) < maneuver->getPosX() && (maneuver->getPosX() - (myPosX + 10) < 500)){
					std::cout<<"leader "<<positionHelper->getId()<<" accept the propo, his pos: "<<myPosX<<", leader pos: "<<maneuver->getPosX();
					positionHelper->setPlatoonId(maneuver->getPlatoonId());
					positionHelper->setPlatoonLane(maneuver->getPlatoonLane());
					toSend = generateMessage();
					toSend->setMessageType(JM_REQUEST_JOIN);
					toSend->setPosX(myPosX);
					sendUnicast(toSend, positionHelper->getPlatoonId());
					std::cout<<", and so, sends a request to "<<positionHelper->getPlatoonId()<<endl;

					if(detectInFrontVehicle->isScheduled())
						cancelEvent(detectInFrontVehicle);
					//detectInFrontVehicle = new cMessage();
					scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);

					timeoutTimer = 0;
					vehicleState = LJS_WAIT_REPLY;
				}
			}

			break;
		}
		case LS_WAIT_JOINER_IN_POSITION: {

			std::cout<<"vehicle "<<positionHelper->getId()<<" wait joiner in position "<<vehicleData.joinerId<<endl;
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
				//the joiner is now in position and is ready to join+
				if (maneuver->getMessageType() == JM_IN_POSITION) {
					//tell him to join the platoon
					toSend = generateMessage();
					toSend->setMessageType(LM_JOIN_PLATOON);
					sendUnicast(toSend, vehicleData.joinerId);

					timer = 0;
					timeoutTimer = 0;
					changeState(LS_WAIT_JOINER_TO_JOIN);
				}else if (maneuver->getMessageType() == JM_REQUEST_JOIN) {
					std::cout<<"leader gets a join request from "<<maneuver->getVehicleId()<<endl;
					insertNewJoiner (maneuver->getVehicleId(), maneuver->getPosX());
				}else if (maneuver->getMessageType() == JM_ABORT_MANEUER){
					std::cout<<"vehicle "<<positionHelper->getId()<<" receive a jm abort maneuver from:"<<maneuver->getVehicleId()<<endl;
					if(maneuver->getVehicleId() == vehicleData.joinerId){
						leaderSendMoveOrLeading();
					}else{
						removeVehicleFromJoiners(maneuver->getVehicleId());
					}
				}else{
					sendNegativeAck(vehicleData.joinerId, LM_ABORT_MANEUVER);
				}
			}

			if(msg == detectInFrontVehicle){
				leaderDetectsInFrontVehicles();
				//detectInFrontVehicle = new cMessage();
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
				timer++;
				timeoutTimer++;
				if(timer == 5){
					sendLeaderProposal();
					timer = 0;
				}
				if(timeoutTimer == MAX_TIME_LEADER_WAITING_IN_POSITION){
					std::cout<<"leader "<<positionHelper->getId()<<" goes in timeout waiting "<<vehicleData.joinerId<<endl;
					sendNegativeAck(vehicleData.joinerId, LM_ABORT_MANEUVER);
					leaderSendMoveOrLeading();
					timeoutTimer = 0;
				}
			}

			break;
		}
		case LS_WAIT_JOINER_TO_JOIN: {
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
				//the joiner has joined the platoon
				if (maneuver->getMessageType() == JM_IN_PLATOON) {
					//add the joiner to the list of vehicles in the platoon
					newPositionHelper->insertFollower(vehicleData.joinerId, maneuver->getSumoId());
					std::cout<<"leader "<<positionHelper->getId()<<" inserts in platoon a follower, its sumo id: "<<maneuver->getSumoId()<<endl;

					//changeState(LS_LEADING);
					leaderSendMoveOrLeading();
				}
				else if(maneuver->getMessageType() == LM_IN_PLATOON){
					std::vector<int> platoon = maneuver->getIds();
					std::vector<char*> sumoIds = maneuver->getSumoIds();

					for(unsigned int i = 0; i < platoon.size(); i++){
						newPositionHelper->insertFollower(platoon[i], sumoIds[i]);
						std::cout<<"leader "<<positionHelper->getId()<<" inserts in platoon a follower, its sumo id: "<<sumoIds[i]<<endl;
					}

					leaderSendMoveOrLeading();

				}else if (maneuver->getMessageType() == JM_REQUEST_JOIN) {
					std::cout<<"leader gets a join request from "<<maneuver->getVehicleId()<<endl;
					insertNewJoiner (maneuver->getVehicleId(), maneuver->getPosX());
				}else if (maneuver->getMessageType() == JM_ABORT_MANEUER){
					if(maneuver->getVehicleId() == vehicleData.joinerId){
						leaderSendMoveOrLeading();
					}else{
						removeVehicleFromJoiners(maneuver->getVehicleId());
					}
				}else{
					sendNegativeAck(vehicleData.joinerId, LM_ABORT_MANEUVER);
				}
			}
			if(msg == detectInFrontVehicle){
				leaderDetectsInFrontVehicles();
				//detectInFrontVehicle = new cMessage();
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
			}

			break;
		}
		case JS_WILL_TO_BE_FOLLOWER: {
			//analyze the leader's proposal, then ask for joining
			if (maneuver && maneuver->getMessageType() == LM_PROPOSE_AS_LEADER) {
				std::cout<<"vehicle "<<positionHelper->getId()<<" receive a leader propo"<<endl;
				Coord veinsPosition = mobility->getPositionAt(simTime());
				Veins::TraCICoord coords = mobility->getManager()->omnet2traci(veinsPosition);
				Coord position(coords.x, coords.y, 0);
				int myPosX = position.x;
				if (maneuver->getPlatoonSpeed() >= vehicleData.minSpeed && maneuver->getPlatoonSpeed()<= vehicleData.maxSpeed && (myPosX + 10) < maneuver->getPosX() && (maneuver->getPosX() - (myPosX + 10) < 500)){
					std::cout<<"vehicle "<<positionHelper->getId()<<" accept the propo, his pos: "<<myPosX<<", leader pos: "<<maneuver->getPosX();
					positionHelper->setPlatoonId(maneuver->getPlatoonId());
					positionHelper->setPlatoonLane(maneuver->getPlatoonLane());
					traciVehicle->setCruiseControlDesiredSpeed(maneuver->getPlatoonSpeed());
					toSend = generateMessage();
					toSend->setMessageType(JM_REQUEST_JOIN);
					toSend->setPosX(myPosX);
					sendUnicast(toSend, positionHelper->getPlatoonId());
					std::cout<<", and so, sends a request to "<<positionHelper->getPlatoonId()<<endl;

					if(detectInFrontVehicle->isScheduled())
						cancelEvent(detectInFrontVehicle);
					//detectInFrontVehicle = new cMessage();
					scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);

					timeoutTimer = 0;
					vehicleState = JS_WAIT_REPLY;
				}
			}
			if(msg == detectInFrontVehicle){
				joinerDetectsInFrontVehicles();
				//detectInFrontVehicle = new cMessage();
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
			}

			break;
		}

		case JS_WAIT_REPLY: {
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId() && maneuver->getMessageType()!=LM_PROPOSE_AS_LEADER) {
				//if the leader told us to move in position, we can start approaching the platoon
				if (maneuver->getMessageType() == LM_MOVE_IN_POSITION) {
					newPositionHelper->setLeader(positionHelper->getPlatoonId(),maneuver->getSumoId());
					newPositionHelper->setFrontVehicle(maneuver->getFrontVehicleId(),maneuver->getFrontVehicleSumoId());
					std::cout<<"vehicle "<<positionHelper->getId()<<" receive a move in pos, leader sumo id: "<<maneuver->getSumoId()<<", front vehicle sumo id: "<<maneuver->getFrontVehicleSumoId()<<endl;
					vehicleData.joinLane = maneuver->getPlatoonLane();
					//check for correct lane. if not in correct lane, change it
					if (vehicleData.actualLane != vehicleData.joinLane) {
						traciVehicle->setFixedLane(vehicleData.joinLane);
						vehicleData.actualLane = vehicleData.joinLane;
					}
					//activate faked CACC. this way we can approach the front car using data obtained through GPS
					traciVehicle->setCACCConstantSpacing(15);
					//we have no data so far, so for the moment just initialize with some fake data
					traciVehicle->setControllerFakeData(15, (maneuver->getPlatoonSpeed()), 0, (maneuver->getPlatoonSpeed()), 0);
					//set a CC speed higher than the platoon speed to approach it
					traciVehicle->setCruiseControlDesiredSpeed((maneuver->getPlatoonSpeed()) + 30/3.6);
					traciVehicle->setActiveController(Plexe::FAKED_CACC);

					//movingInPosition = new cMessage();
					std::cout<<"vehicle "<<positionHelper->getId()<<" in js waitreply is scheduling a movinginpos"<<endl;
					scheduleAt(simTime() + SimTime(detectingVehiclesInterval), movingInPosition);

					if(detectInFrontVehicle->isScheduled())
						cancelEvent(detectInFrontVehicle);
					changeState(JS_MOVE_IN_POSITION);
				}else if (maneuver->getMessageType() == LM_ABORT_MANEUVER){
					int nextLane = vehicleData.actualLane < 3? vehicleData.actualLane + 1 : vehicleData.actualLane - 1;
					vehicleData.actualLane = nextLane;
					traciVehicle->setFixedLane(nextLane);
					resetJoiner();
				}
			}
			if(msg == detectInFrontVehicle){
				joinerDetectsInFrontVehicles();
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
				timeoutTimer++;
				if (timeoutTimer == MAX_TIME_WAITING_REPLY ){
					sendNegativeAck(positionHelper->getPlatoonId(), JM_ABORT_MANEUER);
					resetJoiner();
				}
			}

			break;
		}

		case JS_MOVE_IN_POSITION: {
			std::cout<<"vehicle "<<positionHelper->getId()<<" move in position to "<<positionHelper->getLeaderId()<<endl;

			if(msg == movingInPosition){
				double speed, acceleration, controllerAcceleration, sumoPosX, sumoPosY, sumoTime;
				std::string frontVehicleSumoId = newPositionHelper->getFrontVehicleSumoId();
				traci->vehicle(frontVehicleSumoId).getVehicleData(speed, acceleration, controllerAcceleration, sumoPosX, sumoPosY, sumoTime);
				//get front vehicle position
				Coord frontPosition(sumoPosX, sumoPosY, 0);
				//get my position
				Veins::TraCICoord traciPosition = mobility->getManager()->omnet2traci(mobility->getCurrentPosition());
				Coord position(traciPosition.x, traciPosition.y);
				//compute distance (-4 because of vehicle length)
				double distance = position.distance(frontPosition) - 4;

				double radarDistance, relativeSpeed;
				traciVehicle->getRadarMeasurements(radarDistance, relativeSpeed);
				if(!(radarDistance + 4 > distance  && radarDistance - 4 < distance) && radarDistance != -1/* && traciVehicle->getLaneIndex() == vehicleData.joinLane*/){
					sendNegativeAck(positionHelper->getLeaderId(), JM_ABORT_MANEUER);
					int nextLane = vehicleData.actualLane < 3? vehicleData.actualLane + 1 : vehicleData.actualLane - 1;
					vehicleData.actualLane = nextLane;
					traciVehicle->setFixedLane(nextLane);
					resetJoiner();
				}

				//if we are in position, tell the leader about that
				if (distance < 16) {
					toSend = generateMessage();
					toSend->setMessageType(JM_IN_POSITION);
					sendUnicast(toSend, positionHelper->getLeaderId());

					changeState(JS_WAIT_JOIN);
				}
				std::cout<<"vehicle "<<positionHelper->getId()<<" in js move in pos is scheduling a movinginpos"<<endl;
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), movingInPosition);
			}

			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()){
				if(maneuver->getMessageType() == LM_ABORT_MANEUVER){
					int nextLane = vehicleData.actualLane < 3? vehicleData.actualLane + 1 : vehicleData.actualLane - 1;
					vehicleData.actualLane = nextLane;
					traciVehicle->setFixedLane(nextLane);
					resetJoiner();
				}
			}

			break;
		}

		case JS_WAIT_JOIN: {
			std::cout<<"vehicle "<<positionHelper->getId()<<" wait join to "<<positionHelper->getLeaderId()<<endl;
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId() && maneuver->getMessageType()!=LM_PROPOSE_AS_LEADER ) {

				//if we get confirmation from the leader, switch from faked CACC to real CACC
				if (maneuver->getMessageType() == LM_JOIN_PLATOON) {
					if(traciVehicle->getLaneIndex() == positionHelper->getPlatoonLane()){
						traciVehicle->setActiveController(Plexe::CACC);
						//set spacing to 5 meters to get close to the platoon
						traciVehicle->setCACCConstantSpacing(5);
						//tell the leader that we're now in the platoon
						toSend = generateMessage();
						toSend->setMessageType(JM_IN_PLATOON);
						std::string str = newPositionHelper->getSumoId() ;
						const char* c = str.c_str();
						toSend->setSumoId(c);
						sendUnicast(toSend, positionHelper->getLeaderId());
						changeState(JS_FOLLOW);
					}else{
						sendNegativeAck(positionHelper->getLeaderId(), JM_ABORT_MANEUER);
						resetJoiner();
					}

				}else if(maneuver->getMessageType() == LM_ABORT_MANEUVER){
					int nextLane = vehicleData.actualLane < 3? vehicleData.actualLane + 1 : vehicleData.actualLane - 1;
					vehicleData.actualLane = nextLane;
					traciVehicle->setFixedLane(nextLane);
					resetJoiner();
				}
			}

			/*
			if(timer == MAX_TIME &&  vehicleState == JS_MOVE_IN_POSITION){
				//std::cout<<"vehicle "<<positionHelper->getId()<<" reach max time in move in position and sends an abort"<<endl;
				sendNegativeAck(positionHelper->getLeaderId(), JM_ABORT_MANEUER);
				resetJoiner();
			}
			timer++;
			*/
			break;
		}

		case JS_FOLLOW: {
			std::cout<<"vehicle "<<positionHelper->getId()<<" follows"<<endl;
			if(maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId() && maneuver->getMessageType()!=LM_PROPOSE_AS_LEADER){
				if(maneuver->getMessageType() == LM_CHANGE_LEADER){
					positionHelper->setLeaderId(maneuver->getLeaderId());
					positionHelper->setPlatoonId(maneuver->getLeaderId());
					newPositionHelper->setLeader(maneuver->getLeaderId(),maneuver->getLeaderSumoId());
				}
			}
			break;
		}

		case LJS_WILL_TO_BE_LEADER_OR_FOLLOWER:{
			std::cout<<"vehicle "<<positionHelper->getId()<<" leader of follower"<<endl;

			if (maneuver && maneuver->getMessageType() == JM_REQUEST_JOIN) {

				positionHelper->setLeaderId(positionHelper->getId());
				positionHelper->setIsLeader(true);
				positionHelper->setPlatoonLane(vehicleData.actualLane);
				positionHelper->setPlatoonId(positionHelper->getId());

				toSend = generateMessage();
				toSend->setMessageType(LM_MOVE_IN_POSITION);
				std::string temp = newPositionHelper->getSumoId() ;
				const char* t = temp.c_str();
				toSend->setSumoId(t);
				//this will be the front vehicle for the car which will join
				int frontVehicle = newPositionHelper->getLastVehicle();
				toSend->setFrontVehicleId(frontVehicle);
				std::string str = newPositionHelper->getLastVehicleSumoId() ;
				const char* c = str.c_str();
				toSend->setFrontVehicleSumoId(c);
				toSend->setPlatoonSpeed(vehicleData.speed);

				//who is joining?
				vehicleData.joinerId = maneuver->getVehicleId();

				std::cout<<"leader/follower "<<positionHelper->getId()<<" sends a move in position to "<< vehicleData.joinerId<<endl;
				//send a positive ack to the joiner
				sendUnicast(toSend, vehicleData.joinerId);

				changeState(LS_WAIT_JOINER_IN_POSITION);


			}else if (maneuver && maneuver->getMessageType() == LM_PROPOSE_AS_LEADER && vehicleData.joiners.empty()) {
				Coord veinsPosition = mobility->getPositionAt(simTime());
				Veins::TraCICoord coords = mobility->getManager()->omnet2traci(veinsPosition);
				Coord position(coords.x, coords.y, 0);
				int myPosX = position.x;
				std::cout<<"vehicle "<<positionHelper->getId()<<" receive a leader propo"<<endl;
				if ( maneuver->getPlatoonSpeed() >= vehicleData.minSpeed && maneuver->getPlatoonSpeed()<= vehicleData.maxSpeed && myPosX < maneuver->getPosX()){
					positionHelper->setPlatoonId(maneuver->getPlatoonId());
					positionHelper->setLeaderId(-1);
					positionHelper->setPlatoonLane(maneuver->getPlatoonLane());
					positionHelper->setFrontId(-1);
					positionHelper->setIsLeader(false);
					toSend = generateMessage();
					toSend->setMessageType(JM_REQUEST_JOIN);
					sendUnicast(toSend, positionHelper->getPlatoonId());

					timer = 0;
					changeState(JS_WAIT_REPLY);
				}
			}


			if(msg == detectInFrontVehicle){
				joinerDetectsInFrontVehicles();
				//detectInFrontVehicle = new cMessage();
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
				timer ++;
				if(timer == 8){
					sendLeaderProposal();
					timer = 0;
				}
			}

			if(msg == startManeuver){
				sendLeaderProposal();
			}

			break;
		}
		case LJS_WAIT_REPLY: {
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId() && maneuver->getMessageType()!=LM_PROPOSE_AS_LEADER) {
				//if the leader told us to move in position, we can start approaching the platoon
				if (maneuver->getMessageType() == LM_MOVE_IN_POSITION) {
					std::cout<<"move in position arrived to "<< positionHelper->getId()<<" from "<<maneuver->getPlatoonId()<<endl;
					newPositionHelper->setLeader(positionHelper->getPlatoonId(),maneuver->getSumoId());
					positionHelper->setLeaderId(positionHelper->getPlatoonId());
					std::cout<<"my leader now is: "<<positionHelper->getLeaderId();
					newPositionHelper->setFrontVehicle(maneuver->getFrontVehicleId(),maneuver->getFrontVehicleSumoId());
					positionHelper->setFrontId(maneuver->getFrontVehicleId());
					std::cout<<"vehicle "<<positionHelper->getId()<<" receive a move in pos, leader sumo id: "<<maneuver->getSumoId()<<", front vehicle sumo id: "<<maneuver->getFrontVehicleSumoId()<<endl;
					vehicleData.joinLane = maneuver->getPlatoonLane();
					//check for correct lane. if not in correct lane, change it
					bool blocked = false;
					if (vehicleData.actualLane != vehicleData.joinLane) {
						int direction = vehicleData.actualLane > vehicleData.joinLane? -1 : 1;
						std::vector<std::string> temp = newPositionHelper->getSumoIds();
						std::vector<char*> sumoIds;
						for(unsigned int j = 0; j < temp.size(); j++){
							char* s = new char [temp[j].size() +1];
							memcpy(s, temp[j].c_str(), temp[j].size()+1);
							sumoIds.push_back(s);
						}
						//for(int i = 0; (i < abs(vehicleData.actualLane - vehicleData.joinLane)) && (!blocked); i++){
						int i = 0;
						int deltaLanes = abs(vehicleData.actualLane - vehicleData.joinLane);
						while(i < deltaLanes && !blocked){
							if(traciVehicle->couldChangeLane(direction)){
								for(unsigned int j = 1; (j < sumoIds.size()) && !blocked; j++){
									if(!(traci->vehicle(sumoIds[j]).couldChangeLane(direction)))
										blocked = true;
								}
								if(!blocked){
									for(unsigned int k = 0; k < sumoIds.size(); k++){
										traci->vehicle(sumoIds[k]).setFixedLane(vehicleData.actualLane + direction);
									}
									vehicleData.actualLane += direction;
								}
							}else{
								blocked = true;
							}
							i++;
						}
					}
					if(!blocked){
						//activate faked CACC. this way we can approach the front car using data obtained through GPS
						traciVehicle->setCACCConstantSpacing(15);
						//we have no data so far, so for the moment just initialize with some fake data
						traciVehicle->setControllerFakeData(15, (maneuver->getPlatoonSpeed()), 0, (maneuver->getPlatoonSpeed()), 0);
						//set a CC speed higher than the platoon speed to approach it
						traciVehicle->setCruiseControlDesiredSpeed((maneuver->getPlatoonSpeed()) + 30/3.6);
						traciVehicle->setActiveController(Plexe::FAKED_CACC);

						std::cout<<"vehicle "<<positionHelper->getId()<<" in ljs waitreply is scheduling a movinginpos"<<endl;
						scheduleAt(simTime() + SimTime(detectingVehiclesInterval), movingInPosition);

						if(detectInFrontVehicle->isScheduled())
							cancelEvent(detectInFrontVehicle);
						changeState(LJS_MOVE_IN_POSITION);
					}else{
						sendNegativeAck(positionHelper->getLeaderId(), JM_ABORT_MANEUER);
						returnToLeadingState();
					}
				}else if (maneuver->getMessageType() == LM_ABORT_MANEUVER){
					returnToLeadingState();
					int nextLane = vehicleData.actualLane < 3? vehicleData.actualLane + 1 : vehicleData.actualLane - 1;
					vehicleData.actualLane = nextLane;
					traciVehicle->setFixedLane(nextLane);
				}
			}
			if(msg == detectInFrontVehicle){
				joinerDetectsInFrontVehicles();
				//detectInFrontVehicle = new cMessage();
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
				timeoutTimer++;
				if (timeoutTimer == MAX_TIME_WAITING_REPLY ){
					sendNegativeAck(positionHelper->getPlatoonId(), JM_ABORT_MANEUER);
					returnToLeadingState();
				}
			}

			break;
		}

		case LJS_MOVE_IN_POSITION: {
			std::cout<<"vehicle "<<positionHelper->getId()<<" move in position to "<<positionHelper->getLeaderId()<<endl;

			if(msg == movingInPosition){

				double speed, acceleration, controllerAcceleration, sumoPosX, sumoPosY, sumoTime;
				std::string frontVehicleSumoId = newPositionHelper->getFrontVehicleSumoId();
				traci->vehicle(frontVehicleSumoId).getVehicleData(speed, acceleration, controllerAcceleration, sumoPosX, sumoPosY, sumoTime);
				//get front vehicle position
				Coord frontPosition(sumoPosX, sumoPosY, 0);
				//get my position
				Veins::TraCICoord traciPosition = mobility->getManager()->omnet2traci(mobility->getCurrentPosition());
				Coord position(traciPosition.x, traciPosition.y);
				//compute distance (-4 because of vehicle length)
				double distance = position.distance(frontPosition) - 4;

				double radarDistance, relativeSpeed;
				traciVehicle->getRadarMeasurements(radarDistance, relativeSpeed);
				if(!(radarDistance + 4 > distance  && radarDistance - 4 < distance) && radarDistance != -1 && traciVehicle->getLaneIndex() == vehicleData.joinLane){
					sendNegativeAck(positionHelper->getLeaderId(), JM_ABORT_MANEUER);
					returnToLeadingState();
				}

				//if we are in position, tell the leader about that
				if (distance < 16) {
					toSend = generateMessage();
					toSend->setMessageType(JM_IN_POSITION);
					sendUnicast(toSend, positionHelper->getLeaderId());

					if(movingInPosition->isScheduled())
						cancelEvent(movingInPosition);
					changeState(LJS_WAIT_JOIN);
				}
				std::cout<<"vehicle "<<positionHelper->getId()<<" in ljs moveinpos is scheduling a movinginpos"<<endl;
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), movingInPosition);
			}

			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()){
				if(maneuver->getMessageType() == LM_ABORT_MANEUVER){
					int nextLane = vehicleData.actualLane < 3? vehicleData.actualLane + 1 : vehicleData.actualLane - 1;
					vehicleData.actualLane = nextLane;
					traciVehicle->setFixedLane(nextLane);
					returnToLeadingState();
				}
			}

			break;
		}

		case LJS_WAIT_JOIN: {
			std::cout<<"vehicle "<<positionHelper->getId()<<" wait join to "<<positionHelper->getLeaderId()<<endl;
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId() && maneuver->getMessageType()!=LM_PROPOSE_AS_LEADER ) {

				//if we get confirmation from the leader, switch from faked CACC to real CACC
				if (maneuver->getMessageType() == LM_JOIN_PLATOON) {
					if(traciVehicle->getLaneIndex() == positionHelper->getPlatoonLane()){

						traciVehicle->setActiveController(Plexe::CACC);
						//set spacing to 5 meters to get close to the platoon
						traciVehicle->setCACCConstantSpacing(5);
						//tell the leader that we're now in the platoon
						toSend = generateMessage();
						toSend->setMessageType(LM_IN_PLATOON);

						std::vector<int> platoon ;
						platoon = newPositionHelper->getPlatoon();
						std::vector<std::string> temp;
						temp = newPositionHelper->getSumoIds();
						std::vector<char*> sumoIds;
						for(unsigned int j = 0; j < temp.size(); j++){
							char* s = new char [temp[j].size() +1];
							memcpy(s, temp[j].c_str(), temp[j].size()+1);
							sumoIds.push_back(s);
						}

						toSend->setIds(platoon);
						toSend->setSumoIds(sumoIds);
						sendUnicast(toSend, positionHelper->getLeaderId());
						std::cout<<"vehicle "<<positionHelper->getId()<<"sends a lm_in_position "<<positionHelper->getLeaderId()<<endl;


						toSend = generateMessage();
						toSend->setMessageType(LM_CHANGE_LEADER);
						toSend->setLeaderId(newPositionHelper->getLeaderId());
						std::string temp1 = newPositionHelper->getLeaderSumoId() ;
						const char* t = temp1.c_str();
						toSend->setLeaderSumoId(t);
						for(unsigned int i = 0; i < platoon.size(); i++){
							sendUnicast(toSend->dup(), platoon[i]);
						}

						newPositionHelper->resetVectors();
						changeState(JS_FOLLOW);

					}else{
						sendNegativeAck(positionHelper->getLeaderId(), JM_ABORT_MANEUER);
						returnToLeadingState();
					}

				}else if(maneuver->getMessageType() == LM_ABORT_MANEUVER){
					returnToLeadingState();
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



void HighwayScenario::handleLowerControl(cMessage *msg) {
	//lower control message
	UnicastProtocolControlMessage *ctrl = 0;
	ctrl = dynamic_cast<UnicastProtocolControlMessage *>(msg);

	ManeuverMessage *toSend;
	//TODO: check for double free corruption
	if (ctrl) {
		if(ctrl->getControlCommand() == SEND_FAIL){
			if(role == LEADER){
				std::cout<<"vehicle "<<positionHelper->getId()<<" send fail"<<endl;
				changeState(LS_LEADING);
				sendNegativeAck(vehicleData.joinerId, LM_ABORT_MANEUVER);

			}else if (role == JOINER){
				sendNegativeAck(positionHelper->getPlatoonId(), JM_ABORT_MANEUER);
				resetJoiner();
			}else{
				if(vehicleState == JS_FOLLOW){

					cPacket *temp = ctrl->decapsulate();
					UnicastMessage *unicast = dynamic_cast<UnicastMessage *>(temp);

					toSend = generateMessage();
					toSend->setMessageType(LM_CHANGE_LEADER);
					toSend->setLeaderId(newPositionHelper->getLeaderId());
					std::string temp1 = newPositionHelper->getLeaderSumoId() ;
					const char* t = temp1.c_str();
					toSend->setLeaderSumoId(t);
					std::cout<<"resending the message only to the vehicle that hasn't received the unicast "<<endl;
					sendUnicast(toSend->dup(), unicast->getDestination());

				}else if(positionHelper->isLeader()){
					if(positionHelper->getPlatoonId() != positionHelper->getId())
						returnToLeadingState();
					else
						changeState(LS_LEADING);
					sendNegativeAck(vehicleData.joinerId, LM_ABORT_MANEUVER);
				}else{
					sendNegativeAck(positionHelper->getPlatoonId(), JM_ABORT_MANEUER);
					resetJoiner();
				}
			}
		}
		delete ctrl;
	}
	else {
		delete msg;
	}
}
