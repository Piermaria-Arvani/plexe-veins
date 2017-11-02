#include "veins/modules/application/platooning/maneuver/HighwayScenario.h"

Define_Module(HighwayScenario);

void HighwayScenario::initialize(int stage) {

	BaseScenario::initialize(stage);

	if (stage == 0) {
		vehicleState = V_INIT;
		startManeuver = new cMessage("start maneuver");
		movingInPosition = new cMessage("moving in position");
		detectInFrontVehicle = new cMessage("detect in front vehicle");
		gettingInHighway = new cMessage("getting in highway");
		platoonDismount = new cMessage("dismount platoon");
		MAX_LANE_INDEX = 2;


		//set names for output vectors
		//distance from front vehicle
		//vehicle id
		nodeIdOut.setName("nodeId");
		//current speed
		speedOut.setName("speed");
		//vehicle position
		posxOut.setName("posx");
		posyOut.setName("posy");
		//size of platoon
		platoonSize.setName("platoonSize");
		//
		routeOut.setName("route");
		//
		failure.setName("failure");


		speedRangeAccepted = par("speedRangeAccepted").longValue();
	}

	if (stage == 1) {

		protocol = FindModule<BaseProtocol*>::findSubModule(getParentModule());
		manager = FindModule<PlatoonsTrafficManager *>().findGlobalModule();

		//connect maneuver application to protocol
		protocol->registerApplication(MANEUVER_TYPE, gate("lowerLayerIn"), gate("lowerLayerOut"), gate("lowerControlIn"));
		//we are also interested in receiving beacons: the joiner must compute
		//its distance to the front vehicle while approaching it
		protocol->registerApplication(BaseProtocol::BEACON_TYPE, gate("lowerLayerIn"), gate("lowerLayerOut"), gate("lowerControlIn"));

		newPositionHelper = FindModule<NewPositionHelper*>::findSubModule(getParentModule());
		prepareManeuverCars();

	}

}

int HighwayScenario::determineRole(){
	int id = positionHelper->getId();
	if(id % 15 == 0){
		return 0;
	}else if ( id % 5 == 0){
		return 2;
	}else {
		return 1;
	}
}

void HighwayScenario::setDestinationId(std::string routeId){
	std::string exit = "exit";
	std::size_t indexExit = routeId.find(exit);

	if(indexExit != std::string::npos){
		std::string temp = routeId.substr (indexExit);
		char* c = (char*)temp.c_str();
		vehicleData.destionationId = c;
	}else{
		std::string temp = "end";
		char* c = (char*) temp.c_str();
		vehicleData.destionationId = c;
	}
}

void HighwayScenario::prepareManeuverCars() {

	std::string routeId = traciVehicle->getRouteId();
	setDestinationId(routeId);
	std::string str1 = routeId.substr (0,5);
	if (str1.compare("begin") == 0 || routeId.compare("platoon_route") == 0){
		vehicleData.inHighway = true;
	}else{
		vehicleData.inHighway = false;
	}

	int routeNumber = manager->getRouteNumber(routeId);
	nodeIdOut.record(positionHelper->getId());
	routeOut.record(routeNumber);


	vehicleData.lastDirection = 0;
	vehicleData.lastFailedMessageDestination = -1;

	scheduleAt(simTime() + SimTime(5), startManeuver);
	int temp = determineRole();
	role = JOIN_ROLE(temp);
	switch (role){

		case LEADER_OR_JOINER:
		case LEADER:{
			positionHelper->setLeaderId(positionHelper->getId());
			positionHelper->setIsLeader(true);
			positionHelper->setPlatoonId(positionHelper->getId());

			vehicleData.joinerId = -1;
			break;
		}
		case JOINER:{
			positionHelper->setLeaderId(-1);
			positionHelper->setPlatoonId(-1);
			positionHelper->setFrontId(-1);
			positionHelper->setIsLeader(false);
			positionHelper->setPlatoonLane(-1);
			break;
		}
	}

	vehicleData.speed = (cComponent::uniform(100, 130))/3.6;
	vehicleData.minSpeed = vehicleData.speed - speedRangeAccepted;
	vehicleData.maxSpeed = vehicleData.speed + speedRangeAccepted;
	if(vehicleData.inHighway){
		int lane = traciVehicle->getLaneIndex();
		if(positionHelper->isLeader())
			positionHelper->setPlatoonLane(lane);

		traciVehicle->setCruiseControlDesiredSpeed(vehicleData.speed);
		traciVehicle->setActiveController(Plexe::ACC);
	}else{
		traciVehicle->setActiveController(Plexe::DRIVER);
	}
}

void HighwayScenario::finish() {

	cancelAndDelete(startManeuver);
	cancelAndDelete(detectInFrontVehicle);
	cancelAndDelete(movingInPosition);
	cancelAndDelete(gettingInHighway);
	startManeuver = 0;
	detectInFrontVehicle = 0;
	movingInPosition = 0;
	gettingInHighway = 0;
	if (platoonDismount) {
		cancelAndDelete(platoonDismount);
		platoonDismount = 0;
	}

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

	if (msg == startManeuver || msg == movingInPosition || msg == detectInFrontVehicle || msg == gettingInHighway || msg == platoonDismount)
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
	const char* temp = vehicleData.destionationId.c_str();
	toSend->setDestionationId(temp);
	std::string str = newPositionHelper->getSumoId() ;
	const char* c = str.c_str();
	toSend->setSumoId(c);
	if (role == LEADER_OR_JOINER){
		toSend->setPlatoonLane(traciVehicle->getLaneIndex());
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
	if(distance != -1 && distance < 120 && relativeSpeed < 0){
		vehicleData.consecutivelyBlockedTimes++;
		if(vehicleData.consecutivelyBlockedTimes < MAX_BLOCKED_TIMES){
			int actualLane = traciVehicle->getLaneIndex();
			int nextLane;
			if(actualLane == MAX_LANE_INDEX){
				nextLane = actualLane - 1;
				vehicleData.lastDirection = -1;
			}else if(actualLane == 0){
				nextLane = 1;
				vehicleData.lastDirection = 1;
			}else{
				if(vehicleData.lastDirection == -1){
					nextLane = actualLane - 1;
					vehicleData.lastDirection = -1;
				}else{
					nextLane = actualLane + 1;
					vehicleData.lastDirection = 1;
				}
			}
			bool blocked = false;
			std::vector<std::string> temp = newPositionHelper->getSumoIds();
			std::vector<char*> sumoIds;
			for(unsigned int j = 0; j < temp.size(); j++){
				char* s = new char [temp[j].size() +1];
				memcpy(s, temp[j].c_str(), temp[j].size()+1);
				sumoIds.push_back(s);
			}
			if(traciVehicle->couldChangeLane(vehicleData.lastDirection)){
				for(unsigned int j = 1; (j < sumoIds.size()) && !blocked; j++){
					if(!(traci->vehicle(sumoIds[j]).couldChangeLane(vehicleData.lastDirection)))
						blocked = true;
				}
			}else{
				blocked = true;
			}
			if(!blocked){
				for(unsigned int k = 1; k < sumoIds.size(); k++){
					traci->vehicle(sumoIds[k]).setFixedLane(nextLane);
				}
				traciVehicle->setFixedLane(nextLane);
				positionHelper->setPlatoonLane(nextLane);
			}else{
				traciVehicle->setCruiseControlDesiredSpeed(vehicleData.speed - abs(relativeSpeed*1.5));
			}
		}else{
			traciVehicle->setCruiseControlDesiredSpeed(vehicleData.speed - abs(relativeSpeed*1.5));
		}
		vehicleData.consecutivelyBlockedTimes = 0;
	}
}

void HighwayScenario::joinerDetectsInFrontVehicles(){
	double distance, relativeSpeed;
	traciVehicle->getRadarMeasurements(distance, relativeSpeed);
	if(distance != -1 && distance < 120 && relativeSpeed < 0){
		vehicleData.consecutivelyBlockedTimes++;
		if(vehicleData.consecutivelyBlockedTimes < MAX_BLOCKED_TIMES){
			int actualLane = traciVehicle->getLaneIndex();
			int nextLane;
			if(actualLane == MAX_LANE_INDEX){
				nextLane = actualLane - 1;
				vehicleData.lastDirection = -1;
			}else if(actualLane == 0){
				nextLane = 1;
				vehicleData.lastDirection = 1;
			}else{
				if(vehicleData.lastDirection == -1){
					nextLane = actualLane - 1;
					vehicleData.lastDirection = -1;
				}else{
					nextLane = actualLane + 1;
					vehicleData.lastDirection = 1;
				}
			}
			bool blocked = true;
			if(traciVehicle->couldChangeLane(vehicleData.lastDirection)){
				blocked = false;
			}

			if(blocked){
				traciVehicle->setCruiseControlDesiredSpeed(vehicleData.speed - abs(relativeSpeed*2));
			}else{
				traciVehicle->setFixedLane(nextLane);
			}
			if(role == LEADER_OR_JOINER && positionHelper->isLeader())
				positionHelper->setPlatoonLane(nextLane);
		}else{

			traciVehicle->setCruiseControlDesiredSpeed(vehicleData.speed - abs(relativeSpeed*2));
		}

	}else if(distance < 20 && relativeSpeed == 0){
		changeOneLane();
	}else{
		vehicleData.consecutivelyBlockedTimes = 0;
	}
}

void HighwayScenario::changeOneLane(){
	int actualLane = traciVehicle->getLaneIndex();

	if(actualLane == MAX_LANE_INDEX){
		traciVehicle->setFixedLane(actualLane - 1);
		vehicleData.lastDirection = -1;
	}else if(actualLane == 0){
		traciVehicle->setFixedLane(1);
		vehicleData.lastDirection = 1;
	}else{
		if(vehicleData.lastDirection == -1){
			traciVehicle->setFixedLane(actualLane - 1);
		}else{
			traciVehicle->setFixedLane(actualLane + 1);
			vehicleData.lastDirection = 1;
		}
	}
}

void HighwayScenario::changeState(VEHICLE_STATES state){
	vehicleState = state;
	if(state == VEHICLE_STATES(14)) {
		manager->increaseExitedCars();
	}
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

void HighwayScenario::resetLeader(){
	positionHelper->setLeaderId(positionHelper->getId());
	newPositionHelper->setLeader(positionHelper->getId(),newPositionHelper->getSumoId());
	positionHelper->setIsLeader(true);
	positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());
	positionHelper->setPlatoonId(positionHelper->getId());
	if(movingInPosition->isScheduled())
		cancelEvent(movingInPosition);
	if(!detectInFrontVehicle->isScheduled())
		scheduleAt(simTime()+SimTime(detectingVehiclesInterval), detectInFrontVehicle);
	vehicleData.consecutivelyBlockedTimes = 0;
	changeState(LS_LEADING);
	changeOneLane();
}

void HighwayScenario::dismountPlatoon(){

	// leader sets lane to zero and start decreasing the speed
	traciVehicle->setFixedLane(0);
	//traciVehicle->setCruiseControlDesiredSpeed(20);

	if(newPositionHelper->getPlatoonSize() > 1){

		//send take control message to last follower
		ManeuverMessage *toSend;
		toSend = generateMessage();
		toSend->setMessageType(LM_TAKE_CAR_CONTROL);
		int lastFollower = newPositionHelper->getLastVehicle();
		sendUnicast(toSend, lastFollower);


		//change directly the lane of all the followers. For the last one change also the controller
		std::vector<std::string> temp = newPositionHelper->getSumoIds();
		std::vector<char*> sumoIds;
		if(temp.size() != 0){
			for(unsigned int j = 0; j < temp.size(); j++){
				char* s = new char [temp[j].size() +1];
				memcpy(s, temp[j].c_str(), temp[j].size()+1);
				sumoIds.push_back(s);
			}

			for(unsigned int t = (sumoIds.size()-1); t > 0 ; t--){
				traci->vehicle(sumoIds[t]).setFixedLane(0);
				if(t == sumoIds.size()-1)
					traci->vehicle(sumoIds[t]).setActiveController(Plexe::DRIVER);
			}
		}

		newPositionHelper->removeLastFollower();

		if(newPositionHelper->getPlatoonSize() == 1){
			traciVehicle->setCruiseControlDesiredSpeed(20);
			traciVehicle->setActiveController(Plexe::DRIVER);
			nodeIdOut.record(positionHelper->getId());
			speedOut.record(mobility->getCurrentSpeed().x);
			Coord pos = mobility->getPositionAt(simTime());
			posxOut.record(pos.x);
			posyOut.record(pos.y);
			platoonSize.record(1);
		}else{
			scheduleAt(simTime() + SimTime(dismountingPlatoonTime), platoonDismount);
		}
	}
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
	if (!vehicleData.joiners.empty() && traciVehicle->getDistanceToRouteEnd() >  DISTANCE_FROM_EXIT+1500){

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
		//send a positive ack to the joiner
		sendUnicast(toSend, vehicleData.joinerId);

		changeState(LS_WAIT_JOINER_IN_POSITION);

	}else{
		if(newPositionHelper->getPlatoonSize() == 1 && role == LEADER_OR_JOINER)
			resetJoiner();
		else if(vehicleState != LS_LEADING)
			changeState(LS_LEADING);
	}

	if(!detectInFrontVehicle->isScheduled())
		scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
	timer = 0;
}

void HighwayScenario::recordFailure(JOIN_FAILURE fail){
	nodeIdOut.record(positionHelper->getId());
	failure.record(fail);
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
			timer = 0;
			if(vehicleData.inHighway){
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
			}else {
				scheduleAt(simTime() + SimTime(controlInHighway), gettingInHighway);
				changeState(V_IDLE);
			}

			break;
		}
		case V_IDLE:{
			if(msg == gettingInHighway){
				if(traciVehicle->getDistanceFromRouteBegin() < 700){
					scheduleAt(simTime() + SimTime(controlInHighway), gettingInHighway);
				}else{
					int lane = traciVehicle->getLaneIndex();
					traciVehicle->setCruiseControlDesiredSpeed(vehicleData.speed);
					traciVehicle->setActiveController(Plexe::ACC);
					scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
					if(role == LEADER){
						positionHelper->setPlatoonLane(lane);
						changeState(LS_LEADING);
					}else if (role == JOINER){
						changeState(JS_WILL_TO_BE_FOLLOWER);
					}else if (role == LEADER_OR_JOINER){
						positionHelper->setPlatoonLane(lane);
						changeState(LJS_WILL_TO_BE_LEADER_OR_FOLLOWER);
					}
				}
			}
			break;
		}
		case V_EXIT:{
			if(msg == platoonDismount){

				//send take control message to last follower
				int lastFollower = newPositionHelper->getLastVehicle();
				ManeuverMessage *toSend;
				toSend = generateMessage();
				toSend->setMessageType(LM_TAKE_CAR_CONTROL);
				sendUnicast(toSend, lastFollower);


				//change directly the lane of all the followers. For the last one change also the controller
				const char* lastFollowerSumoId = (newPositionHelper->getLastVehicleSumoId().c_str());
				traci->vehicle(lastFollowerSumoId).setActiveController(Plexe::DRIVER);

				newPositionHelper->removeFollower(lastFollower);

				if(newPositionHelper->getPlatoonSize() == 1){
					traciVehicle->setCruiseControlDesiredSpeed(20);
					traciVehicle->setActiveController(Plexe::DRIVER);
					nodeIdOut.record(positionHelper->getId());
					speedOut.record(mobility->getCurrentSpeed().x);
					Coord pos = mobility->getPositionAt(simTime());
					posxOut.record(pos.x);
					posyOut.record(pos.y);
					platoonSize.record(1);
				}else{
					platoonDismount = new cMessage("dismount platoon");
					scheduleAt(simTime() + SimTime(dismountingPlatoonTime), platoonDismount);
				}
			}
			break;
		}
		case LS_LEADING: {
			if(msg == startManeuver){
				sendLeaderProposal();
			}


			if(msg == detectInFrontVehicle){
				leaderDetectsInFrontVehicles();
				if(traciVehicle->getDistanceToRouteEnd() < DISTANCE_FROM_EXIT){
					dismountPlatoon();
					changeState(V_EXIT);
				}
				//detectInFrontVehicle = new cMessage();
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
				timer ++;
				if(timer == leaderProposalInterval){
					sendLeaderProposal();
					timer = 0;
				}
			}

			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
				if (maneuver->getMessageType() == JM_REQUEST_JOIN) {
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
				Coord veinsPosition = mobility->getPositionAt(simTime());
				Veins::TraCICoord coords = mobility->getManager()->omnet2traci(veinsPosition);
				Coord position(coords.x, coords.y, 0);
				int myPosX = position.x;
				if (maneuver->getPlatoonSpeed() >= vehicleData.minSpeed && maneuver->getPlatoonSpeed()<= vehicleData.maxSpeed &&
						(myPosX + 10) < maneuver->getPosX() && (maneuver->getPosX() - myPosX < 1000)
						&& maneuver->getDestionationId() == vehicleData.destionationId && traciVehicle->getDistanceToRouteEnd() > DISTANCE_FROM_EXIT+1500){

					positionHelper->setPlatoonId(maneuver->getPlatoonId());
					positionHelper->setPlatoonLane(maneuver->getPlatoonLane());
					toSend = generateMessage();
					toSend->setMessageType(JM_REQUEST_JOIN);
					toSend->setPosX(myPosX);
					sendUnicast(toSend, positionHelper->getPlatoonId());

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

				if(traciVehicle->getDistanceToRouteEnd() < DISTANCE_FROM_EXIT){
					dismountPlatoon();
					changeState(V_EXIT);
				}

				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
				timer++;
				timeoutTimer++;
				if(timer == leaderProposalInterval){
					sendLeaderProposal();
					timer = 0;
				}
				if(timeoutTimer == MAX_TIME_LEADER_WAITING_IN_POSITION){
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
					//write data to output files
					nodeIdOut.record(positionHelper->getId());
					speedOut.record(mobility->getCurrentSpeed().x);
					Coord pos = mobility->getPositionAt(simTime());
					posxOut.record(pos.x);
					posyOut.record(pos.y);
					platoonSize.record(newPositionHelper->getPlatoonSize());

					//changeState(LS_LEADING);
					leaderSendMoveOrLeading();
				}
				else if(maneuver->getMessageType() == LM_IN_PLATOON){
					std::vector<int> platoon = maneuver->getIds();
					std::vector<char*> sumoIds = maneuver->getSumoIds();

					for(unsigned int i = 0; i < platoon.size(); i++){
						newPositionHelper->insertFollower(platoon[i], sumoIds[i]);
					}
					leaderSendMoveOrLeading();

				}else if (maneuver->getMessageType() == JM_REQUEST_JOIN) {
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
				if(traciVehicle->getDistanceToRouteEnd() < DISTANCE_FROM_EXIT){
					dismountPlatoon();
					changeState(V_EXIT);
				}
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
			}

			break;
		}
		case JS_WILL_TO_BE_FOLLOWER: {
			//analyze the leader's proposal, then ask for joining
			if (maneuver && maneuver->getMessageType() == LM_PROPOSE_AS_LEADER) {
				Coord veinsPosition = mobility->getPositionAt(simTime());
				Veins::TraCICoord coords = mobility->getManager()->omnet2traci(veinsPosition);
				Coord position(coords.x, coords.y, 0);
				int myPosX = position.x;

				if (maneuver->getPlatoonSpeed() >= vehicleData.minSpeed && maneuver->getPlatoonSpeed()<= vehicleData.maxSpeed &&
						(myPosX + 50) < maneuver->getPosX() && (maneuver->getPosX() - myPosX < MAX_DISTANCE_FROM_PLATOON_TO_JOIN)
						&& maneuver->getDestionationId() == vehicleData.destionationId && traciVehicle->getDistanceToRouteEnd() > DISTANCE_FROM_EXIT+1500){
					positionHelper->setPlatoonId(maneuver->getPlatoonId());
					positionHelper->setPlatoonLane(maneuver->getPlatoonLane());
					if(mobility->getCurrentSpeed().x > maneuver->getPlatoonSpeed())
						traciVehicle->setCruiseControlDesiredSpeed(maneuver->getPlatoonSpeed());
					toSend = generateMessage();
					toSend->setMessageType(JM_REQUEST_JOIN);
					toSend->setPosX(myPosX);
					sendUnicast(toSend, positionHelper->getPlatoonId());

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
				if(traciVehicle->getDistanceToRouteEnd() < DISTANCE_FROM_EXIT){
					traciVehicle->setCruiseControlDesiredSpeed(20);
					traciVehicle->setFixedLane(0);
					traciVehicle->setActiveController(Plexe::DRIVER);
					changeState(V_EXIT);
				}
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
					vehicleData.joinLane = maneuver->getPlatoonLane();
					vehicleData.platoonSpeed = maneuver->getPlatoonSpeed();


					int actualLane = traciVehicle->getLaneIndex();
					int direction = actualLane > traci->vehicle(newPositionHelper->getLeaderSumoId()).getLaneIndex()? -1 : 1;
					//check for correct lane. if not in correct lane, change it
					if (actualLane != traci->vehicle(newPositionHelper->getLeaderSumoId()).getLaneIndex() && traciVehicle->couldChangeLane(direction)) {
						traciVehicle->setFixedLane(traci->vehicle(newPositionHelper->getLeaderSumoId()).getLaneIndex());
					}

					if(movingInPosition->isScheduled())
						cancelEvent(movingInPosition);
					scheduleAt(simTime() + SimTime(detectingVehiclesInterval), movingInPosition);

					if(detectInFrontVehicle->isScheduled())
						cancelEvent(detectInFrontVehicle);
					changeState(JS_MOVE_IN_POSITION);
				}else if (maneuver->getMessageType() == LM_ABORT_MANEUVER){
					changeOneLane();
					resetJoiner();
				}else if(maneuver->getMessageType() == LM_ABORT_MANEUVER_SEND_FAIL){
					changeOneLane();
					resetJoiner();
					recordFailure(MESSAGE_FAIL);
				}
			}
			if(msg == detectInFrontVehicle){
				joinerDetectsInFrontVehicles();
				if(traciVehicle->getDistanceToRouteEnd() < 700){
					traciVehicle->setCruiseControlDesiredSpeed(20);
					traciVehicle->setFixedLane(0);
					traciVehicle->setActiveController(Plexe::DRIVER);
					changeState(V_EXIT);
				}
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
				timeoutTimer++;
				if (timeoutTimer == MAX_TIME_WAITING_REPLY ){
					recordFailure(WAIT_REPLY_TIME_OUT);
					sendNegativeAck(positionHelper->getPlatoonId(), JM_ABORT_MANEUER);
					resetJoiner();
				}
			}

			break;
		}

		case JS_MOVE_IN_POSITION: {
			if(msg == movingInPosition){
				if (traciVehicle->getLaneIndex() != traci->vehicle(newPositionHelper->getLeaderSumoId()).getLaneIndex()){
					int leaderLane = traci->vehicle(newPositionHelper->getLeaderSumoId()).getLaneIndex();
					int direction = traciVehicle->getLaneIndex() > leaderLane? -1 : 1;
					if(traciVehicle->couldChangeLane(direction)) {
						traciVehicle->setFixedLane(leaderLane);
					}
				}else{
					if(traciVehicle->getActiveController() != 3){

						//activate faked CACC. this way we can approach the front car using data obtained through GPS
						traciVehicle->setCACCConstantSpacing(15);
						//we have no data so far, so for the moment just initialize with some fake data
						traciVehicle->setControllerFakeData(15, vehicleData.platoonSpeed, 0, vehicleData.platoonSpeed, 0);
						//set a CC speed higher than the platoon speed to approach it
						traciVehicle->setCruiseControlDesiredSpeed(vehicleData.platoonSpeed + 30/3.6);
						traciVehicle->setActiveController(Plexe::FAKED_CACC);
					}
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
					if(!(radarDistance + 1 > distance  && radarDistance - 1 < distance) && radarDistance != -1){
						sendNegativeAck(positionHelper->getLeaderId(), JM_ABORT_MANEUER);
						recordFailure(VEHICLE_INTRUSION);
						changeOneLane();
						resetJoiner();
					}else if(traciVehicle->getDistanceToRouteEnd() < 700){
						traciVehicle->setCruiseControlDesiredSpeed(20);
						traciVehicle->setFixedLane(0);
						traciVehicle->setActiveController(Plexe::DRIVER);
						changeState(V_EXIT);
					}

					//if we are in position, tell the leader about that
					if (distance < 16) {
						toSend = generateMessage();
						toSend->setMessageType(JM_IN_POSITION);
						sendUnicast(toSend, positionHelper->getLeaderId());

						changeState(JS_WAIT_JOIN);
					}
				}
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), movingInPosition);
			}

			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()){
				if(maneuver->getMessageType() == LM_ABORT_MANEUVER){
					changeOneLane();
					resetJoiner();
				}else if(maneuver->getMessageType() == LM_ABORT_MANEUVER_SEND_FAIL){
					changeOneLane();
					resetJoiner();
					recordFailure(MESSAGE_FAIL);
				}
			}

			break;
		}

		case JS_WAIT_JOIN: {
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
						recordFailure(VEHICLE_INTRUSION);
						sendNegativeAck(positionHelper->getLeaderId(), JM_ABORT_MANEUER);
						resetJoiner();
					}

				}else if(maneuver->getMessageType() == LM_ABORT_MANEUVER){
					changeOneLane();
					resetJoiner();
				}else if(maneuver->getMessageType() == LM_ABORT_MANEUVER_SEND_FAIL){
					changeOneLane();
					resetJoiner();
					recordFailure(MESSAGE_FAIL);
				}
			}
			break;
		}

		case JS_FOLLOW: {
			if(maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId() && maneuver->getMessageType()!=LM_PROPOSE_AS_LEADER){
				if(maneuver->getMessageType() == LM_CHANGE_LEADER){
					positionHelper->setLeaderId(maneuver->getLeaderId());
					positionHelper->setPlatoonId(maneuver->getLeaderId());
					newPositionHelper->setLeader(maneuver->getLeaderId(),maneuver->getLeaderSumoId());
				}else if(maneuver->getMessageType() == LM_TAKE_CAR_CONTROL){
					resetJoiner();
					changeState(V_EXIT);
				}
			}
			break;
		}

		case LJS_WILL_TO_BE_LEADER_OR_FOLLOWER:{
			if (maneuver && maneuver->getMessageType() == JM_REQUEST_JOIN) {
				positionHelper->setLeaderId(positionHelper->getId());
				positionHelper->setIsLeader(true);
				positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());
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

				//send a positive ack to the joiner
				sendUnicast(toSend, vehicleData.joinerId);

				changeState(LS_WAIT_JOINER_IN_POSITION);


			}else if (maneuver && maneuver->getMessageType() == LM_PROPOSE_AS_LEADER && vehicleData.joiners.empty()) {
				Coord veinsPosition = mobility->getPositionAt(simTime());
				Veins::TraCICoord coords = mobility->getManager()->omnet2traci(veinsPosition);
				Coord position(coords.x, coords.y, 0);
				int myPosX = position.x;
				if ( maneuver->getPlatoonSpeed() >= vehicleData.minSpeed && maneuver->getPlatoonSpeed()<= vehicleData.maxSpeed &&
						(myPosX + 50) < maneuver->getPosX() && (maneuver->getPosX() - myPosX < MAX_DISTANCE_FROM_PLATOON_TO_JOIN)
						&& maneuver->getDestionationId() == vehicleData.destionationId && traciVehicle->getDistanceToRouteEnd() > DISTANCE_FROM_EXIT+1500){
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
				if(traciVehicle->getDistanceToRouteEnd() < 700){
					traciVehicle->setCruiseControlDesiredSpeed(20);
					traciVehicle->setFixedLane(0);
					traciVehicle->setActiveController(Plexe::DRIVER);
					changeState(V_EXIT);
				}
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
				timer ++;
				if(timer == leaderProposalInterval){
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
					newPositionHelper->setLeader(positionHelper->getPlatoonId(),maneuver->getSumoId());
					positionHelper->setLeaderId(positionHelper->getPlatoonId());
					positionHelper->setIsLeader(false);
					newPositionHelper->setFrontVehicle(maneuver->getFrontVehicleId(),maneuver->getFrontVehicleSumoId());
					positionHelper->setFrontId(maneuver->getFrontVehicleId());
					vehicleData.joinLane = maneuver->getPlatoonLane();
					//check for correct lane. if not in correct lane, change it
					bool blocked = false;
					if (traciVehicle->getLaneIndex() != traci->vehicle(newPositionHelper->getLeaderSumoId()).getLaneIndex()) {
						int direction = traciVehicle->getLaneIndex() > traci->vehicle(newPositionHelper->getLeaderSumoId()).getLaneIndex()? -1 : 1;
						std::vector<std::string> temp = newPositionHelper->getSumoIds();
						std::vector<char*> sumoIds;
						for(unsigned int j = 0; j < temp.size(); j++){
							char* s = new char [temp[j].size() +1];
							memcpy(s, temp[j].c_str(), temp[j].size()+1);
							sumoIds.push_back(s);
						}
						//for(int i = 0; (i < abs(vehicleData.actualLane - vehicleData.joinLane)) && (!blocked); i++){
						int i = 0;
						int deltaLanes = abs(traciVehicle->getLaneIndex() - traci->vehicle(newPositionHelper->getLeaderSumoId()).getLaneIndex());
						while(i < deltaLanes && !blocked){
							if(traciVehicle->couldChangeLane(direction)){
								for(unsigned int j = 1; (j < sumoIds.size()) && !blocked; j++){
									if(!(traci->vehicle(sumoIds[j]).couldChangeLane(direction)))
										blocked = true;
								}
								if(!blocked){
									for(unsigned int k = 0; k < sumoIds.size(); k++){
										traci->vehicle(sumoIds[k]).setFixedLane(traciVehicle->getLaneIndex() + direction);
									}
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

						scheduleAt(simTime() + SimTime(detectingVehiclesInterval), movingInPosition);

						if(detectInFrontVehicle->isScheduled())
							cancelEvent(detectInFrontVehicle);
						changeState(LJS_MOVE_IN_POSITION);
					}else{
						sendNegativeAck(positionHelper->getLeaderId(), JM_ABORT_MANEUER);
						resetLeader();
					}
				}else if (maneuver->getMessageType() == LM_ABORT_MANEUVER){
					resetLeader();
					changeOneLane();
				}else if(maneuver->getMessageType() == LM_ABORT_MANEUVER_SEND_FAIL){
					resetLeader();
					changeOneLane();
					recordFailure(MESSAGE_FAIL);
				}
			}
			if(msg == detectInFrontVehicle){
				joinerDetectsInFrontVehicles();
				//detectInFrontVehicle = new cMessage();
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), detectInFrontVehicle);
				timeoutTimer++;
				if (timeoutTimer == MAX_TIME_WAITING_REPLY ){
					recordFailure(WAIT_REPLY_TIME_OUT);
					sendNegativeAck(positionHelper->getPlatoonId(), JM_ABORT_MANEUER);
					resetLeader();
				}
			}

			break;
		}

		case LJS_MOVE_IN_POSITION: {
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
				if(!(radarDistance + 1 > distance  && radarDistance - 1 < distance) && radarDistance != -1 && traciVehicle->getLaneIndex() == traci->vehicle(newPositionHelper->getLeaderSumoId()).getLaneIndex()){
					sendNegativeAck(positionHelper->getLeaderId(), JM_ABORT_MANEUER);
					resetLeader();
					recordFailure(VEHICLE_INTRUSION);
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
				scheduleAt(simTime() + SimTime(detectingVehiclesInterval), movingInPosition);
			}

			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()){
				if(maneuver->getMessageType() == LM_ABORT_MANEUVER){
					changeOneLane();
					resetLeader();
				}else if(maneuver->getMessageType() == LM_ABORT_MANEUVER_SEND_FAIL){
					recordFailure(MESSAGE_FAIL);
					changeOneLane();
					resetLeader();
				}
			}

			break;
		}

		case LJS_WAIT_JOIN: {
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
						recordFailure(VEHICLE_INTRUSION);
						sendNegativeAck(positionHelper->getLeaderId(), JM_ABORT_MANEUER);
						resetLeader();
					}

				}else if(maneuver->getMessageType() == LM_ABORT_MANEUVER){
					changeOneLane();
					resetLeader();
				}else if(maneuver->getMessageType() == LM_ABORT_MANEUVER_SEND_FAIL){
					recordFailure(MESSAGE_FAIL);
					changeOneLane();
					resetLeader();
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
			UnicastMessage *unicast = 0;
			unicast = dynamic_cast<UnicastMessage *>(ctrl->decapsulate());
			int destination = unicast->getDestination();
			if(role == LEADER){
				if(vehicleState == V_EXIT){
					cPacket *tmp = unicast->decapsulate();
					ManeuverMessage *temp = dynamic_cast<ManeuverMessage *>(tmp);
					if(temp){
						if(temp->getMessageType() == LM_TAKE_CAR_CONTROL){
							toSend = generateMessage();
							toSend->setMessageType(LM_TAKE_CAR_CONTROL);

							sendUnicast(toSend, destination);
						}
					}
				}else{
					leaderSendMoveOrLeading();
					if(vehicleData.lastFailedMessageDestination == destination){
						counterSendFail++;
						if(counterSendFail < maxSendFail){
							sendNegativeAck(destination, LM_ABORT_MANEUVER_SEND_FAIL);
						}else{
							counterSendFail = 0;
						}
					}else {
						vehicleData.lastFailedMessageDestination = destination;
						counterSendFail++;
						sendNegativeAck(destination, LM_ABORT_MANEUVER_SEND_FAIL);
					}
				}
			}else if (role == JOINER){
				if(vehicleData.lastFailedMessageDestination == destination){
					counterSendFail++;
					if(counterSendFail < maxSendFail){
						sendNegativeAck(destination, JM_ABORT_MANEUER);
					}else{
						counterSendFail = 0;
					}
				}else {
					vehicleData.lastFailedMessageDestination = destination;
					counterSendFail++;
					sendNegativeAck(destination, JM_ABORT_MANEUER);
					recordFailure(MESSAGE_FAIL);
					resetJoiner();
				}

			}else{
				if(vehicleState == JS_FOLLOW){

					toSend = generateMessage();
					toSend->setMessageType(LM_CHANGE_LEADER);
					toSend->setLeaderId(newPositionHelper->getLeaderId());
					std::string temp1 = newPositionHelper->getLeaderSumoId() ;
					const char* t = temp1.c_str();
					toSend->setLeaderSumoId(t);
					sendUnicast(toSend, unicast->getDestination());

				}else if(positionHelper->isLeader()){
					if(vehicleState ==  V_EXIT){
						ManeuverMessage *toSend;
						toSend = generateMessage();
						toSend->setMessageType(LM_TAKE_CAR_CONTROL);
						sendUnicast(toSend, destination);
					}else{
						if(counterSendFail > 0){
							if(positionHelper->getPlatoonId() != positionHelper->getId())
								resetLeader();
							else
								leaderSendMoveOrLeading();
						}

						if(vehicleData.lastFailedMessageDestination == destination){
							counterSendFail++;
							if(counterSendFail < maxSendFail){
								sendNegativeAck(destination, LM_ABORT_MANEUVER_SEND_FAIL);
							}else{
								counterSendFail = 0;
							}
						}else {
							vehicleData.lastFailedMessageDestination = destination;
							counterSendFail++;
							sendNegativeAck(destination, LM_ABORT_MANEUVER_SEND_FAIL);
						}
					}
				}else{
					sendNegativeAck(positionHelper->getPlatoonId(), JM_ABORT_MANEUER);
					resetJoiner();
				}
			}
			delete unicast;
		}
		delete ctrl;

	}
	else {
		delete msg;
	}
}
