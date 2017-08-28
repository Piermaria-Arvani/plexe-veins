#include "veins/modules/application/platooning/maneuver/HighwayScenario.h"

Define_Module(HighwayScenario);

void HighwayScenario::initialize(int stage) {

	BaseScenario::initialize(stage);

	if (stage == 0) {
		vehicleState = V_INIT;
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

		if(positionHelper->getId() == 0){
			//schedule the start of the maneuver
			startManeuver = new cMessage();
			scheduleAt(simTime() + SimTime(10), startManeuver);
		}
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
	int lane = positionHelper->getId()%4;
	std::cout<<" lane"<< lane<<endl;
	vehicleData.minSpeed = 20;
	vehicleData.maxSpeed = 40 ;
	vehicleData.speed = 30 + (positionHelper->getId()%3) * 2;
	std::cout<<"speed "<<vehicleData.speed<<endl;

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

	if (startManeuver) {
		cancelAndDelete(startManeuver);
		startManeuver = 0;
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

	if (msg == startManeuver)
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
	double posX = position.x + (newPositionHelper->getPlatoonSize()*9);
	toSend = generateMessage();
	toSend->setPosX(posX);
	toSend->setPlatoonSpeed(vehicleData.speed);
	toSend->setMessageType(LM_PROPOSE_AS_LEADER);
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

void HighwayScenario::controlNextJoiner(){
	unsigned int i = 0;
	while (i < vehicleData.timeOutJoiners.size() && !vehicleData.timeOutJoiners.empty() && !vehicleData.joiners.empty()){
		if (vehicleData.joiners.front() == vehicleData.timeOutJoiners[i]){
			vehicleData.joiners.pop();
			vehicleData.timeOutJoiners.erase(vehicleData.timeOutJoiners.begin()+i);
			i = 0;
		}
		i++;
	}
}

void HighwayScenario::insertNewJoiner(int joinerId){
	vehicleData.joiners.push(joinerId);
}

void HighwayScenario::leaderDetectsInFrontVehicles(){
	double distance, relativeSpeed;
	traciVehicle->getRadarMeasurements(distance, relativeSpeed);
	if(distance != -1 && distance < 10 && relativeSpeed < 0){
		if(vehicleData.actualLane < 3){
			traciVehicle->setFixedLane(vehicleData.actualLane + 1);
			vehicleData.actualLane += 1;
		}else{
			traciVehicle->setFixedLane(vehicleData.actualLane - 1);
			vehicleData.actualLane -= 1;
		}
		positionHelper->setPlatoonLane(vehicleData.actualLane);
	}
}

void HighwayScenario::joinerDetectsInFrontVehicles(){
	double distance, relativeSpeed;
	traciVehicle->getRadarMeasurements(distance, relativeSpeed);
	if(distance != -1 && distance < 10 && relativeSpeed < 0){
		if(vehicleData.actualLane < 3){
			traciVehicle->setFixedLane(vehicleData.actualLane + 1);
			vehicleData.actualLane += 1;
		}else{
			traciVehicle->setFixedLane(vehicleData.actualLane - 1);
			vehicleData.actualLane -= 1;
		}
	}
}

void HighwayScenario::changeState(VEHICLE_STATES state){
	vehicleState = state;
	timer = 0;
}

void HighwayScenario::handleVehicleMsg(cMessage *msg) {

	//this message can be a self message, or a unicast message
	//with an encapsulated beacon or maneuver message
	ManeuverMessage *maneuver = 0;
	cPacket *encapsulated = 0;
	PlatooningBeacon *beacon = 0;
	//maneuver message to be sent, if needed
	ManeuverMessage *toSend;

	//first check if this is a unicast message, and in case if it is a beacon or a maneuver
	UnicastMessage *unicast = dynamic_cast<UnicastMessage *>(msg);
	if (unicast) {
		encapsulated = unicast->decapsulate();
		maneuver = dynamic_cast<ManeuverMessage *>(encapsulated);
		beacon = dynamic_cast<PlatooningBeacon *>(encapsulated);
	}

	//check current leader status
	switch(vehicleState) {
		case V_INIT: {
			vehicleState = V_IDLE;
		}
		case V_IDLE:{
			if(role == LEADER){
				changeState(LS_LEADING);
			}else if (role == JOINER){
				changeState(JS_WILL_TO_BE_FOLLOWER);
			}else if (role == LEADER_OR_JOINER){
				changeState(LJS_WILL_TO_BE_LEADER_OR_FOLLOWER);
			}
			if(!vehicleData.joiners.empty()){
				vehicleData.joiners.pop();
			}
			vehicleData.timeOutJoiners.clear();
			break;
		}
		case LS_LEADING: {
			std::cout<<"vehicle "<<positionHelper->getId()<<" leading"<<endl;

			if(timer == (positionHelper->getId() + 5) || (timer % (100+positionHelper->getId()) == 0)){
				leaderDetectsInFrontVehicles();
				//notify to other cars the will to be the leader
				sendLeaderProposal();
			}
			timer++;

			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
				if (maneuver->getMessageType() == JM_REQUEST_JOIN) {
					insertNewJoiner(maneuver->getVehicleId());
				}else if(maneuver->getMessageType() == JM_ABORT_MANEUER){
					vehicleData.timeOutJoiners.push_back(maneuver->getVehicleId());
				}
			}

			if (!vehicleData.joiners.empty()){
				controlNextJoiner();
				if(!vehicleData.joiners.empty()){
					toSend = generateMessage();
					toSend->setMessageType(LM_MOVE_IN_POSITION);
					//this will be the front vehicle for the car which will join
					int frontVehicle = newPositionHelper->getLastVehicle();
					toSend->setFrontVehicleId(frontVehicle);
					toSend->setPlatoonSpeed(vehicleData.speed);
					//who is joining?
					vehicleData.joinerId = vehicleData.joiners.front();
					vehicleData.joiners.pop();
					std::cout<<"leader "<<positionHelper->getId()<<" sends a move in position to "<< vehicleData.joinerId<<endl;
					//send a positive ack to the joiner
					sendUnicast(toSend, vehicleData.joinerId);

					changeState(LS_WAIT_JOINER_IN_POSITION);
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

					changeState(LS_WAIT_JOINER_TO_JOIN);
				}else if (maneuver->getMessageType() == JM_REQUEST_JOIN) {
					insertNewJoiner(maneuver->getVehicleId());
				}else if (maneuver->getMessageType() == JM_ABORT_MANEUER){
					std::cout<<"leader "<<positionHelper->getId()<<" gets abort maneuver in wait joiner position from "<<maneuver->getVehicleId()<<endl;
					if(maneuver->getVehicleId() == vehicleData.joinerId){
						changeState(LS_LEADING);
					}else{
						vehicleData.timeOutJoiners.push_back(maneuver->getVehicleId());
					}
				}
			}
			if(timer == (positionHelper->getId() + 5) || (timer % (100+positionHelper->getId()) == 0)){
				//notify to other cars the will to be the leader
				sendLeaderProposal();
			}
			if(timer % (5 + positionHelper->getId())){
				leaderDetectsInFrontVehicles();
			}

			timer++;
			if(timer == MAX_TIME){
				vehicleData.joiners.pop();
				sendNegativeAck(vehicleData.joinerId, LM_ABORT_MANEUVER);

				changeState(LS_LEADING);
			}
			break;
		}
		case LS_WAIT_JOINER_TO_JOIN: {
			std::cout<<"vehicle "<<positionHelper->getId()<<" wait joiner to join"<<endl;
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId()) {
				//the joiner has joined the platoon
				if (maneuver->getMessageType() == JM_IN_PLATOON) {
					//add the joiner to the list of vehicles in the platoon
					newPositionHelper->insertFollower(vehicleData.joinerId);

					changeState(LS_LEADING);
				}
				else if (maneuver->getMessageType() == JM_REQUEST_JOIN) {
					insertNewJoiner(maneuver->getVehicleId());
				}else if (maneuver->getMessageType() == JM_ABORT_MANEUER){
					std::cout<<"leader "<<positionHelper->getId()<<" gets abort maneuver in wait joiner to join from "<<maneuver->getFrontVehicleId()<<endl;
					if(maneuver->getVehicleId() == vehicleData.joinerId){
						changeState(LS_LEADING);
					}else{
						vehicleData.timeOutJoiners.push_back(maneuver->getVehicleId());
					}
				}
			}
			if(timer % (5 + positionHelper->getId())){
				leaderDetectsInFrontVehicles();
			}

			break;
		}
		case JS_WILL_TO_BE_FOLLOWER: {
			std::cout<<"vehicle "<<positionHelper->getId()<<" will to be follower"<<endl;
			//analyze the leader's proposal, then ask for joining
			if (maneuver && maneuver->getMessageType() == LM_PROPOSE_AS_LEADER) {
				Coord veinsPosition = mobility->getPositionAt(simTime());
				Veins::TraCICoord coords = mobility->getManager()->omnet2traci(veinsPosition);
				Coord position(coords.x, coords.y, 0);
				int myPosX = position.x;
				if (maneuver->getPlatoonSpeed() >= vehicleData.minSpeed && maneuver->getPlatoonSpeed()<= vehicleData.maxSpeed && myPosX < maneuver->getPosX()){
					std::cout<<"propose received from "<<maneuver->getVehicleId()<<" platoon id "<<maneuver->getPlatoonId()<<endl;
					positionHelper->setPlatoonId(maneuver->getPlatoonId());
					positionHelper->setPlatoonLane(maneuver->getPlatoonLane());
					traciVehicle->setCruiseControlDesiredSpeed(maneuver->getPlatoonSpeed());
					toSend = generateMessage();
					toSend->setMessageType(JM_REQUEST_JOIN);
					sendUnicast(toSend, positionHelper->getPlatoonId());

					changeState(JS_WAIT_REPLY);
				}
			}
			if(timer % (5 + positionHelper->getId())){
				leaderDetectsInFrontVehicles();
			}

			break;
		}

		case JS_WAIT_REPLY: {
			std::cout<<"vehicle "<<positionHelper->getId()<<" wait reply from "<<positionHelper->getPlatoonId()<<endl;
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId() && maneuver->getMessageType()!=LM_PROPOSE_AS_LEADER) {
				//if the leader told us to move in position, we can start approaching the platoon
				if (maneuver->getMessageType() == LM_MOVE_IN_POSITION) {
					std::cout<<"move in position arrived to "<< positionHelper->getId()<<" from "<<maneuver->getPlatoonId()<<endl;
					positionHelper->setLeaderId(positionHelper->getPlatoonId());//////////////////
					positionHelper->setFrontId(maneuver->getFrontVehicleId());
					vehicleData.joinLane = maneuver->getPlatoonLane();
					//check for correct lane. if not in correct lane, change it
					if (vehicleData.actualLane != vehicleData.joinLane) {
						std::cout<<"vehicle "<<positionHelper->getId()<<" lane change, my actual lane is"<<vehicleData.actualLane<<"i need to go to lane"<<vehicleData.joinLane<<endl;

						traciVehicle->setFixedLane(vehicleData.joinLane);
						vehicleData.actualLane = vehicleData.joinLane;
					}
					//activate faked CACC. this way we can approach the front car using data obtained through GPS
					traciVehicle->setCACCConstantSpacing(15);
					//we have no data so far, so for the moment just initialize with some fake data
					traciVehicle->setControllerFakeData(15, vehicleData.speed, 0, vehicleData.speed, 0);
					//set a CC speed higher than the platoon speed to approach it
					traciVehicle->setCruiseControlDesiredSpeed((maneuver->getPlatoonSpeed()) + 30/3.6);
					traciVehicle->setActiveController(Plexe::FAKED_CACC);

					changeState(JS_MOVE_IN_POSITION);
				}else if (maneuver->getMessageType() == LM_ABORT_MANEUVER){
					positionHelper->setPlatoonId(-1);
					positionHelper->setLeaderId(-1);
					positionHelper->setPlatoonLane(-1);
					if (role == JOINER){
						changeState(JS_WILL_TO_BE_FOLLOWER);
					}else{
						changeState(LJS_WILL_TO_BE_LEADER_OR_FOLLOWER);
					}
				}
			}
			else if (!maneuver || (maneuver && maneuver->getMessageType() != LM_MOVE_IN_POSITION)){
				joinerDetectsInFrontVehicles();
			}

			timer ++;
			if(timer== MAX_TIME && vehicleState == JS_WAIT_REPLY){

				sendNegativeAck(positionHelper->getLeaderId(),JM_ABORT_MANEUER);
				positionHelper->setPlatoonId(-1);
				positionHelper->setLeaderId(-1);
				positionHelper->setPlatoonLane(-1);
				if (role == JOINER){
					changeState(JS_WILL_TO_BE_FOLLOWER);
				}else{
					changeState(LJS_WILL_TO_BE_LEADER_OR_FOLLOWER);
				}
			}
			break;
		}

		case JS_MOVE_IN_POSITION: {
			std::cout<<"vehicle "<<positionHelper->getId()<<" move in position to "<<positionHelper->getLeaderId()<<endl;
			//if we get data, just feed the fake CACC
			if (beacon && beacon->getVehicleId() == positionHelper->getFrontId()) {
				//get front vehicle position
				Coord frontPosition(beacon->getPositionX(), beacon->getPositionY(), 0);
				//get my position
				Veins::TraCICoord traciPosition = mobility->getManager()->omnet2traci(mobility->getCurrentPosition());
				Coord position(traciPosition.x, traciPosition.y);
				//compute distance (-4 because of vehicle length)
				double distance = position.distance(frontPosition) - 4;

				double radarDistance, relativeSpeed;
				traciVehicle->getRadarMeasurements(radarDistance, relativeSpeed);
				if(!(radarDistance + 1 > distance  && radarDistance - 1 < distance) && radarDistance != -1 && traciVehicle->getLaneIndex() == vehicleData.joinLane){
					std::cout<<"distance from beacon "<<distance<<", distance from radar "<<radarDistance<<endl;
					std::cout<<"joiner "<<positionHelper->getId()<<" sends abort maneuver in move in position to "<<positionHelper->getLeaderId()<<endl;
					sendNegativeAck(positionHelper->getLeaderId(),JM_ABORT_MANEUER);
					positionHelper->setPlatoonId(-1);
					positionHelper->setLeaderId(-1);
					positionHelper->setPlatoonLane(-1);
					if (role == JOINER){
						changeState(JS_WILL_TO_BE_FOLLOWER);
					}else{
						changeState(LJS_WILL_TO_BE_LEADER_OR_FOLLOWER);
					}
				}
				//if we are in position, tell the leader about that
				if (distance < 16) {
					toSend = generateMessage();
					toSend->setMessageType(JM_IN_POSITION);
					sendUnicast(toSend, positionHelper->getLeaderId());

					changeState(JS_WAIT_JOIN);
				}
			}
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId() && maneuver->getMessageType()!=LM_PROPOSE_AS_LEADER){
				if(maneuver->getMessageType() == LM_ABORT_MANEUVER){
					positionHelper->setPlatoonId(-1);
					positionHelper->setLeaderId(-1);
					positionHelper->setPlatoonLane(-1);
					if (role == JOINER){
						changeState(JS_WILL_TO_BE_FOLLOWER);
					}else{
						changeState(LJS_WILL_TO_BE_LEADER_OR_FOLLOWER);
					}
				}
			}
			timer++;
			if(timer == MAX_TIME &&  vehicleState == JS_MOVE_IN_POSITION){
				sendNegativeAck(positionHelper->getLeaderId(),JM_ABORT_MANEUER);
				positionHelper->setPlatoonId(-1);
				positionHelper->setLeaderId(-1);
				positionHelper->setPlatoonLane(-1);
				if (role == JOINER){
					changeState(JS_WILL_TO_BE_FOLLOWER);
				}else{
					changeState(LJS_WILL_TO_BE_LEADER_OR_FOLLOWER);
				}
			}
			break;
		}

		case JS_WAIT_JOIN: {
			std::cout<<"vehicle "<<positionHelper->getId()<<" wait join to "<<positionHelper->getLeaderId()<<endl;
			if (maneuver && maneuver->getPlatoonId() == positionHelper->getPlatoonId() && maneuver->getMessageType()!=LM_PROPOSE_AS_LEADER ) {

				//if we get confirmation from the leader, switch from faked CACC to real CACC
				if (maneuver->getMessageType() == LM_JOIN_PLATOON) {
					traciVehicle->setActiveController(Plexe::CACC);
					//set spacing to 5 meters to get close to the platoon
					traciVehicle->setCACCConstantSpacing(5);
					//tell the leader that we're now in the platoon
					toSend = generateMessage();
					toSend->setMessageType(JM_IN_PLATOON);
					sendUnicast(toSend, positionHelper->getLeaderId());
					changeState(JS_FOLLOW);
				}else if(maneuver->getMessageType() == LM_ABORT_MANEUVER){
					positionHelper->setPlatoonId(-1);
					positionHelper->setLeaderId(-1);
					positionHelper->setPlatoonLane(-1);
					if (role == JOINER){
						changeState(JS_WILL_TO_BE_FOLLOWER);
					}else{
						changeState(LJS_WILL_TO_BE_LEADER_OR_FOLLOWER);
					}
				}
			}
			break;
		}

		case JS_FOLLOW: {
			std::cout<<"vehicle "<<positionHelper->getId()<<" follows"<<endl;
			break;
		}

		case LJS_WILL_TO_BE_LEADER_OR_FOLLOWER:{
			std::cout<<"vehicle "<<positionHelper->getId()<<" leader of follower"<<endl;
			if (maneuver && maneuver->getMessageType() == JM_REQUEST_JOIN) {
				insertNewJoiner(maneuver->getVehicleId());

				positionHelper->setLeaderId(positionHelper->getId());
				positionHelper->setIsLeader(true);
				positionHelper->setPlatoonLane(vehicleData.actualLane);
				positionHelper->setPlatoonId(positionHelper->getId());

				changeState(LS_LEADING);
			}else if (maneuver && maneuver->getMessageType() == LM_PROPOSE_AS_LEADER && vehicleData.joiners.empty()) {
				Coord veinsPosition = mobility->getPositionAt(simTime());
				Veins::TraCICoord coords = mobility->getManager()->omnet2traci(veinsPosition);
				Coord position(coords.x, coords.y, 0);
				int myPosX = position.x;
				if ( maneuver->getPlatoonSpeed() >= vehicleData.minSpeed && maneuver->getPlatoonSpeed()<= vehicleData.maxSpeed && myPosX < maneuver->getPosX()){
					positionHelper->setPlatoonId(maneuver->getPlatoonId());
					positionHelper->setLeaderId(-1);
					positionHelper->setPlatoonLane(maneuver->getPlatoonLane());
					positionHelper->setFrontId(-1);
					positionHelper->setIsLeader(false);
					toSend = generateMessage();
					toSend->setMessageType(JM_REQUEST_JOIN);
					sendUnicast(toSend, positionHelper->getPlatoonId());

					changeState(JS_WAIT_REPLY);
				}
			}


			timer++;
			if (timer % (50+positionHelper->getId()) == 0){
				joinerDetectsInFrontVehicles();
				sendLeaderProposal();
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
	//TODO: check for double free corruption
	if (ctrl) {
		std::cout<<"vehicle "<<positionHelper->getId()<<" passed in handle lower control, type "<< ctrl->getControlCommand()<<endl;
		if(ctrl->getControlCommand() == SEND_FAIL){
			std::cout<<"vehicle "<<positionHelper->getId()<<" passed in handle lower control send fail"<<endl;
			if(role == LEADER){
				vehicleState = LS_LEADING;
				sendNegativeAck(vehicleData.joinerId, LM_ABORT_MANEUVER);

			}else if (role == JOINER){
				vehicleState = JS_WILL_TO_BE_FOLLOWER;
				sendNegativeAck(vehicleData.joinerId, JM_ABORT_MANEUER);
				positionHelper->setPlatoonId(-1);
				positionHelper->setLeaderId(-1);
				positionHelper->setPlatoonLane(-1);

			}else{
				if(vehicleState == LS_LEADING || vehicleState == LS_WAIT_JOINER_IN_POSITION || vehicleState == LS_WAIT_JOINER_TO_JOIN){
					vehicleState == LS_LEADING;
					sendNegativeAck(vehicleData.joinerId, LM_ABORT_MANEUVER);

				}else{
					vehicleState = LJS_WILL_TO_BE_LEADER_OR_FOLLOWER;
					sendNegativeAck(positionHelper->getPlatoonId(), JM_ABORT_MANEUER);
					positionHelper->setPlatoonId(-1);
					positionHelper->setLeaderId(-1);
					positionHelper->setPlatoonLane(-1);
				}
			}
		}
		delete ctrl;
	}
	else {
		delete msg;
	}
}
