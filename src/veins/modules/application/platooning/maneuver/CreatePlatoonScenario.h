#ifndef CREATEPLATOONSCENARIO_H_
#define CREATEPLATOONSCENARIO_H_

#include "veins/modules/application/platooning/scenarios/BaseScenario.h"
#include "veins/modules/application/platooning/protocols/BaseProtocol.h"

#include "veins/modules/application/platooning/utilities/NewPositionHelper.h"
#include "veins/modules/application/platooning/messages/ManeuverMessage_m.h"
#include <queue>

class CreatePlatoonScenario : public BaseScenario
	{

	protected:
		//define the roles
		enum JOIN_ROLE {LEADER, FOLLOWER, JOINER};
		//data that each car needs to keep
		struct VEHICLE_DATA {
			double              speed;      //speed of the platoon
			int                 joinLane;   //the lane chosen for joining the platoon
			int                 joinerId;   //the id of the vehicle joining the platoon
			std::vector<int>    formation;  //list of vehicles in the platoon
			std::queue<int>		joiners;	//queue of vehicles that want to join
		};
		//define the states for each role
		typedef enum _LEADER_STATES {
			LS_INIT = 0,
			LS_LEADING = 1,
			LS_WAIT_JOINER_IN_POSITION = 2,
			LS_WAIT_JOINER_TO_JOIN = 3
		} LEADER_STATES;
		typedef enum _JOINER_STATES {
			JS_INIT = 0,
			JS_IDLE = 1,
			JS_WAIT_REPLY = 2,
			JS_MOVE_IN_POSITION =3,
			JS_WAIT_JOIN = 4,
			JS_FOLLOW = 5
		} JOINER_STATES;
		typedef enum _FOLLOWER_STATES {
			FS_INIT = 0,
			FS_FOLLOW = 1
		} FOLLOWER_STATES;
		//define the messages that can be sent by each role
		enum LEADER_MSGS {
			LM_PROPOSE_AS_LEADER = 0,
			LM_MOVE_IN_POSITION = 1,
			LM_JOIN_PLATOON = 2,
			LM_UPDATE_FORMATION = 3
		};
		enum JOINER_MSGS {
			JM_REQUEST_JOIN = 4,
			JM_IN_POSITION = 5,
			JM_IN_PLATOON = 6
		};

		LEADER_STATES leaderState;
		FOLLOWER_STATES followerState;
		JOINER_STATES joinerState;
		//the role of this vehicle
		JOIN_ROLE role;
		//the position of this vehicle in the platoon
		int position;
		//data known by the vehicle
		struct VEHICLE_DATA vehicleData;
		//message used to start the maneuver
		cMessage *startManeuver;
		//pointer to protocol
		BaseProtocol *protocol;

		NewPositionHelper *newPositionHelper;

	public:

		static const int MANEUVER_TYPE = 12347;
		virtual void initialize(int stage);
		virtual void finish();

	protected:
		void sendUnicast(cPacket *msg, int destination);

	private:

	public:
		CreatePlatoonScenario()  {
			startManeuver = 0;
			positionHelper = 0;
		}

	protected:

		virtual void handleSelfMsg(cMessage *msg);
		//override this method of BaseApp. we want to handle it ourself
		virtual void handleLowerMsg(cMessage *msg);
		virtual void handleLowerControl(cMessage *msg);
		virtual void sendLeaderProposal();

		ManeuverMessage *generateMessage();

		void handleLeaderMsg(cMessage *msg);
		void handleJoinerMsg(cMessage *msg);
		void handleFollowerMsg(cMessage *msg);

		void prepareManeuverCars(int platoonLane);

	};

#endif
