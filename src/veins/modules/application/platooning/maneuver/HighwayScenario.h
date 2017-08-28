#ifndef HIGHWAYSCENARIO_H_
#define HIGHWAYSCENARIO_H_

#include "veins/modules/application/platooning/scenarios/BaseScenario.h"
#include "veins/modules/application/platooning/protocols/BaseProtocol.h"

#include "veins/modules/application/platooning/utilities/NewPositionHelper.h"
#include "veins/modules/application/platooning/messages/ManeuverMessage_m.h"
#include <queue>
#include <time.h>

class HighwayScenario : public BaseScenario
	{

	protected:
		//define the roles
		enum JOIN_ROLE {LEADER, JOINER, LEADER_OR_JOINER};
		//data that each car needs to keep
		struct VEHICLE_JOINER {
			int 				id;
			double				speed;
		};
		struct VEHICLE_DATA {
			double              speed;      //speed of the platoon
			double 				maxSpeed;	//maximum speed a machine wants to go
			double				minSpeed;	//minimum speed a machine wants to go
			int					actualLane;
			int                 joinLane;   //the lane chosen for joining the platoon
			int                 joinerId;   //the id of the vehicle joining the platoon
			std::queue<int>		joiners;	//queue of vehicles that want to join
			std::vector<int>	timeOutJoiners;
		};

		//define the states for each role
		typedef enum _VEHICLE_STATES {
			V_INIT = 0,
			V_IDLE = 1,
			LS_LEADING = 2,
			LS_WAIT_JOINER_IN_POSITION = 3,
			LS_WAIT_JOINER_TO_JOIN = 4,
			JS_WILL_TO_BE_FOLLOWER = 5,
			JS_WAIT_REPLY = 6,
			JS_MOVE_IN_POSITION = 7,
			JS_WAIT_JOIN = 8,
			JS_FOLLOW = 9,
			LJS_WILL_TO_BE_LEADER_OR_FOLLOWER = 10

		} VEHICLE_STATES;
		//define the messages that can be sent by each role
		enum VEHICLE_MSGS {
			LM_PROPOSE_AS_LEADER = 0,
			LM_REFUSE_JOIN = 1,
			LM_MOVE_IN_POSITION = 2,
			LM_JOIN_PLATOON = 3,
			JM_REQUEST_JOIN = 4,
			JM_IN_POSITION = 5,
			JM_IN_PLATOON = 6,
			LM_ABORT_MANEUVER = 7,
			JM_ABORT_MANEUER = 8
		};

		VEHICLE_STATES vehicleState;
		//the role of this vehicle
		JOIN_ROLE role;
		//the position of this vehicle in the platoon
		int position;
		int timer;
		const int MAX_TIME = 400;
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
		HighwayScenario()  {
			startManeuver = 0;
			positionHelper = 0;
		}

	protected:

		virtual void handleSelfMsg(cMessage *msg);
		//override this method of BaseApp. we want to handle it ourself
		virtual void handleLowerMsg(cMessage *msg);
		virtual void handleLowerControl(cMessage *msg);
		//sends leader proposal and information about its platoon
		virtual void sendLeaderProposal();
		virtual void sendNegativeAck(int destination, int type);
		//finds who is the next joiner comparing the queue of requests received with the vector of timeout received
		virtual void controlNextJoiner();
		//determines the role of a vehicle depending on the vehicle id
		virtual int determineRole ();
		//analyzes a joiner request. If the request can be accepted it will insert the joiner into the queue
		virtual void insertNewJoiner(int joinerId);
		virtual void leaderDetectsInFrontVehicles();
		virtual void joinerDetectsInFrontVehicles();
		virtual void changeState(VEHICLE_STATES state);

		ManeuverMessage *generateMessage();

		void handleVehicleMsg(cMessage *msg);

		void prepareManeuverCars();

	};

#endif
