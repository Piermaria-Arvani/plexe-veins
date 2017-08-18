#ifndef CREATEPLATOONTRAFFICMANAGER_H_
#define CREATEPLATOONTRAFFICMANAGER_H_

#include <veins/modules/application/platooning/traffic/PlatoonsTrafficManager.h>

class CreatePlatoonTrafficManager : public PlatoonsTrafficManager
{

public:

	virtual void initialize(int stage);
	virtual void finish();

	CreatePlatoonTrafficManager() : PlatoonsTrafficManager() {
		insertJoinerMessage = 0;
	}

protected:

	cMessage *insertJoinerMessage;
	cMessage* message;

	void insertJoiner();

	virtual void handleSelfMsg(cMessage *msg);

};

#endif
