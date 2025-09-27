#pragma once

#include "veins/veins.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

using namespace omnetpp;

namespace veins {

class VEINS_API AmbulanceAppLayer_Secure : public DemoBaseApplLayer {
	public:
		virtual void initialize(int stage);
	protected:
		simtime_t lastDroveAt;
		bool sentMessage;
		int currentSubscribedServiceId;
	protected:
		int messageLength;
		int messageValidity;

        double signDelay;
        double verifyDelay;

        virtual void handleSelfMsg(cMessage* msg);
};

}
