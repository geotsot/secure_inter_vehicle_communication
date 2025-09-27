#pragma once

#include "veins/veins.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/messages/CustomBasicSafetyMessage_m.h"

using namespace omnetpp;

namespace veins {

class VEINS_API AttackerAppLayer : public DemoBaseApplLayer {
	public:
		virtual void initialize(int stage);
	protected:
		simtime_t lastDroveAt;
		bool sentMessage;
		int currentSubscribedServiceId;
		std::vector<CustomBasicSafetyMessage*> msgStack;
	protected:
        virtual void onWSM(BaseFrame1609_4* wsm);

        virtual void handleSelfMsg(cMessage* msg);
};

}
