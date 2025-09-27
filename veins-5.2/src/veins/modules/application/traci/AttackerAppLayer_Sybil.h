#pragma once

#include "veins/veins.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

using namespace omnetpp;

namespace veins {

class VEINS_API AttackerAppLayer_Sybil : public DemoBaseApplLayer {
	public:
		virtual void initialize(int stage);
	protected:
		simtime_t lastDroveAt;
		bool sentMessage;
		int currentSubscribedServiceId;
		std::vector<DemoSafetyMessage*> msgStack;
		std::vector<DemoSafetyMessage*> lastReportedPosition;
	protected:
        virtual void onBSM(DemoSafetyMessage* bsm);
};

}
