#pragma once

#include "veins/veins.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/base/utils/Coord.h"
#include <algorithm>

using namespace omnetpp;

namespace veins {

class VEINS_API NormalAppLayer : public DemoBaseApplLayer {
    struct neighbor {
        simtime_t lastUpdate;
        Coord position;
        Coord speed;
        int repPositive;
        int repNegative;
    };

	public:
		virtual void initialize(int stage);
        virtual void finish();
	protected:
		simtime_t lastDroveAt;
		bool sentMessage;
		int currentSubscribedServiceId;

        int messageLength;

        double delayBSM_P1;
        double delayBSM_P2;

        simtime_t carAheadLastUpdate;

        std::vector<neighbor> neighborsList;
	protected:
		virtual void onBSM(DemoSafetyMessage* bsm);
        virtual void onWSM(BaseFrame1609_4* wsm);

        virtual void handleSelfMsg(cMessage* msg);

        virtual double getEuclideanDistance(Coord x1, Coord x2);
};

}
