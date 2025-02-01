#ifndef NormalAppLayer_H
#define NormalAppLayer_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/base/utils/Coord.h"
#include <algorithm>

namespace veins {

class NormalAppLayer : public BaseWaveApplLayer {
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
		virtual void onBSM(BasicSafetyMessage* bsm);
        virtual void onWSM(WaveShortMessage* wsm);

        virtual void handleSelfMsg(cMessage* msg);

        virtual double getEuclideanDistance(Coord x1, Coord x2);
};

} // namespace veins

#endif
