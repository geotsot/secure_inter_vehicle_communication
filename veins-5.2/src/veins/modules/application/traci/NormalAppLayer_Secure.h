#ifndef NormalAppLayer_Secure_H
#define NormalAppLayer_Secure_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/base/utils/Coord.h"
#include <algorithm>

namespace veins {

class NormalAppLayer_Secure : public BaseWaveApplLayer {
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

        std::vector<neighbor> neighborsList;
	protected:
		virtual void onBSM(BasicSafetyMessage* bsm);
        virtual void onWSM(WaveShortMessage* wsm);

        virtual void handleSelfMsg(cMessage* msg);

        virtual double getEuclideanDistance(Coord x1, Coord x2);

        double delayBSM_P1_transmission;
        double delayBSM_P2_transmission;
        double delayBSM_P1_end_to_end;
        double delayBSM_P2_end_to_end;

        double signDelay;
        double verifyDelay;
};


} // namespace veins

#endif
