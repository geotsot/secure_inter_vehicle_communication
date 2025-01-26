#ifndef AmbulanceAppLayer_H
#define AmbulanceAppLayer_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

using namespace omnetpp;

namespace veins {


class AmbulanceAppLayer : public BaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
	protected:
		simtime_t lastDroveAt;
		bool sentMessage;
		int currentSubscribedServiceId;
	protected:
        int messageLength;

        virtual void handleSelfMsg(cMessage* msg);
};

#endif

} // namespace veins
