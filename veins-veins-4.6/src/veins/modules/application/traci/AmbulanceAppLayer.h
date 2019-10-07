#ifndef AmbulanceAppLayer_H
#define AmbulanceAppLayer_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

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
