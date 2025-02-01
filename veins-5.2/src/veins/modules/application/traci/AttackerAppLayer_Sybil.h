#ifndef AttackerAppLayer_Sybil_H
#define AttackerAppLayer_Sybil_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

namespace veins {

class AttackerAppLayer_Sybil : public BaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
	protected:
		simtime_t lastDroveAt;
		bool sentMessage;
		int currentSubscribedServiceId;
		std::vector<BasicSafetyMessage*> msgStack;
		std::vector<BasicSafetyMessage*> lastReportedPosition;
	protected:
        virtual void onBSM(BasicSafetyMessage* bsm);
};

} // namespace veins

#endif
