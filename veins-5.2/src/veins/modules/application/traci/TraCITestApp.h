//
// Copyright (C) 2006-2012 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef TraCITestApp_H
#define TraCITestApp_H

#include <set>
#include <list>

#include <omnetpp.h>

#include "veins/base/modules/BaseApplLayer.h"
#include "veins/base/utils/MiXiMDefs.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using veins::TraCIMobility;
using veins::TraCICommandInterface;

/**
 * FIXME
 */
namespace veins {

class TraCITestApp : public BaseApplLayer {
	public:
		int numInitStages() const { return std::max(BaseApplLayer::numInitStages(), 1); }
		void initialize(int stage);
		void finish();

	protected:
		static const simsignalwrap_t mobilityStateChangedSignal;

	protected:
		// module parameters
		bool debug;
		int testNumber;

		TraCIMobility* mobility;
		TraCICommandInterface* traci;
		TraCICommandInterface::Vehicle* traciVehicle;
		std::set<std::string> visitedEdges; /**< set of edges this vehicle visited */
		bool hasStopped; /**< true if at some point in time this vehicle traveled at negligible speed */

	protected:
		void handleSelfMsg(cMessage*);
		void handleLowerMsg(cMessage*);

		void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject* details);

		void handlePositionUpdate();
};

}  // namespace veins

#endif
