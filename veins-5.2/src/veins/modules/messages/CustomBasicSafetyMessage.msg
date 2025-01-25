import veins.base.utils.Coord;
import veins.modules.messages.WaveShortMessage;

namespace veins;

packet CustomBasicSafetyMessage extends WaveShortMessage {
	Coord senderPos;
	Coord senderSpeed;
	int16_t eventIndicator;
	uint64_t validity;
	
	uint8_t certificate[126];
	uint8_t signature[56];
}
