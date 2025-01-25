import veins.base.utils.Coord;

namespace veins;

packet CustomBasicSafetyMessagePart2 {
	//Version of the Wave Short Message
	uint8_t wsmVersion = 0;
	//Determine which security mechanism was used
	uint8_t securityType = 0;
	//Channel Number on which this packet was sent
	int8_t channelNumber;
	//Data rate with which this packet was sent
	int8_t dataRate = 1;

	int8_t txPwrLevel;
	//Unique number to identify the service
	int32_t psid = 0;
	//Provider Service Context
	int64_t psc;
	//Length of Wave Short Message
	uint16_t wsmLength;
	//Data of Wave Short Message

	// BSM
	uint8_t msgID = 2;
	uint8_t msgCount;
	uint32_t temporaryID;
	int16_t secMark;

	//int32_t latitude
	//int32_t longitude
	//int16_t elevation
	Coord position;
	int32_t positionalAccuracy;

	//int16_t speed
	Coord speed;
	int16_t heading;
	int64_t acceleration;

	int16_t brakes;

	uint8_t sizeW;
	uint8_t sizeL;
	uint8_t sizeH;


	// BSM pt. 2
	int16_t eventFlag;


	// additional data
	simtime_t timestamp = 0;
	uint64_t validity;


	// security overead
	uint8_t cert[126];
	uint8_t signature[56];
}
