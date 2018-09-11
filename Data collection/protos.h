/******      Protos      ******/
#ifndef PROTOS_H_
#define PROTOS_H_

#include "main.h"
#include <pb_encode.h>
#include <IoTpairingProtos.h>


typedef struct DataBlock {
	IoT_pairing_GenericDataArray orientation;
	IoT_pairing_GenericDataArray linearAccel;
	IoT_pairing_GenericDataArray gravity;
	IoT_pairing_GenericDataArray accel;
	IoT_pairing_GenericDataArray gyro;
	IoT_pairing_GenericDataArray mag;
	IoT_pairing_GenericDataArray *fields[6] = {&orientation, &linearAccel, &gravity, &accel, &gyro, &mag};
} DataBlock;

typedef struct FilledProtoBuf {
	const uint8_t *buf;
	uint16_t len;
} FilledProtoBuf;

class ProtoEncoder {
	public:
		ProtoEncoder() : pb_stream(pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer))) {}
		FilledProtoBuf encodeMsg(IoT_pairing_DataBlock const *dataBlock);

		static void resetArrayCount(IoT_pairing_XYZArray & xyzArray);
		static void resetArrayCount(IoT_pairing_QuatArray & quatArray);
		static void setCurrTime(IoT_pairing_DataBlock &dataBlock);	// Helper method to set the current time as t_latest

	protected:
		uint8_t pb_buffer[IoT_pairing_DataBlock_size];
		pb_ostream_t pb_stream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
};

#endif
