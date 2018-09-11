/******      Protos      ******/
#include "protos.h"


FilledProtoBuf ProtoEncoder::encodeMsg(IoT_pairing_DataBlock const *dataBlock) {
	// Reset output stream (since stream is static, it would otherwise start writing after the last position it wrote to, so eventually the buffer would get full)
	pb_stream.bytes_written = 0;
	pb_stream.state = pb_buffer+pb_stream.bytes_written;

	// Encode the DataBlock
	bool status = pb_encode(&pb_stream, IoT_pairing_DataBlock_fields, dataBlock);

	// Then just check for any errors..
	if (!status) {
		printf("Encoding failed: %s\n", PB_GET_ERROR(&pb_stream));
		return {NULL, 0};
	} else {
		uint16_t bufLen = pb_stream.bytes_written;	// Compute the actual length of the pb
		bool printOut = false;

		printf("Successfully encoded DataBlock using %dB: t=%d", pb_stream.bytes_written, dataBlock->t_latest.seconds);
		printf(".%09d", dataBlock->t_latest.nanos);
		if (printOut) {
			printf(":\t[");
			for (int i=0; i<pb_stream.bytes_written; ++i) {
				printf(" %d,", pb_buffer[i]);
			}
			printf(" ]");
		}
		printf("\n");
	}

	return {pb_buffer, (uint16_t)pb_stream.bytes_written};
}

void ProtoEncoder::resetArrayCount(IoT_pairing_XYZArray & xyzArray) {
	xyzArray.x_count = 0;
	xyzArray.y_count = 0;
	xyzArray.z_count = 0;
}

void ProtoEncoder::resetArrayCount(IoT_pairing_QuatArray & quatArray) {
	quatArray.w_count = 0;
	quatArray.x_count = 0;
	quatArray.y_count = 0;
	quatArray.z_count = 0;
}

void ProtoEncoder::setCurrTime(IoT_pairing_DataBlock &dataBlock) {	// Helper method to set the current time as t_latest
	unsigned long t = micros();

	dataBlock.t_latest.seconds = t/1000000;
	dataBlock.t_latest.nanos = (t%1000000)*1000;
}
