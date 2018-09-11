import struct
import serial
import os
from datetime import datetime
from log_helper import logger
from protos.python import sensor_data_pb2 as SensorData
from google.protobuf.timestamp_pb2 import Timestamp


def main(serial_port='/dev/cu.SLAB_USBtoUART', baud_rate=115200, out_base_folder='data'):
    logger.info("Connecting to {}...".format(serial_port))
    s = serial.Serial(serial_port, baud_rate)
    logger.success("Successfully connected! Please reboot the ESP32")

    try:
        # Read serial until we receive the " -- SETUP COMPLETE -- " message
        while True:
            l = s.readline().rstrip()  # Read a line (removing any trailing \r\n)
            logger.debug(l)
            if l == b'-- SETUP COMPLETE --':
                logger.notice("Esp32 is done booting, collecting data until Ctrl+C is pressed!")
                break

        # Create folder for
        out_folder = os.path.join(out_base_folder, str(datetime.now())[:-7].replace(':', '-'))
        os.makedirs(out_folder)

        # Collect data (read samples and write them to files)
        while True:
            pb_len = struct.unpack('<H', s.read(2))[0]               # Read 2 bytes that indicate how long the proto message will be and convert to uint16
            pb = s.read(pb_len)                                      # Read the proto contents
            data_block = SensorData.DataBlock().FromString(pb)  # Decode them
            logger.info("Message received: {} (t={:.6f})".format(data_block.id, data_block.t_latest.seconds + data_block.t_latest.nanos/1e9))


    except KeyboardInterrupt:
        logger.notice("Stopping data collection!")
    finally:
        s.close()

    # Cleaup, etc.
    logger.success("Goodbye!")


if __name__ == '__main__':
    main()
