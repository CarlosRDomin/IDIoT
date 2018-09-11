import serial
import os
from datetime import datetime
from log_helper import logger


def main(serial_port='/dev/cu.SLAB_USBtoUART', baud_rate=115200, out_base_folder='data'):
    logger.info("Connecting to {}...".format(serial_port))
    s = serial.Serial(serial_port, baud_rate)
    logger.success("Successfully connected! Waiting for ESP32 to boot")
    print
    try:
        # Read serial until we receive the " -- SETUP COMPLETE -- " message
        while True:
            l = s.readline().rstrip('\n')
            logger.debug(l)
            if l == ' -- SETUP COMPLETE -- ':
                logger.notice("Esp32 is done booting, collecting data until Ctrl+C is pressed!")
                break

        # Create folder for
        out_folder = os.path.join(out_base_folder, str(datetime.now())[:-7].replace(':', '-'))
        os.makedirs(out_folder)

        # Collect data (read samples and write them to files)
        while True:
            pb_len = s.read(2)  # Read 2 bytes that indicate how long the proto message will be

    except KeyboardInterrupt:
        print("Stopping data collection!")
    finally:
        s.close()

    # Cleaup, etc.
    pass


if __name__ == '__main__':
    main()
