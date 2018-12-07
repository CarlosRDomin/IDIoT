import struct
import serial
import glob
import numpy as np
import h5py
import os
from datetime import datetime
from log_helper import logger
from protos.python import sensor_data_pb2 as SensorData
from record_cam import record_cam
from multiprocessing import Process, Queue


def init_fields_to_save():
    # Create a dictionary with the fields (eg: orientation) we're going to save
    fields_to_save = {}
    for field in SensorData.DataBlock.DESCRIPTOR.fields:
        if field.message_type is None: continue  # Not interested in native types (float, int, etc. because those are Fsamp, id, etc.)
        if field.message_type.name in ('QuatArray', 'XYZArray', 'CalibrationStatus'):  # Fields we're interested in: orientation, linearAccel, calibStatus, etc.
            fields_to_save[field.name] = {}  # Create an entry in the dictionary with this field's name
            for subfield in field.message_type.fields:
                fields_to_save[field.name][subfield.name] = []  # Initialize an empty list (which will hold the data) with the subfield's name (eg: 'x')

    return fields_to_save


def save_data_to_file(fields_to_save, out_base_folder, experiment_t_start, filename_prefix, imu_t_start, F_samp):
    if F_samp <= 0:  # F_samp=0 means data was never collected -> Don't save an empty file!
        logger.warn("Nothing to save, skipping saving data to file")
        return

    out_base_folder = os.path.join(out_base_folder, str(experiment_t_start)[:-7].replace(':', '-'))
    os.makedirs(out_base_folder, exist_ok=True)  # Make sure base folder exists
    out_filename = os.path.join(out_base_folder, "{}{}.h5".format(filename_prefix, str(experiment_t_start)[:-7].replace(':', '-')))
    logger.info("Saving collected data at '{}'...".format(out_filename))
    with h5py.File(out_filename) as hf:
        hf.attrs['t_start'] = str(imu_t_start)
        hf.attrs['F_samp'] = F_samp
        for field_name, field_children in fields_to_save.items():
            field_group = hf.create_group(field_name)
            for axis, axis_values in field_children.items():
                field_group.create_dataset(axis, data=np.array(axis_values, dtype=np.uint8 if isinstance(axis_values[0], int) else np.float32))
    logger.success("Done! Collected data saved at '{}' :)".format(out_filename))


def collect_BNO055_data(experiment_t_start=None, serial_port='/dev/cu.SLAB_USBtoUART', baud_rate=115200, out_base_folder='data', out_filename_prefix='BNO055_', imu_ready_queue=None):
    # NOTE: In order to allow for multiple simultaneous IMUs, experiment_t_start indicates the name of the enclosing folder where the data will be saved,
    # and each one will have their own imu_t_start indicating the precise time when each IMU was rebooted (and therefore its data started logging)

    logger.info("Connecting to {}...".format(serial_port))
    s = serial.Serial(serial_port, baud_rate)
    logger.success("Successfully connected! Please reboot the ESP32")

    # Initialize variables
    fields_to_save = init_fields_to_save()
    data_block = SensorData.DataBlock()
    imu_t_start = None
    F_samp = 0

    try:
        # Read serial until we receive the " -- SETUP COMPLETE -- " message
        while True:
            l = s.readline().rstrip()  # Read a line (removing any trailing \r\n)
            logger.debug(l)
            if l == b'-- SETUP COMPLETE --':
                logger.notice("Esp32 is done booting, collecting data until Ctrl+C is pressed!")
                break

        # Update imu_t_start to the actual time the data collection started
        imu_t_start = datetime.now()
        if experiment_t_start is None:
            experiment_t_start = imu_t_start  # If user didn't specify an experiment t_start, it's probably only collecting data for 1 IMU -> Use imu_t_start to name the data folder
        if imu_ready_queue is not None:  # Pass the current time through the queue to notify the main process to start recording the camera
            imu_ready_queue.put(imu_t_start)

        # Collect data (read samples and append them to fields_to_save)
        last_msg_id = None
        while True:
            pb_len = struct.unpack('<H', s.read(2))[0]               # Read 2 bytes that indicate how long the proto message will be and convert to uint16
            pb = s.read(pb_len)                                      # Read the proto contents
            data_block.ParseFromString(pb)  # Decode them
            logger.info("Message received: {} (t={:.6f})".format(data_block.id, data_block.t_latest.seconds + data_block.t_latest.nanos/1e9))

            # Check if any data got lost (hopefully not :D)
            if last_msg_id is None:
                F_samp = data_block.F_samp  # Store the sampling frequency so we can save it in the file
            elif data_block.id - last_msg_id > 1:
                n_lost = data_block.id - last_msg_id - 1
                logger.critical("Lost {} packet{}! (Previous message ID was {}, current is {})".format(n_lost, 's' if n_lost>1 else '', data_block.id, last_msg_id))
            last_msg_id = data_block.id

            # Append new data to fields_to_save
            for field_name, field_children in fields_to_save.items():
                field_children_values = getattr(data_block, field_name)
                for axis, axis_values in field_children.items():
                    axis_new_values = getattr(field_children_values, axis)  # Fetch new sensor readings
                    axis_values.append(axis_new_values) if isinstance(axis_new_values, int) else axis_values.extend(axis_new_values)  # Append values

    except KeyboardInterrupt:
        logger.notice("Stopping data collection!")
    finally:
        # Close serial port and save all data to a file
        s.close()
        save_data_to_file(fields_to_save, out_base_folder, experiment_t_start, out_filename_prefix, imu_t_start, F_samp)

    logger.success("Goodbye from BNO055 data collection!")


def main():
    T_START = datetime.now()
    IMU_SERIAL_PORTS = glob.glob('/dev/cu.SLAB_USBtoUART*')  # ['/dev/cu.SLAB_USBtoUART', '/dev/cu.SLAB_USBtoUART9']
    imu_ready_queue = Queue(len(IMU_SERIAL_PORTS))  # Use a queue (multi-process safe) to communicate when each process is ready to collect data (esp32 was restarted)

    # Start one process per IMU (each BNO055 is connected to its own esp32)
    p = []
    for i,port in enumerate(IMU_SERIAL_PORTS):
        p.append(Process(target=collect_BNO055_data, kwargs={"experiment_t_start": T_START, "serial_port": port, "out_filename_prefix": 'BNO055_{}_'.format(i+1), "imu_ready_queue": imu_ready_queue}))
        p[-1].start()

    # BLOCK until all esp32s are rebooted and ready to collect data
    for i in range(len(p)):
        imu_t_start = imu_ready_queue.get()

    # Start recording video
    record_cam(1, t_start=T_START)

    # When record_cam exits (KeyboardInterrupt), terminate all IMU processes and exit
    for pp in p:
        pp.terminate()  # Send a KeyboardInterrupt to each process running collect_BNO055_data
    for pp in p:
        pp.join()  # Wait for all processes to terminate
    logger.success("Experiment done, bye!!")


if __name__ == '__main__':
    main()
