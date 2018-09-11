#include "GPIO.h"

/******************************************/
/*                 Timer                  */
/******************************************/
Timer::Timer(ISRcb onTimerCb, uint16_t clkDiv, uint64_t timerCount, uint8_t timerNumber) : onTimerCb(onTimerCb), clkDiv(clkDiv), timerCount(timerCount) {
	timer = timerBegin(timerNumber, clkDiv, true);	// Use the available timer found, countUp=true
	timerAttachInterrupt(timer, onTimerCb, true);
	timerAlarmWrite(timer, timerCount, true);
}

void Timer::start() {
	timerAlarmEnable(timer);
}

void Timer::stop() {
	timerAlarmDisable(timer);
}

void Timer::remove() {
	timerEnd(timer);
	timer = NULL;
	onTimerCb = NULL;
}


/******************************************/
/*                 BNO055                 */
/******************************************/
namespace BNO055 {
	Timer timer = Timer(&onTimer, timerClkDiv, timerCount, timerNumber);
	volatile SemaphoreHandle_t semaphoreSampleSensor = xSemaphoreCreateBinary(), semaphoreSendSensorData = xSemaphoreCreateBinary();
	Adafruit_BNO055 bno = Adafruit_BNO055();
	IoT_pairing_DataBlock dataBlocks[2];
	bool currDataBlock = 0;

	void setup() {
		Wire.begin(13, 14, 400000);  // Setup SDA, SCL pins and set 400kHz I2C clock

		// Init sensor
		if (!bno.begin()) {
			consolePrintF("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
			while(1);
		} else {
			consolePrintF("Connected to the BNO055!\n");
		}

		// Wait for boot and init sensor
		delay(1000);
		bno.setExtCrystalUse(true);

		// Initialize proto buffers
		for (unsigned int i=0; i<2; ++i) {
			dataBlocks[i] = IoT_pairing_DataBlock_init_default;	// Initialize all fields in the block (orientation, linearAccel, etc.) to default
			dataBlocks[i].F_samp = Fsamp;
			dataBlocks[i].id = i;
		}

		// Create the sampling task (will be triggered every time the timer fires) and the sending task (once buffer gets full)
		xTaskCreatePinnedToCore(
        	taskSampleSensor,   /* Function to implement the task */
			"SampleSensorTask", /* Name of the task */
			8192,               /* Stack size in words */
			NULL,               /* Task input parameter */
			1,                  /* Priority of the task (higher number -> higher priority) */
			NULL,               /* Task handle. */
        	1);                 /* Core where the task should run (core 1, otherwise Wire seems to have trouble...) */
		xTaskCreatePinnedToCore(
        	taskSendSensorData,   /* Function to implement the task */
			"SendSensorDataTask", /* Name of the task */
			20000,                /* Stack size in words */
			NULL,                 /* Task input parameter */
			1,                    /* Priority of the task (higher number -> higher priority) */
			NULL,                 /* Task handle. */
        	0);                   /* Core where the task should run (core 0) */

		// Start sampling
		timer.start();
	}

	inline IRAM_ATTR void onTimer() {
		xSemaphoreGiveFromISR(semaphoreSampleSensor, NULL);	// Signal taskSampleSensor() that the timer has fired
	}

	void addReadingToDataArray(IoT_pairing_GenericDataArray & dataArray, imu::Vector<3> const & v) {
		dataArray.data_x[dataArray.data_x_count++] = v.x();
		dataArray.data_y[dataArray.data_y_count++] = v.y();
		dataArray.data_z[dataArray.data_z_count++] = v.z();
	}

	void addReadingToXYZArray(IoT_pairing_XYZArray & xyzArray, imu::Vector<3> const & v) {
		xyzArray.x[xyzArray.x_count++] = v.x();
		xyzArray.y[xyzArray.y_count++] = v.y();
		xyzArray.z[xyzArray.z_count++] = v.z();
	}

	void addReadingToQuatArray(IoT_pairing_QuatArray & quatArray, imu::Quaternion const & q) {
		quatArray.w[quatArray.w_count++] = q.w();
		quatArray.x[quatArray.x_count++] = q.x();
		quatArray.y[quatArray.y_count++] = q.y();
		quatArray.z[quatArray.z_count++] = q.z();
	}

	void setCalibrationStatus(IoT_pairing_CalibrationStatus & calibStatus) {
		uint8_t overall, gyro, accel, mag;
	    bno.getCalibration(&overall, &gyro, &accel, &mag);
		calibStatus.overall = overall;
		calibStatus.accel = accel;
		calibStatus.gyro = gyro;
		calibStatus.mag = mag;
	}

	void taskSampleSensor(void *pvParameters) {
		while (true) {
			// Block until timer fires
			if (xSemaphoreTake(semaphoreSampleSensor, 0) == pdTRUE) {
				// Collect a new sensor data sample
				addReadingToQuatArray(dataBlocks[currDataBlock].orientation, bno.getQuat());
				addReadingToXYZArray(dataBlocks[currDataBlock].linearAccel, bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL));
				addReadingToXYZArray(dataBlocks[currDataBlock].gravity, bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY));
				addReadingToXYZArray(dataBlocks[currDataBlock].accel, bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER));
				addReadingToXYZArray(dataBlocks[currDataBlock].gyro, bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE));
				addReadingToXYZArray(dataBlocks[currDataBlock].mag, bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER));

				// Check if we've reached the end of this buffer -> Mark it as ready to send and start writing to the other one
				if (dataBlocks[currDataBlock].orientation.x_count >= 10) {
					ProtoEncoder::setCurrTime(dataBlocks[currDataBlock]);	// Set t_latest to the current time
					setCalibrationStatus(dataBlocks[currDataBlock].calibStatus);
					currDataBlock = !currDataBlock;							// Switch to the other buffer (so we start filling it while the one we just filled gets sent)
					dataBlocks[currDataBlock].id += 2;						// Increase the id to the next unique number
					ProtoEncoder::resetArrayCount(dataBlocks[currDataBlock].orientation);	// Reset .x_count, .y_count, etc.
					ProtoEncoder::resetArrayCount(dataBlocks[currDataBlock].linearAccel);
					ProtoEncoder::resetArrayCount(dataBlocks[currDataBlock].gravity);
					ProtoEncoder::resetArrayCount(dataBlocks[currDataBlock].accel);
					ProtoEncoder::resetArrayCount(dataBlocks[currDataBlock].gyro);
					ProtoEncoder::resetArrayCount(dataBlocks[currDataBlock].mag);
					xSemaphoreGive(semaphoreSendSensorData);				// Notify taskSendSensorData() it's time to send the old buffer
				}
			}
		}
	}

	void taskSendSensorData(void *pvParameters) {
		ProtoEncoder protoEncoder;
		while (true) {
			// Block until buffer gets full and is ready to be sent
			if (xSemaphoreTake(semaphoreSendSensorData, 0) == pdTRUE) {
				FilledProtoBuf encodedMsg = protoEncoder.encodeMsg(&dataBlocks[!currDataBlock]);	// Encode the proto message
				if (USE_SERIAL_INSTEAD_OF_WIFI) {													// And send it
					Serial.write(encodedMsg.buf, encodedMsg.len);
				} else {
					wsSensorData.binaryAll((const char*)encodedMsg.buf, encodedMsg.len);
				}
			}
		}
	}
}
