#ifndef GPIO_H_
#define GPIO_H_

#include "main.h"
#include "protos.h"
#include <Wire.h>				// I2C library (to configure custom SDA, SCL pins)
#include <Adafruit_BNO055.h>	// BNO055 (orientation sensor) driver (get/set calibration, get sensor data, etc.)


/******************************************/
/*                 Timer                  */
/******************************************/
class Timer {
public:
	typedef void (*ISRcb)(void);		// ISR expects a `void *(void)` callback

	Timer(ISRcb onTimerCb, uint16_t clkDiv, uint64_t timerCount, uint8_t timerNumber);
	~Timer() { remove(); }

	void start();
	void stop();
	void remove();

private:
	hw_timer_t *timer;
	ISRcb onTimerCb;
	uint16_t clkDiv;
	uint64_t timerCount;
};


/******************************************/
/*                 BNO055                 */
/******************************************/
namespace BNO055 {
	constexpr uint16_t timerClkDiv = 2;
	constexpr float timerBaseFreq = 80000000/timerClkDiv;	// Timer freq is 80MHz with a minimum clock divider of 2, so the actual base frequency is 40MHz
	constexpr float Fsamp = 50;	// Sample at 100Hz (fastest the sensor allows in fusion mode)
	constexpr uint64_t timerCount = round(timerBaseFreq/Fsamp);
	constexpr uint8_t timerNumber = 0;	// Use Timer0 for sampling the BNO055
	extern Timer timer;
	extern volatile SemaphoreHandle_t semaphoreSampleSensor, semaphoreSendSensorData;
	extern Adafruit_BNO055 bno;
	extern IoT_pairing_DataBlock dataBlocks[2];	// Double-buffered (so we can keep filling while we send the other one)
	extern bool currDataBlock;	// The index in dataBlocks we're writing to ([0] or [1])

	void setup();
	inline IRAM_ATTR void onTimer();
	void addReadingToDataArray(IoT_pairing_GenericDataArray & dataArray, imu::Vector<3> const & v);
	void addReadingToXYZArray(IoT_pairing_XYZArray & xyzArray, imu::Vector<3> const & v);
	void addReadingToQuatArray(IoT_pairing_QuatArray & quatArray, imu::Quaternion const & q);
	void setCalibrationStatus(IoT_pairing_CalibrationStatus & calibStatus);
	void taskSampleSensor(void *pvParameters);
	void taskSendSensorData(void *pvParameters);
}

#endif
