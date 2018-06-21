/*
   butterfly.hpp : Implementation of Hackflight Board routines for Butterfly
                   dev board + MPU9250 IMU + brushless motors

   Additional library required: https://github.com/simondlevy/KrisWinerMPU9250

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <Wire.h>
#include <Servo.h>

#include <MPU9250.h> 
#include <ArduinoTransfer.h>
#include <MahonyAHRS.h>

#include "hackflight.hpp"
#include "realboard.hpp"

namespace hf {

    // Interrupt support 
    static bool gotNewData;
    static void interruptHandler()
    {
        gotNewData = true;
    }

    class Butterfly : public RealBoard {

        private:

            // Motor pins
            const uint8_t MOTOR_PINS[4] = {3, 4, 5, 6};

            // Min, max PWM values
            const uint16_t PWM_MIN = 990;
            const uint16_t PWM_MAX = 2000;

            // Butterfly board follows Arduino standard for LED pin
            const uint8_t LED_PIN = 13;

            // MPU9250 add-on board has interrupt on Butterfly pin 8
            const uint8_t INTERRUPT_PIN = 8;

            // Create byte-transfer objects for Arduino I^2C 
            ArduinoI2C mpu = ArduinoI2C(MPU9250::MPU9250_ADDRESS);

            // Use the MPU9250 in master mode
            MPU9250Master _imu = MPU9250Master(&mpu);;

            // Run motor ESCs using standard Servo library
            Servo _escs[4];

            // Paramters to experiment with ------------------------------------------------------------------------

            // Sensor full-scale settings
            const Ascale_t ASCALE = AFS_8G;
            const Gscale_t GSCALE = GFS_2000DPS;

            // SAMPLE_RATE_DIVISOR: (1 + SAMPLE_RATE_DIVISOR) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
            // SAMPLE_RATE_DIVISOR = 0 means 1 kHz sample rate for both accel and gyro, 4 means 200 Hz, etc.
            const uint8_t SAMPLE_RATE_DIVISOR = 4;         

            // Instance variables -----------------------------------------------------------------------------------

            // For scaling to normal units (accelerometer G's, gyrometer rad/sec
            float _aRes;
            float _gRes;

            // Used to read all 14 bytes at once from the MPU9250 accel/gyro
            int16_t _imuData[7] = {0,0,0,0,0,0,0};

            // Quaternion support
            Mahony filter;
            const float QUATERNION_HZ = 200.f;
            const float QUATERNION_INPUT_SCALE = 50.f;
            const uint32_t MICROS_PER_READING = 1000000 / QUATERNION_HZ;
            unsigned long  _microsPrevious;

            // We compute these at startup
            float _gyroBias[3]        = {0,0,0};
            float _accelBias[3]       = {0,0,0};

            // Accel, gyro readings are computed in getGyrometer() and used in getEulerAngles()
            float _ax, _ay, _az, _gx, _gy, _gz;

            // Helpers -----------------------------------------------------------------------------------

            // Raw analog-to-digital values converted to radians per second
            float adc2rad(int16_t adc) 
            {
                return (adc * _gRes) * M_PI / 180;
            }

        protected:

            void delayMilliseconds(uint32_t msec)
            {
                delay(msec);
            }

            void ledSet(bool is_on)
            { 
                digitalWrite(LED_PIN, is_on ? LOW : HIGH);
            }

            uint8_t serialAvailableBytes(void)
            {
                return Serial.available();
            }

            uint8_t serialReadByte(void)
            {
                return Serial.read();
            }

            void serialWriteByte(uint8_t c)
            {
                Serial.write(c);
            }

            void writeMotor(uint8_t index, float value)
            {
                _escs[index].writeMicroseconds((uint16_t)(PWM_MIN+value*(PWM_MAX-PWM_MIN)));
            }

            bool getGyrometer(float gyro[3])
            {
                if (gotNewData) {

                    gotNewData = false;

                    if (_imu.checkNewData()) {

                        _imu.readMPU9250Data(_imuData); 

                        // Convert the accleration value into g's
                        _ax = _imuData[0]*_aRes - _accelBias[0];  // get actual g value, this depends on scale being set
                        _ay = _imuData[1]*_aRes - _accelBias[1];   
                        _az = _imuData[2]*_aRes - _accelBias[2];  

                        // Convert the gyro value into degrees per second
                        _gx = adc2rad(_imuData[4]);
                        _gy = adc2rad(_imuData[5]);
                        _gz = adc2rad(_imuData[6]);

                        // Copy gyro values back out
                        gyro[0] = _gx;
                        gyro[1] = _gy;
                        gyro[2] = _gz;

                        return true;

                    } // if (_imu.checkNewAccelGyroData())

                } // if gotNewData

                return false;
            }

            bool getEulerAngles(float eulerAngles[3])
            {
                uint32_t microsCurrent = micros();
                if (microsCurrent - _microsPrevious >= MICROS_PER_READING) {

                    // update the quaternion filter
                    filter.updateIMU(QUATERNION_INPUT_SCALE*_gx, QUATERNION_INPUT_SCALE*_gy, QUATERNION_INPUT_SCALE*_gz, QUATERNION_INPUT_SCALE*_ax, QUATERNION_INPUT_SCALE*_ay, QUATERNION_INPUT_SCALE*_az);

                    eulerAngles[0] = filter.getRollRadians();
                    eulerAngles[1] = filter.getPitchRadians();
                    eulerAngles[2] = filter.getYawRadians();

                    _microsPrevious = microsCurrent;

                    return true;

                } 

                return false;
            }

            bool getAccelerometer(float accelGs[3])
            {
                (void)accelGs;
                return false;
            }

            bool getBarometer(float & pressure)
            {
                (void)pressure;
                return false;
            }

        public:

            Butterfly(void)
            {
                // Begin serial comms
                Serial.begin(115200);

                // Setup LED pin and turn it off
                pinMode(LED_PIN, OUTPUT);
                digitalWrite(LED_PIN, HIGH);

                // Set up the interrupt pin, it's set as active high, push-pull
                pinMode(INTERRUPT_PIN, INPUT);
                attachInterrupt(INTERRUPT_PIN, interruptHandler, RISING);  

                // Connect to the ESCs and send them the baseline values
                for (uint8_t k=0; k<4; ++k) {
                    _escs[k].attach(MOTOR_PINS[k]);
                    _escs[k].writeMicroseconds(PWM_MIN);
                }

                // Start I^2C
                Wire.begin();
                Wire.setClock(400000); // I2C frequency at 400 kHz
                delay(1000);

                // Reset the MPU9250
                _imu.resetMPU9250(); 

                // Start the quaternion filter
                filter.begin(QUATERNION_HZ);

                // get sensor resolutions, only need to do this once
                _aRes = _imu.getAres(ASCALE);
                _gRes = _imu.getGres(GSCALE);

                // Calibrate gyro and accelerometers, load biases in bias registers
                _imu.calibrateMPU9250(_gyroBias, _accelBias); 

                // Initialize the MPU9250
                _imu.initMPU9250(ASCALE, GSCALE, SAMPLE_RATE_DIVISOR); 

                // Do general real-board initialization
                RealBoard::init();

                // Reset timer for quaternion update
                _microsPrevious = 0;
            }


    }; // class Butterfly

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
