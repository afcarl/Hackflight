/*
   hackflight.hpp : general header, plus init and update methods

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cmath>

#include "board.hpp"
#include "msp.hpp"
#include "mixer.hpp"
#include "receiver.hpp"
#include "stabilizer.hpp"
#include "debug.hpp"
#include "datatypes.hpp"

namespace hf {

    class Hackflight {

        private: 

            // Passed to Hackflight::init() for a particular board and receiver
            Board      * _board;
            Receiver   * _receiver;
            Stabilizer * _stabilizer;
            Mixer      * _mixer;

            // MSP (serial comms)
            MSP _msp;

            // Vehicle state
            vehicleState_t _state;

            // Auxiliary switch state for change detection
            uint8_t _auxState;

            // Safety
            bool _failsafe;

            // Support for headless mode
            float _yawInitial;

            bool safeAngle(uint8_t axis)
            {
                return fabs(_state.eulerAngles[axis]) < _stabilizer->maxArmingAngle;
            }

            void checkQuaternion(void)
            {
                float q[4];

                if (_board->getQuaternion(q)) {

					// Compute Euler angles from quaternion
                    _state.eulerAngles[0] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
                    _state.eulerAngles[1] = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
                    _state.eulerAngles[2] = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]); 

                    // Convert heading from [-pi,+pi] to [0,2*pi]
                    if (_state.eulerAngles[AXIS_YAW] < 0) {
                        _state.eulerAngles[AXIS_YAW] += 2*M_PI;
                    }

                    // Update stabilizer with new Euler angles
                    _stabilizer->updateEulerAngles(_state.eulerAngles);

                    // Synch serial comms to quaternion check
                    doSerialComms();
                }
            }

            void checkGyroRates(void)
            {
                float gyroRates[3];

                if (_board->getGyrometer(gyroRates)) {

                    demands_t demands;

                    // Start with demands from receiver
                    memcpy(&demands, &_receiver->demands, sizeof(demands_t));

                    // Run stabilization to get updated demands
                    _stabilizer->modifyDemands(gyroRates, demands);

                    // Add in any corrections sent over MSP (companion board, etc.)
                    _msp.modifyDemands(demands);

                    // Sync failsafe to gyro loop
                    checkFailsafe();

                    // Use updated demands to run motors
                    if (_state.armed && !_failsafe && !_receiver->throttleIsDown()) {
                        Debug::printf("%2.2f %+2.2f %+2.2f %+2.2f", 
                                demands.throttle, demands.roll, demands.pitch, demands.yaw);
                        _mixer->runArmed(demands);
                    }
                    else {
                        Debug::printf("NOT FLYING");
                    }
                }
            }

			// a hack to work with the simulator's ground-truth values
            void checkGroundTruth(void)
            {
                if (_board->getGroundTruth(_state)) {
                }
            }

            void checkFailsafe(void)
            {
                if (_state.armed && _receiver->lostSignal()) {
                    _mixer->cutMotors();
                    _state.armed = false;
                    _failsafe = true;
                    _board->showArmedStatus(false);
                }
            } 

            void checkReceiver(void)
            {
                // Acquire receiver demands, passing yaw angle for headless mode
                if (!_receiver->getDemands(_state.eulerAngles[AXIS_YAW] - _yawInitial)) return;

                // Update stabilizer with cyclic demands
                _stabilizer->updateDemands(_receiver->demands);

                // When landed, reset integral component of PID
                if (_receiver->throttleIsDown()) {
                    _stabilizer->resetIntegral();
                }

                // Disarm
                if (_state.armed && _receiver->disarming()) {
                    _state.armed = false;
                } 

                // Arm (after lots of safety checks!)
                if (!_state.armed && _receiver->arming() && !_auxState && !_failsafe && safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH)) {
                    _state.armed = true;
                    _yawInitial = _state.eulerAngles[AXIS_YAW]; // grab yaw for headless mode
                }

                // Detect aux switch changes for altitude-hold, loiter, etc.
                if (_receiver->demands.aux != _auxState) {
                    _auxState = _receiver->demands.aux;
                    handleAuxSwitch(_receiver->demands);
                }

                // Cut motors on throttle-down
                if (_state.armed && _receiver->throttleIsDown()) {
                    _mixer->cutMotors();
                }

                // Set LED based on arming status
                _board->showArmedStatus(_state.armed);

            } // checkReceiver

            void doSerialComms(void)
            {
                while (_board->serialAvailableBytes() > 0) {
                    _msp.update(_board->serialReadByte());
                }

                while (_msp.availableBytes() > 0) {
                    _board->serialWriteByte(_msp.readByte());
                }

                // Support motor testing from GCS
                if (!_state.armed) {
                    _mixer->runDisarmed();
                }
            }

			// XXX ad-hoc for Nengo
            void handleAuxSwitch(demands_t & demands)
            {
                // Start
                if (demands.aux > 0) {
                    _state.holdingAltitude = true;
                    _state.initialThrottleHold = demands.throttle;
                    _state.targetAltitude = _state.altitude;
                }

                // Stop
                else {
                    _state.holdingAltitude = false;
                }
            }

        public:

            void init(Board * board, Receiver * receiver, Stabilizer * stabilizer, Mixer * mixer)
            {  
                // Store the essentials
                _board      = board;
                _receiver   = receiver;
                _stabilizer = stabilizer;
                _mixer      = mixer;

                // Initialize MSP (serial comms)
                _msp.init(&_state, receiver, mixer);

                // Initialize the receiver
                _receiver->init();

                // Tell the mixer which board to use
                _mixer->board = board; 
				
                // Start unarmed
                _state.armed = false;
                _failsafe = false;

				// XXX Set altitude-hold state for ad-hoc Nengo project
                _state.initialThrottleHold = 0;
                _state.holdingAltitude = false;

             } // init

            void update(void)
            {
                checkGyroRates();
                checkQuaternion();
                checkReceiver();
                checkGroundTruth(); // a hack to work with the simulator
            } 

    }; // class Hackflight

} // namespace
