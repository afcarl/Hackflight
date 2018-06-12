/*
   msp.hpp : MSP (Multiwii Serial Protocol) support

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

#include "receiver.hpp"
#include "mixer.hpp"
#include "datatypes.hpp"

namespace hf {

    class MSP {

        friend class Hackflight;

        private:

        // See http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
        static const uint8_t MSP_RC_NORMAL        =    121;
        static const uint8_t MSP_ATTITUDE_RADIANS =    122; 
        static const uint8_t MSP_SET_DEMANDS      =    225;    
        static const uint8_t MSP_SET_MOTOR_NORMAL =    215;    
        static const uint8_t MSP_SET_ARMED        =    216;    

		// XXX Ad-hoc, for Nengo altitude-hold project
        static const uint8_t MSP_ALTITUDE_TARGET  =    123; 

        static const int INBUF_SIZE  = 128;
        static const int OUTBUF_SIZE = 128;

        typedef enum serialState_t {
            IDLE,
            HEADER_START,
            HEADER_M,
            HEADER_ARROW,
            HEADER_SIZE,
            HEADER_CMD
        } serialState_t;

        // Passed to init() by Hackflight 
        vehicleState_t * _vehicleState;
        Receiver * _receiver;
        Mixer * _mixer;

        // Received from companion board
        demands_t _demands; 

        uint8_t _checksum;
        uint8_t _inBuf[INBUF_SIZE];
        uint8_t _inBufIndex;
        uint8_t _outBuf[OUTBUF_SIZE];
        uint8_t _outBufIndex;
        uint8_t _outBufSize;
        uint8_t _cmdMSP;
        uint8_t _offset;
        uint8_t _dataSize;

        serialState_t   _parserState;
        
        void serialize8(uint8_t a)
        {
            _outBuf[_outBufSize++] = a;
            _checksum ^= a;
        }

        void serialize16(int16_t a)
        {
            serialize8(a & 0xFF);
            serialize8((a >> 8) & 0xFF);
        }

        uint8_t read8(void)
        {
            return _inBuf[_inBufIndex++] & 0xff;
        }

        uint16_t read16(void)
        {
            uint16_t t = read8();
            t += (uint16_t)read8() << 8;
            return t;
        }

        uint32_t read32(void)
        {
            uint32_t t = read16();
            t += (uint32_t)read16() << 16;
            return t;
        }

        float readFloat(void)
        {
            float f = 0;
            uint32_t t = read32();
            memcpy(&f, &t, 4);
            return f;
        }

        void serializeFloats(float f[], uint8_t n)
        {
            _outBufSize = 0;
            _outBufIndex = 0;

            headSerialReply(4*n);

            for (uint8_t k=0; k<n; ++k) {
                uint32_t a;
                memcpy(&a, &f[k], 4);
                serialize32(a);
            }
        }

        void serialize32(uint32_t a)
        {
            serialize8(a & 0xFF);
            serialize8((a >> 8) & 0xFF);
            serialize8((a >> 16) & 0xFF);
            serialize8((a >> 24) & 0xFF);
        }


        void headSerialResponse(uint8_t err, uint8_t s)
        {
            serialize8('$');
            serialize8('M');
            serialize8(err ? '!' : '>');
            _checksum = 0;               // start calculating a new _checksum
            serialize8(s);
            serialize8(_cmdMSP);
        }

        void headSerialReply(uint8_t s)
        {
            headSerialResponse(0, s);
        }

        void headSerialError(uint8_t s)
        {
            headSerialResponse(1, s);
        }

        void tailSerialReply(void)
        {
            serialize8(_checksum);
        }

        void dispatchCommand(void)
        {
            switch (_cmdMSP) {

                case MSP_SET_MOTOR_NORMAL:
                    for (uint8_t i = 0; i < _mixer->nmotors; i++)
                        _mixer->motorsDisarmed[i] = readFloat();
                    headSerialReply(0);
                    break;

                case MSP_SET_ARMED:
                    if (read8()) {  // got arming command: arm only if throttle is down
                        if (_receiver->throttleIsDown()) {
                            _vehicleState->armed = true;
                        }
                    }
                    else {          // got disarming command: always disarm
                        _vehicleState->armed = false;
                    }
                    headSerialReply(0);
                    break;

                case MSP_SET_DEMANDS:
                    _demands.throttle = readFloat();
                    _demands.roll     = readFloat();
                    _demands.pitch    = readFloat();
                    _demands.yaw      = readFloat();
                    break;

                case MSP_RC_NORMAL:
                    serializeFloats(_receiver->rawvals, 8);
                    break;

                case MSP_ATTITUDE_RADIANS: 
                    serializeFloats(_vehicleState->eulerAngles, 3);
                    break;

				// XXX ad-hoc for Nengo
                case MSP_ALTITUDE_TARGET: 
                    {
						float vals[2];
						vals[0] = _vehicleState->altitude;
						vals[1] = _vehicleState->targetAltitude;
                        serializeFloats(vals, 2);
                    }
                    break;

                // don't know how to handle the (valid) message, indicate error
                default:                   
                    headSerialError(0);
                    break;
            }
        }

        protected:

        void init(vehicleState_t * vehicleState, Receiver * receiver, Mixer * mixer)
        {
            _vehicleState = vehicleState;
            _receiver = receiver;
            _mixer = mixer;

            _demands = {0, 0, 0, 0};

            _checksum = 0;
            _outBufIndex = 0;
            _outBufSize = 0;
            _cmdMSP = 0;
            _offset = 0;
            _dataSize = 0;
            _parserState = IDLE;
        }

        void update(uint8_t c)
        {
            switch (_parserState) {
                case IDLE:
                    _parserState = (c == '$') ? HEADER_START : IDLE;
                    break;
                case HEADER_START:
                    _parserState = (c == 'M') ? HEADER_M : IDLE;
                    break;
                case HEADER_M:
                    _parserState = (c == '<') ? HEADER_ARROW : IDLE;
                    break;
                case HEADER_ARROW:
                    if (c > INBUF_SIZE) {       // now we are expecting the payload size
                        _parserState = IDLE;
                        return;
                    }
                    _dataSize = c;
                    _offset = 0;
                    _checksum = 0;
                    _inBufIndex = 0;
                    _checksum ^= c;
                    _parserState = HEADER_SIZE;      // the command is to follow
                    break;
                case HEADER_SIZE:
                    _cmdMSP = c;
                    _checksum ^= c;
                    _parserState = HEADER_CMD;
                    break;
                case HEADER_CMD:
                    if (_offset < _dataSize) {
                        _checksum ^= c;
                        _inBuf[_offset++] = c;
                    } else  {
                        if (_checksum == c) {        // compare calculated and transferred _checksum
                            dispatchCommand();
                            tailSerialReply();
                        }
                        _parserState = IDLE;
                    }

            } // switch (_parserState)

        } // update

        // For use with companion boards, etc.
        void modifyDemands(demands_t & demands)
        {
            demands.throttle += _demands.throttle;
            demands.roll     += _demands.roll;
            demands.pitch    += _demands.pitch;
            demands.yaw      += _demands.yaw;
        }

        uint8_t availableBytes(void)
        {
            return _outBufSize;
        }

        uint8_t readByte(void)
        {
            _outBufSize--;
            return _outBuf[_outBufIndex++];
        }


    }; // class MSP


} // namespace
