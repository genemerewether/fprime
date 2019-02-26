/*
 * ZmqRadioCfg.hpp
 *
 *  Created on: Aug 14, 2017
 *      Author: dkooi 
 */

#ifndef ZMQ_RADIOCFG_HPP
#define ZMQ_RADIOCFG_HPP

namespace Zmq{

    static const NATIVE_UINT_TYPE ZMQ_RADIO_ENDPOINT_NAME_SIZE = 256;
    static const NATIVE_UINT_TYPE ZMQ_RADIO_MSG_SIZE = 256;

    static const NATIVE_UINT_TYPE ZMQ_RADIO_LINGER = 0;
    static const NATIVE_UINT_TYPE ZMQ_RADIO_RCVTIMEO = 500; // ms
    static const NATIVE_UINT_TYPE ZMQ_RADIO_SNDTIMEO = 500; // ms

    static const NATIVE_UINT_TYPE ZMQ_RADIO_SNDHWM  = 5; // Allow 5 packets to queue up

    static const NATIVE_UINT_TYPE ZMQ_RADIO_REG_MSG_SIZE = 3;
    static const NATIVE_UINT_TYPE ZMQ_RADIO_REG_RESP_MSG_SIZE = 3;

    static const NATIVE_UINT_TYPE ZMQ_RADIO_NUM_RECV_TRIES = 5;

}



#endif /* ZMQ_RADIOCFG_HPP */
