/*
 * uart_defs.h
 *
 *  Created on: Nov 1, 2016
 *      Author: tcanham
 */

#ifndef DSPRELAY_INCLUDE_UART_DEFS_H_
#define DSPRELAY_INCLUDE_UART_DEFS_H_

// Borrowed from DSP serial header

enum DspUartRates {
    UART_BITRATE_ILLEGAL_1,            /**< Illegal bit-rate 1*/
    UART_BITRATE_ILLEGAL_3,            /**< Illegal bit-rate 2*/
    UART_BITRATE_ILLEGAL_4,            /**< Illegal bit-rate 4*/
    UART_BITRATE_ILLEGAL_5,            /**< Illegal bit-rate 5*/
    UART_BITRATE_ILLEGAL_6,            /**< Illegal bit-rate 6*/
    UART_BITRATE_ILLEGAL_7,            /**< Illegal bit-rate 7*/
    UART_BITRATE_300,                  /**< 300  bit-rate     */
    UART_BITRATE_600,                  /**< 600  bit-rate     */
    UART_BITRATE_1200,                 /**< 1200 bit-rate     */
    UART_BITRATE_2400,                 /**< 2400 bit-rate     */
    UART_BITRATE_4800,                 /**< 4800 bit-rate     */
    UART_BITRATE_9600,                 /**< 9600 bit-rate     */
    UART_BITRATE_14400,                /**< 14400 bit-rate    */
    UART_BITRATE_19200,                /**< 19200  bit-rate   */
    UART_BITRATE_38400,                /**< 38400  bit-rate   */
    UART_BITRATE_57600,                /**< 57600  bit-rate   */
    UART_BITRATE_76800,                /**< 76800  bit-rate   */
    UART_BITRATE_115200,               /**< 115200 bit-rate   */
    UART_BITRATE_230400,               /**< 230400 bit-rate   */
    UART_BITRATE_250000,
    UART_BITRATE_460800,               /**< 460800 bit-rate   */
    UART_BITRATE_921600,               /**< 921600 bit-rate   */
    UART_BITRATE_2000000,              /**< 2000000 bit-rate  */
    UART_BITRATE_2900000,              /**< 2900000 bit-rate  */
    UART_BITRATE_3000000,              /**< 3000000 bit-rate  */
    UART_BITRATE_3200000,              /**< 3200000 bit-rate  */
    UART_BITRATE_3686400,              /**< 3686400 bit-rate  */
    UART_BITRATE_4000000              /**< 4000000 bit-rate  */
};

#endif /* DSPRELAY_INCLUDE_UART_DEFS_H_ */
