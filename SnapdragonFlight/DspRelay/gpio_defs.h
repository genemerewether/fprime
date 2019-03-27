/*
 * gpio_defs.h
 *
 *  Created on: Oct 20, 2016
 *      Author: tcanham
 */

#ifndef DSPRELAY_INCLUDE_GPIO_DEFS_H_
#define DSPRELAY_INCLUDE_GPIO_DEFS_H_

enum DSP_RELAY_GPIO_DIRECTION_TYPE {
    DSP_RELAY_GPIO_DIRECTION_INPUT   = 0,     /**< SET TO INPUT */
    DSP_RELAY_GPIO_DIRECTION_OUTPUT  = 1,     /**< SET TO OUTPU */
    DSP_RELAY_GPIO_DIRECTION_MAX_NUM,         /**< maximum number of DspAL GPIO direction defined */
};

enum DSP_RELAY_GPIO_VALUE_TYPE {
    DSP_RELAY_GPIO_LOW_VALUE = 0,  /**< Drive the output LOW */
    DSP_RELAY_GPIO_HIGH_VALUE = 1, /**< Drive the output HIGH */
    DSP_RELAY_GPIO_VALUE_TYPE_MAX_NUM, /**< maximum number of DspAL GPIO value types defined */
};

#endif /* DSPRELAY_INCLUDE_GPIO_DEFS_H_ */
