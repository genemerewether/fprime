/*
 * EthernetFTComponentImplCfg.hpp
 *
 *  Created on: Sep 21, 2017
 *      Author: tcanham
 */

#ifndef ATINETBOX_ATINETBOXCOMPONENTIMPLCFG_HPP_
#define ATINETBOX_ATINETBOXCOMPONENTIMPLCFG_HPP_

enum {
    FT_DECIMATE = 50, // filter down the number of packets to send
    FT_AA_FILT_NUMER_SIZE = 6, // number of coefficients in numerator of FT antialiasing filter
    FT_AA_FILT_DENOM_SIZE = 6, // number of coefficients in denomenator of FT antialiasing filter
    FT_AA_FILT_SIZE = FT_AA_FILT_NUMER_SIZE + FT_AA_FILT_DENOM_SIZE,
    FT_AA_ACTUAL_FILT_SIZE = FT_AA_FILT_NUMER_SIZE > FT_AA_FILT_DENOM_SIZE ? FT_AA_FILT_NUMER_SIZE : FT_AA_FILT_DENOM_SIZE
};

#endif /* ATINETBOX_ATINETBOXCOMPONENTIMPLCFG_HPP_ */
