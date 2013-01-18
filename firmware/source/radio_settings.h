/*
 * Radio settings
 *
 * created on 2012-5-14 by fgb (derived from or_const.h by apullin)
 */

#ifndef __RADIO_SETTINGS_H
#define __RADIO_SETTINGS_H

#ifdef OCTOROACH
#define RADIO_MY_CHAN         0x11
#define RADIO_PAN_ID          0x2110
#define RADIO_DEST_ADDR       0x2111
#define RADIO_SRC_ADDR        0x2112
#endif

#define RONF
// ronf robot radio addresses
#ifdef RONF
#define RADIO_MY_CHAN 0x13
#define RADIO_PAN_ID 0x2060
//Hard code in destination address (basestation) for now, update to be dynamic later
#define RADIO_DEST_ADDR 0x2011
#define RADIO_SRC_ADDR 0x2052
#endif



#define RADIO_TXPQ_MAX_SIZE   30
#define RADIO_RXPQ_MAX_SIZE   10


#endif //__RADIO_SETTINGS_H
