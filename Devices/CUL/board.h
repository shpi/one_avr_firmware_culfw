#ifndef _BOARD_H
#define _BOARD_H

// Feature definitions
#define BOARD_ID_STR            "CUL868"
#define BOARD_ID_USTR           L"CUL868"

#undef MULTI_FREQ_DEVICE	// available in multiple versions: 433MHz,868MHz
#define BOARD_ID_STR433         "CUL433"
#define BOARD_ID_USTR433        L"CUL433"

#define HAS_USB                  1
#define USB_BUFSIZE             64      // Must be a supported USB endpoint size
#define USB_MAX_POWER	       100
#undef HAS_FHT_80b                     // PROGMEM: 1374b, RAM: 90b
#undef HAS_RF_ROUTER                   // PROGMEM: 1248b  RAM: 44b
#undef RFR_FILTER                      // PROGMEM:   90b  RAM:  4b
#undef HAS_HOERMANN
#undef HAS_HOERMANN_SEND               // PROGMEM:  220
#undef HAS_CC1101_RX_PLL_LOCK_CHECK_TASK_WAIT	// PROGMEM: 118b
#undef HAS_CC1101_PLL_LOCK_CHECK_MSG		// PROGMEM:  22b
#undef HAS_CC1101_PLL_LOCK_CHECK_MSG_SW	// PROGMEM:  22b

#undef  RFR_DEBUG                       // PROGMEM:  354b  RAM: 14b
#define  HAS_FASTRF                      // PROGMEM:  468b  RAM:  1b

#undef HAS_FHT_8v                    // PROGMEM:  586b  RAM: 23b
#undef HAS_FHT_TF
#undef FHTBUF_SIZE                   //                 RAM: 174b
#define RCV_BUCKETS            4      //                 RAM: 25b * bucket
#define FULL_CC1100_PA                // PROGMEM:  108b
#define HAS_RAWSEND                   //
#undef HAS_ASKSIN                    // PROGMEM: 1314
#undef HAS_ASKSIN_FUP                // PROGMEM:   78
#define HAS_MORITZ                    // PROGMEM: 1696
#undef HAS_ESA                       // PROGMEM:  286
#undef HAS_TX3                       // PROGMEM:  168
#undef HAS_INTERTECHNO               // PROGMEM: 1352
#undef HAS_TCM97001                  // PROGMEM:  264
#undef HAS_UNIROLL                   // PROGMEM:   92
#undef HAS_MEMFN                     // PROGMEM:  168
#undef HAS_SOMFY_RTS                 // PROGMEM: 1716
#undef HAS_BELFOX                    // PROGMEM:  214

#define TTY_BUFSIZE          128      // RAM: TTY_BUFSIZE*4
#undef HAS_MBUS                      // PROGMEM: 2536
#undef MBUS_NO_TX                       // PROGMEM:  962
#undef HAS_RFNATIVE                  // PROGMEM:  580
#undef HAS_KOPP_FC                   // PROGMEM: 3370

// No features to define below

#include <avr/io.h>
#include <avr/power.h>

#if !defined(clock_prescale_set) && __AVR_LIBC_VERSION__  < 10701UL
#  warning "avr/power.h needs patching for prescaler functions to work."
#  warning "for the m32u4 add __AVR_ATmega32U4__ for cpu types on prescale block"
#  warning "for the m32u2 add __AVR_ATmega32U2__ for cpu types on prescale block"
#endif

#define PB1 PORTB1
#define PB2 PORTB2
#define PB3 PORTB3
#define PD6 PORTD6
#define PE7 PORTE7
#define PF7 PORTF7

#define SPI_PORT		PORTB
#define SPI_DDR			DDRB
#define SS_DDR          DDRD
#define SPI_SS			PD6
#define SPI_MISO		PB3
#define SPI_MOSI		PB2
#define SPI_SCLK		PB1

#define CC1100_CS_DDR		SS_DDR
#define CC1100_CS_PORT        PORTD
#define CC1100_CS_PIN		SPI_SS
#define CC1100_OUT_DDR        DDRF
#define CC1100_OUT_PORT       PORTF
#define CC1100_OUT_PIN        PF7
#define CC1100_OUT_IN         PINF
#define CCTEMP_MUX            CC1100_OUT_PIN

#define CC1100_IN_DDR		DDRE
#define CC1100_IN_PORT        PINE
#define CC1100_IN_PIN         PE6
#define CC1100_INT		INT6
#define CC1100_INTVECT        INT6_vect
#define CC1100_ISC		ISC60
#define CC1100_EICR           EICRB

#undef LED_DDR
#undef LED_PORT
#undef LED_PIN


#define CUL_HW_REVISION "CUL_V3"

#undef MARK433_PORT            //PORTB
#undef MARK433_PIN             //PINB
#undef MARK433_BIT             //6
#undef MARK915_PORT            //PORTB
#undef MARK915_PIN             //PINB
#undef MARK915_BIT             //5

#ifndef SET_BIT
#define SET_BIT(PORT, BITNUM) ((PORT) |= (1<<(BITNUM)))
#endif
#ifndef CLEAR_BIT
#define CLEAR_BIT(PORT, BITNUM) ((PORT) &= ~(1<<(BITNUM)))
#endif
#define TOGGLE_BIT(PORT, BITNUM) ((PORT) ^= (1<<(BITNUM)))

#endif // __BOARD_H__
