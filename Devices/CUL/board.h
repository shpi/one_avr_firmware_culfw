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

#define PB0 PORTB0
#define PB1 PORTB1
#define PB2 PORTB2
#define PB3 PORTB3
#define PB6 PORTB6
#define PD2 PORTD2
#define PD3 PORTD3

#define SPI_PORT		PORTB
#define SPI_DDR			DDRB
#define SPI_SS			PB0
#define SPI_MISO		PB3
#define SPI_MOSI		PB2
#define SPI_SCLK		PB1

#define CC1100_CS_DDR		SPI_DDR
#define CC1100_CS_PORT        SPI_PORT
#define CC1100_CS_PIN		SPI_SS
#define CC1100_OUT_DDR        DDRD
#define CC1100_OUT_PORT       PORTD
#define CC1100_OUT_PIN        PD3
#define CC1100_OUT_IN         PIND
#define CC1100_IN_DDR		DDRD
#define CC1100_IN_PORT        PIND
#define CC1100_IN_PIN         PD2
#define CC1100_IN_IN          PIND
#define CC1100_INT		INT2
#define CC1100_INTVECT        INT2_vect
#define CC1100_ISC		ISC20
#define CC1100_EICR           EICRA
#undef LED_DDR
#undef LED_PORT
#undef LED_PIN


#define CUL_HW_REVISION "CUL_V3"

#define MARK433_PORT            PORTB
#define MARK433_PIN             PINB
#define MARK433_BIT             6
#define MARK915_PORT            PORTB
#define MARK915_PIN             PINB
#define MARK915_BIT             5

#endif // __BOARD_H__
