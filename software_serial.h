// Copyright 2009 Olivier Gillet.
//
// Author: Olivier Gillet (ol.gillet@gmail.com)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// -----------------------------------------------------------------------------
//
// BufferedSoftwareSerialOutput, the name says it all:
// - It is software serial.
// - Contrary to the other Software Serial implementations out there which use
// fine-tuned busy loop to delay between bits, this implementation bangs bits
// in a timer interrupt ; while writes are done in an asynchronous, non-blocking
// way to a buffer.
//
// Caveats:
// - The timer rate must be a multiple of the baud rate otherwise this will
// miserably FAIL.
// - Tested only for 2400 (main timer at 31250 Hz) and 4800 (main timer at
// 62500 Hz) baud per seconds.
// - Please check the timing tolerance of your device's UART!
//
// SoftwareSerialOutput is a Vanilla blocking implementation, taken from the
// Arduino libs ; Copyright (c) 2006 David A. Mellis.

#ifndef AVRLIB_SOFTWARE_SERIAL_H_
#define AVRLIB_SOFTWARE_SERIAL_H_

#include <util/delay_basic.h>
#include "avrlib/avrlib.h"
#include "avrlib/gpio.h"
#include "avrlib/ring_buffer.h"

namespace avrlib {

enum SoftwareSerialState {
  START_BIT = 0,
  END_BIT = 9,
  NEXT_BYTE = 10
};

// Parameters:
// TxPin: digital pin used for transmission.
// timer_rate: frequency (Hz) of the timer which will drive the output.
// baud_rate: target baud rate. must be a divisor of timer_rate.
// buffer_size: prefered buffer size.
template<typename TxPin, uint16_t timer_rate, uint32_t baud_rate,
          uint8_t buffer_size_>
class BufferedSoftwareSerialOutput {
  typedef BufferedSoftwareSerialOutput<TxPin, timer_rate,
                                       baud_rate, buffer_size_> Me;
  typedef RingBuffer<Me> OutputBuffer;
 public:
  typedef uint8_t Value;
  enum {
    prescaler_reset_value = timer_rate / baud_rate,
    buffer_size = buffer_size_,
    data_size = 8
  };
  static void Init() {
    prescaler_counter_ = prescaler_reset_value;
    tx_state_ = NEXT_BYTE;
    TxPin::set_mode(DIGITAL_OUTPUT);
    TxPin::High();
  }
  static inline void Write(Value v) {
    OutputBuffer::Write(v);
  }
  static inline uint8_t writable() { return OutputBuffer::writable(); }
  static inline uint8_t NonBlockingWrite(Value v ) {
    return OutputBuffer::NonBlockingWrite(v);
  }
  static inline void Overwrite(Value v) { OutputBuffer::Overwrite(v); }
  static inline void Tick() {
    --prescaler_counter_;
    if (prescaler_counter_ > 0) {
      return;
    }
    prescaler_counter_ = prescaler_reset_value;
    // Check in which stage of the transmission we are.
    if (tx_state_ == NEXT_BYTE) {
      // Attempt to start the transmission of the next byte in the buffer, (if
      // there is any).
      int16_t next_byte = OutputBuffer::NonBlockingRead();
      if (next_byte >= 0) {
        tx_byte_ = next_byte;
        tx_state_ = START_BIT;
        return;
      } else {
        return;
      }
    } else if (tx_state_ == START_BIT) {
      TxPin::Low();
      tx_symbol_mask_ = 1;
    } else if (tx_state_ == END_BIT) {
      TxPin::High();
    } else {
      TxPin::set_value(tx_byte_ & tx_symbol_mask_);
      tx_symbol_mask_ <<= 1;
    }
    ++tx_state_;
  }

 private:
  static uint8_t prescaler_counter_;

  // Mask shifted every symbol, used to extract bits from the current character
  // to transmit.
  static uint8_t tx_symbol_mask_;

  static uint8_t tx_state_;
  static uint8_t tx_byte_;

  DISALLOW_COPY_AND_ASSIGN(BufferedSoftwareSerialOutput);
};

template<typename TxPin, uint16_t timer_rate, uint32_t baud_rate,
          uint8_t buffer_size_>
uint8_t BufferedSoftwareSerialOutput<TxPin, timer_rate, baud_rate,
                                     buffer_size_>::prescaler_counter_;

template<typename TxPin, uint16_t timer_rate, uint32_t baud_rate,
          uint8_t buffer_size_>
uint8_t BufferedSoftwareSerialOutput<TxPin, timer_rate, baud_rate,
                                     buffer_size_>::tx_symbol_mask_;

template<typename TxPin, uint16_t timer_rate, uint32_t baud_rate,
          uint8_t buffer_size_>
uint8_t BufferedSoftwareSerialOutput<TxPin, timer_rate, baud_rate,
                                     buffer_size_>::tx_state_;

template<typename TxPin, uint16_t timer_rate, uint32_t baud_rate,
          uint8_t buffer_size_>
uint8_t BufferedSoftwareSerialOutput<TxPin, timer_rate, baud_rate,
                                     buffer_size_>::tx_byte_;


// Parameters:
// TxPin: digital pin used for transmission.
// baud_rate: target baud rate
// Following code from NewSoftSerial, Copyright (c) 2006 David A. Mellis.
// For proper timing delay should be much greater than overhead
// So for F_CPU=1MHz baud rates more than 19200 are questionable
template<typename TxPin, uint32_t baud_rate>
struct SoftwareSerialOutput {
  static void Init() {
    TxPin::set_mode(DIGITAL_OUTPUT);
    TxPin::High();
  }
  static const uint16_t delay_iterations = F_CPU / baud_rate / 4; // 4 CPU cycles per iteration
  static void Write(uint8_t tx_byte) {
    uint8_t oldSREG = SREG;
    cli();
    TxPin::Low();
    _delay_loop_2(delay_iterations - 2); // correction for overhead
    for (uint8_t i = 8; i; i--) {
      TxPin::set_value(tx_byte & 1);
      _delay_loop_2(delay_iterations - 2);
      tx_byte >>= 1;
    }
    TxPin::High();
    SREG = oldSREG;
    _delay_loop_2(delay_iterations);
  }
};

}  // namespace avrlib

#endif // AVRLIB_SOFTWARE_SERIAL_H_
