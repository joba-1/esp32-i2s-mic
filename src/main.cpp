/**
 * Enhanced streams-i2s-webserver_wav.ino optimized for INMP441
 * 
 * - Features a flexible mono to multi channel converter
 * - #define COPY_LOG_OFF or change AudioLogger level to Warning to avoid hickups
 * - Adjust pin numbers in setup() according to your board
 * - Works fine with Chrome or VLC, my INMP441 and git main of ESP32 pio platform and arduino-audio-tools as of Nov 2023
 * - Works with I2S_MSB_FORMAT (datasheet), and also with I2S_STD_FORMAT, I2S_PHILIPS_FORMAT or I2S_LEFT_JUSTIFIED_FORMAT
 * - I see hickups with 44100 samples/s. Probably too much for the hardware?
 * 
 * have fun, JoBa-1
 */

#include <Arduino.h>

#include "AudioTools.h"

// Contains MY_SSID and MY_PASS 
#include "cred.h"


/**
 * @brief Extend channel Cx value of type T to all Cn channels
 */
template<typename T, size_t Cn, size_t Cx>
class ExtendChan : public BaseConverter {
  public:
  ExtendChan() : _max_val(0), _counter(0), _prev_ms(0) {}

  size_t convert(uint8_t *src, size_t size) {
    T *chan = (T*)src;
    size_t samples = (size / Cn) / sizeof(T);
    for( size_t s=0; s<samples; s++ ) {
      T val = (Cx < Cn) ? chan[s*Cn+Cx] : 0;

      for( size_t c=0; c<Cn; c++ ) {
        if( c != Cx ) {
          chan[s*Cn+c] = val;
        }
      }

      if( _max_val < val ) { 
          _max_val = val;
      }

      _counter++;
      uint32_t now = millis();
      if( now - _prev_ms > 1000 ) { 
        _prev_ms = now;
        LOGI("ExtendChan samples: %u, amplitude: %d", _counter, _max_val);
        _max_val = 0;
      }
    }
    return samples * Cn * sizeof(T);
  }

  private:
  T _max_val;
  uint32_t _counter;
  uint32_t _prev_ms;
};


// Type of a channel sample, number of channels and sample rate
typedef int32_t in_chan_t;            // received sample size: low 24 of 32 bits, see INMP441 datasheet
typedef int16_t out_chan_t;           // as supported by AudioWAVServer
constexpr size_t NumChan = 2;         // received channels, see INMP441 datasheet
constexpr size_t SampleRate = 22050;  // lower gives fast playback and gaps


constexpr size_t InChanBytes = sizeof(in_chan_t);
constexpr size_t OutChanBytes = sizeof(out_chan_t);

constexpr size_t InChanBits = InChanBytes * CHAR_BIT;
constexpr size_t OutChanBits = OutChanBytes * CHAR_BIT;


AudioInfo in(SampleRate, NumChan, InChanBits);
AudioInfo out(SampleRate, NumChan, OutChanBits);

/**
 * @brief Custom Stream 
 * someone reads from our out stream, which triggers us reading from our input stream
 * read lower 24bit of 32bit values, shift by 8 bits to return 16bit values
 */
class Convert024to16 : public AudioStream {
public:
  Convert024to16(Stream &stream) : _in(&stream) {}

  void setStream(Stream &stream) { _in = &stream; }

  bool begin() {
    TRACEI();
    _read = 0;
    _last = 0;
    _written = 0;
    _maxval = 0;
    return _in != NULL;
  }

  void end() {
    TRACEI();
    _in = NULL;
  }

  /// amount of data available
  int available() { return (_in->available() / InChanBytes) * OutChanBytes; }
  // int available() { return _in->available(); }

  /// Read 32bit, convert to 16bit and send to out stream
  size_t readBytes(uint8_t *buffer, size_t size) override {
    // only read in chunks of out stream type (for now 16 bit)
    size_t samples = size / OutChanBytes;
  
    // write at most as much as in stream can provide
    size_t available_samples = _in->available() / InChanBytes;
    if( available_samples < samples ) samples = available_samples;

    for( size_t i = 0; i < samples; i++ ) {
      in_chan_t value;
      size_t read = _in->readBytes((uint8_t *)&value, InChanBytes);

      _read += read;

      if( read != InChanBytes ) {
        LOGE("READ ERROR");
        return i * OutChanBytes;
      }

      // convert one 24bit value in 32bits to 16bit value and store in buffer
      // *((out_chan_t *)buffer + i) = (out_chan_t)((value & 0xffffff) >> 8);
      uint8_t *val = (uint8_t *)&value;
      // val0: zero 
      // val1: noise?
      // val2: low, 
      // val3: high
      int16_t v2 = (int16_t)(val[3] << 8 | val[2]);

      // enhance low volume
      bool negate = false;
      if( v2 < 0 ) {
        negate = true;
        v2=-v2;
      } 
      v2 = v2 - INT16_MAX;
      v2 = INT16_MAX - (v2 * v2) / INT16_MAX;
      if( negate ) {
        v2 = -v2;
      }

      if( _maxval < v2 ) _maxval = v2;

      buffer[OutChanBytes*i] = v2 & 0xff;  // val[2];    // low
      buffer[OutChanBytes*i+1] = (v2 >> 8) & 0xff;  // val[3] ;  // high
    }

    _written += samples * OutChanBytes;

    uint32_t now = millis();
    if( now - _last > _interval ) {
      _last = now;
      LOGI("Written   %u Bytes/s", _written);
      LOGI("Read/2    %u Bytes/s", _read/2);
      LOGI("Amplitude %u", _maxval);
      _written = 0;
      _read = 0;
      _maxval = 0;
    }

    return samples * OutChanBytes;
  }

private:
  Stream *_in;
  uint32_t _read;
  uint32_t _last;
  uint32_t _written;
  uint16_t _maxval;

  static const uint32_t _interval = 1000;
};


I2SStream i2s;  // INMP441 delivers 24 as 32bit
Convert024to16 cvt(i2s);
ExtendChan<out_chan_t, NumChan, 0> extend;  // one channel has no data, see INMP441 datasheet
AudioWAVServer srv(MY_SSID, MY_PASS);  // Streaming with VLC and Chrome works, Firefox downloads.


void setup() {
  Serial.begin(BAUDRATE);
  AudioLogger::instance().begin(Serial, AudioLogger::Info);

  auto icfg = i2s.defaultConfig(RX_MODE);
  icfg.i2s_format = I2S_MSB_FORMAT;
  icfg.copyFrom(in);

  icfg.pin_data = 21;
  icfg.pin_ws = 17;
  icfg.pin_bck = 16;

  i2s.begin(icfg);
  cvt.begin();

  auto wcfg = srv.wavEncoder().defaultConfig();
  wcfg.copyFrom(out);
  srv.begin(cvt, out, &extend);
  srv.wavEncoder().setAudioInfo(wcfg);
}


void loop() {
  srv.copy();
}
