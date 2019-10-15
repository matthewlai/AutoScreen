/*
 * This file is part of the libostrich project.
 *
 * Copyright (C) 2019 Matthew Lai <m@matthewlai.ca>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <array>
#include <cmath>
#include <string>

#include "adc.h"
#include "gpio_defs.h"
#include "ostrich.h"
#include "systick.h"
#include "usb/serial.h"

#include <libopencm3/stm32/desig.h>

using namespace Ostrich;

constexpr bool kRecordMode = false;
constexpr uint32_t kCycleTimeMicroseconds = 27; // 37 kHz
constexpr uint32_t kOnTimeMicroseconds = 9; // 33% duty cycle to save energy
constexpr uint32_t kOffTimeMicroseconds =
    kCycleTimeMicroseconds - kOnTimeMicroseconds;
constexpr uint32_t kRecordLengthMicroseconds = 500 * 1000; // 500ms

constexpr uint32_t kUpSignalDurationMicroseconds = 1300 * 1000; // 1.3s

constexpr std::array<uint16_t, 192> kUpSignal = {
  1298, 422, 1298, 415, 1303, 417, 1298, 420, 
  446, 1274, 440, 1275, 440, 1274, 440, 1281, 
  440, 1275, 440, 1274, 446, 1274, 1298, 420, 
  440, 1275, 447, 1269, 441, 1276, 1293, 427, 
  1295, 427, 1295, 420, 446, 1267, 1295, 427, 
  1295, 420, 1305, 413, 441, 1274, 1307, 420, 
  1293, 427, 1296, 420, 441, 1272, 1297, 423, 
  1298, 419, 1300, 419, 1301, 415, 446, 22933, 
  1298, 417, 1298, 423, 1298, 416, 1300, 420, 
  445, 1274, 441, 1274, 438, 1274, 447, 1276, 
  440, 1275, 439, 1274, 441, 1274, 1302, 420, 
  441, 1274, 441, 1274, 441, 1274, 1294, 427, 
  1295, 427, 1295, 420, 440, 1281, 1295, 420, 
  1298, 419, 1298, 419, 445, 1276, 1297, 425, 
  1300, 418, 1297, 423, 442, 1276, 1297, 417, 
  1303, 421, 1297, 420, 1298, 416, 445, 22945, 
  1302, 415, 1299, 424, 1291, 427, 1295, 420, 
  441, 1276, 441, 1281, 434, 1281, 434, 1288, 
  434, 1281, 434, 1281, 441, 1274, 1294, 427, 
  441, 1274, 437, 1281, 442, 1275, 1300, 424, 
  1297, 418, 1302, 419, 439, 1278, 1297, 420, 
  1299, 419, 1299, 419, 447, 1270, 1300, 426, 
  1295, 423, 1302, 413, 444, 1279, 1297, 416, 
  1299, 420, 1299, 423, 1299, 416, 446
};

constexpr std::array<uint16_t, 192> kStopSignal = {
  1291, 425, 1295, 427, 1287, 427, 1295, 428, 
  434, 1281, 434, 1281, 434, 1281, 434, 1282, 
  440, 1275, 441, 1281, 1294, 423, 434, 1280, 
  440, 1274, 437, 1281, 442, 1274, 1294, 427, 
  1293, 427, 1295, 420, 442, 1274, 442, 1273, 
  1295, 427, 1292, 420, 439, 1280, 442, 1280, 
  1294, 421, 1300, 416, 441, 1281, 1294, 424, 
  1292, 424, 1292, 423, 446, 1275, 1295, 22077, 
  1292, 427, 1295, 421, 1298, 421, 1296, 424, 
  440, 1275, 440, 1276, 440, 1275, 439, 1280, 
  442, 1274, 442, 1274, 1294, 425, 443, 1269, 
  442, 1273, 440, 1282, 436, 1275, 1293, 431, 
  1294, 424, 1294, 424, 436, 1280, 440, 1276, 
  1299, 421, 1293, 421, 438, 1280, 440, 1276, 
  1300, 421, 1296, 422, 440, 1276, 1299, 421, 
  1291, 427, 1295, 420, 441, 1274, 1295, 22091, 
  1292, 421, 1307, 414, 1300, 419, 1305, 411, 
  449, 1267, 449, 1268, 449, 1267, 449, 1275, 
  442, 1274, 442, 1274, 1300, 416, 449, 1269, 
  444, 1274, 447, 1267, 449, 1268, 1307, 417, 
  1300, 420, 1305, 411, 449, 1268, 449, 1268, 
  1307, 414, 1298, 418, 443, 1274, 444, 1281, 
  1293, 427, 1301, 413, 441, 1274, 1300, 420, 
  1306, 413, 1299, 420, 439, 1281, 1294
};

constexpr std::array<uint16_t, 192> kDownSignal = {
  1294, 427, 1294, 421, 1300, 421, 1291, 422, 
  442, 1274, 442, 1274, 442, 1275, 442, 1276, 
  433, 1281, 1294, 427, 434, 1281, 434, 1281, 
  434, 1281, 433, 1281, 432, 1281, 1295, 427, 
  1293, 427, 1288, 427, 1298, 415, 441, 1276, 
  1297, 424, 1295, 418, 1297, 423, 440, 1280, 
  434, 1281, 436, 1276, 439, 1275, 1294, 427, 
  1292, 423, 1297, 422, 1298, 418, 1297, 22068, 
  1300, 419, 1294, 427, 1288, 427, 1288, 427, 
  434, 1281, 440, 1274, 441, 1274, 441, 1281, 
  434, 1281, 1288, 427, 434, 1281, 440, 1281, 
  434, 1277, 439, 1277, 439, 1274, 1297, 427, 
  1291, 428, 1289, 427, 1295, 424, 437, 1273, 
  1297, 423, 1296, 420, 1294, 428, 436, 1280, 
  442, 1274, 436, 1281, 439, 1276, 1291, 421, 
  1294, 426, 1294, 422, 1296, 421, 1297, 22082, 
  1287, 434, 1288, 427, 1299, 421, 1288, 427, 
  434, 1281, 434, 1283, 440, 1274, 439, 1282, 
  440, 1275, 1297, 423, 440, 1275, 440, 1275, 
  442, 1274, 442, 1274, 442, 1274, 1300, 423, 
  1298, 423, 1292, 426, 1292, 425, 442, 1274, 
  1294, 428, 1294, 420, 1295, 423, 442, 1282, 
  434, 1280, 436, 1280, 436, 1280, 1296, 421, 
  1294, 427, 1294, 422, 1295, 423, 1295
};

// These pins are used in parallel to drive the same LED.
OutputPin<PIN_B12> led_0(GPIO_PUPD_NONE, GPIO_OTYPE_OD);
OutputPin<PIN_B13> led_1(GPIO_PUPD_NONE, GPIO_OTYPE_OD);
OutputPin<PIN_B14> led_2(GPIO_PUPD_NONE, GPIO_OTYPE_OD);
OutputPin<PIN_B15> led_3(GPIO_PUPD_NONE, GPIO_OTYPE_OD);

void DriveLED(uint32_t duration_us) {
  uint32_t end_time = GetTimeMicroseconds() + duration_us;

  while (true) {
    uint32_t time_microseconds = GetTimeMicroseconds();
    if (time_microseconds > end_time) {
      led_0 = 1;
      led_1 = 1;
      led_2 = 1;
      led_3 = 1;
      break;
    }

    if ((time_microseconds % kCycleTimeMicroseconds) < kOnTimeMicroseconds) {
      led_0 = 0;
      led_1 = 0;
      led_2 = 0;
      led_3 = 0;
    } else {
      led_0 = 1;
      led_1 = 1;
      led_2 = 1;
      led_3 = 1;
    }
  }
}

void RecordMain() {
  USBSerial serial;
  SingleConversionADC<ADC1> adc;
  auto photo_input = adc.GetGPIOInput<6>();
  photo_input.SetSamplingTime(1000); // Sample at max 1MHz.

  bool recording = false;
  uint64_t recording_end_time = 0;
  bool active = false;
  bool last_active = false;
  uint64_t last_active_time = 0;

  std::vector<uint32_t> transition_times;

  while (true) {
    float reading = photo_input.ReadNormalized();
    auto time_us = GetTimeMicroseconds();

    if (reading < 0.8f) {
      active = true;
      last_active_time = time_us;
    } else if (active) {
      if ((time_us - last_active_time) > kCycleTimeMicroseconds) {
        active = false;
      }
    }

    if (!recording && active) {
      // Start recording.
      recording = true;
      recording_end_time = time_us + kRecordLengthMicroseconds;
      transition_times.clear();
      transition_times.push_back(time_us);
      last_active = active;
    } else if (recording) {
      if (time_us > recording_end_time) {
        recording = false;
        serial << "Num transitions: " << transition_times.size() << std::endl;
        for (std::size_t i = 0; i < transition_times.size() - 1; ++i) {
          int32_t diff = transition_times[i + 1] - transition_times[i];
          serial << diff << ", ";
          if (i % 8 == 7) {
            serial << std::endl;
          }
        }
        serial << std::endl;
      } else {
        if (last_active != active) {
          transition_times.push_back(time_us);
          last_active = active;
        }
      }
    }

    if (active) {
      led_0 = 0;
      led_1 = 0;
      led_2 = 0;
      led_3 = 0;
    } else {
      led_0 = 1;
      led_1 = 1;
      led_2 = 1;
      led_3 = 1;
    }
  }
}

template <typename T>
void SendSignal(const T& signal) {
  for (std::size_t i = 0; i < signal.size(); ++i) {
    if (i % 2 == 0) {
      DriveLED(signal[i]);
    } else {
      DelayMicroseconds(signal[i]);
    }
  }
}

void OperationMain() {
  // Wait for caps to charge.
  led_0 = 1;
  led_1 = 1;
  led_2 = 1;
  led_3 = 1;
  DelayMilliseconds(1000);

  // Screen down all the way.
  SendSignal(kDownSignal);
  DelayMicroseconds(20000000);
  SendSignal(kStopSignal);

  // Back up to setpoint.
  SendSignal(kUpSignal);
  DelayMicroseconds(kUpSignalDurationMicroseconds);
  SendSignal(kStopSignal);

  // First get a baseline voltage reading.
  SingleConversionADC<ADC1> adc;
  auto vbus_div_2 = adc.GetGPIOInput<10>();

  float sum = 0.0f;
  for (int i = 0; i < 100; ++i) {
    sum += vbus_div_2.ReadNormalized();
    DelayMilliseconds(10);
  }

  float avg_baseline = sum / 100.0f;

  // If we read less than baseline * 0.9 consecutively for 0.01s, we are losing
  // power.
  int consecutive_lows = 0;
  while (consecutive_lows < 100) {
    float reading = vbus_div_2.ReadNormalized();
    if (reading < (avg_baseline * 0.9f)) {
      ++consecutive_lows;
    } else {
      consecutive_lows = 0;
    }
    DelayMicroseconds(100);
  }

  // Screen up.
  SendSignal(kUpSignal);
}

int main() {
  if (kRecordMode) {
    RecordMain();
  } else {
    OperationMain();
  }
}
