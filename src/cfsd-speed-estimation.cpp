/*
 * Copyright (C) 2018  Love Mowitz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <mutex>
#include <chrono>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("freq")) {
    std::cerr << argv[0] << "Estimates speed for Lynx" << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --freq=<Update frequency> "
      << "[--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << "--cid=111 --freq=50 --verbose" << std::endl;
    retCode = 1;
  } else {
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    float FREQ{static_cast<float>(std::stof(commandlineArguments["freq"]))};
    bool const VERBOSE{static_cast<bool>(commandlineArguments.count("verbose"))};

    std::mutex readingsMutex;
    float wheelSpeedLeft{0.0f}, wheelSpeedRight{0.0f};

    auto onWheelSpeedReading{[&readingsMutex, &wheelSpeedLeft, &wheelSpeedRight, VERBOSE](cluon::data::Envelope &&envelope)
        {
          uint16_t senderStamp = envelope.senderStamp();
          if (senderStamp == 1904) {
            auto wheelSpeedReading = cluon::extractMessage<opendlv::proxy::AxleAngularVelocityReading>(std::move(envelope));
            {
              std::lock_guard<std::mutex> lock(readingsMutex);
              wheelSpeedLeft = wheelSpeedReading.axleAngularVelocity();
            }
            if (VERBOSE) {
              std::cout << "[ACTION-MOTION] FL wheel speed reading: " << wheelSpeedLeft << std::endl;
            }
          } else if (senderStamp == 1903) {
            auto wheelSpeedReading = cluon::extractMessage<opendlv::proxy::AxleAngularVelocityReading>(std::move(envelope));
            {
              std::lock_guard<std::mutex> lock(readingsMutex);
              wheelSpeedRight = wheelSpeedReading.axleAngularVelocity();
            }
            if (VERBOSE) {
              std::cout << "[ACTION-MOTION] FR wheel speed reading: " << wheelSpeedRight << std::endl;
            }
          }
        }};
    od4.dataTrigger(opendlv::proxy::AxleAngularVelocityReading::ID(), onWheelSpeedReading);


    auto atFrequency{[&readingsMutex, &wheelSpeedLeft, &wheelSpeedRight, &od4, VERBOSE]() -> bool
        {
            float groundSpeed;
            {
              std::lock_guard<std::mutex> lock(readingsMutex);
              groundSpeed = (wheelSpeedLeft + wheelSpeedRight) * 0.5f;
            }
            opendlv::proxy::GroundSpeedReading gsr;
            gsr.groundSpeed(groundSpeed);
            od4.send(gsr, cluon::time::now(), 2000);
            return true;
        }};
    od4.timeTrigger(FREQ, atFrequency);


    // Just sleep as this microservice is data driven
    using namespace std::literals::chrono_literals;
    while(od4.isRunning()) {
      std::this_thread::sleep_for(1s);
    }

  }
  return retCode;
}

