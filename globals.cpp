#include "globals.h"

#include <iostream>
#include <time.h>
#include <X11/Xlib.h>

#include "controllooper.h"
#include "imu.h"
#include "hw/i2c.h"
#include "hw/uart.h"
#include "propellers.h"
#include "quad.h"

#include "proto.pb.h"

using namespace std;

Globals &Globals::self() {
  static Globals instance;
  return instance;
}

Globals::Globals() {
  simulated_quad_running_ = true;

  if (XOpenDisplay(NULL)) {
    environment_ = Simulation;
    i2c_ = nullptr;
    simulated_quad_ = new Quad();
    remote_quad_ = new Quad();
  } else {
    environment_ = OnBoard;
    i2c_ = new I2c();
    simulated_quad_ = nullptr;
    remote_quad_ = nullptr;
  }

  const char *uart_usb = "/dev/ttyUSB0";
  const char *uart_ama = "/dev/ttyAMA0";
  uart_ = new Uart(environment_ == OnBoard ? uart_ama : uart_usb);

  control_looper_ = new ControlLooper();
  imu_ = new Imu(environment_);
  propellers_ = new Propellers(environment_);

  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  initial_seconds_ = ts.tv_sec;

  if (environment_ == Simulation) {
    new thread(&Globals::runSimulatedQuad, this);
  } /*else {*/
    new thread(&Globals::doCommunicationLoop, this);
//   } // TODO !THIS !
}

int64_t Globals::currentTime_us() const {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return 1000000 * (ts.tv_sec - initial_seconds_) + ts.tv_nsec / 1000;
}

void Globals::sleep_us(int64_t duration) const {
  timespec ts;
  ts.tv_sec  =  duration / 1000000;
  ts.tv_nsec = (duration % 1000000) * 1000;
  nanosleep(&ts, 0); // TODO should make use of the return value, in case error or interrupt?
};

void Globals::sleepUntil_us(int64_t time) const {
  int64_t current = currentTime_us();
  if (current >= time) {
    return;
  }
  sleep_us(time - current);
}

void Globals::setSimulationRunning(bool run) {
  assert(isSimulation());
  simulated_quad_running_ = run;
  imu_->setUpdating(run);
}

void Globals::reset() {
  if (isSimulation()) {
    simulated_quad_->reset();
    setSimulationRunning(false);
  }
  imu_->reset();
  propellers_->reset();
  if (remote_quad_ != nullptr) {
    remote_quad_->reset();
  }
}

void Globals::runSimulatedQuad() {
  sleep_us(100000); // sleep 0.1 seconds to allow stuff to come online
  int64_t start_time = Globals::self().currentTime_us();
  int64_t iteration = 0;
  const int64_t refresh_us = TsWorldMs * 1000 / 10;
  while (true) {
    iteration++;
    if (simulated_quad_running_) {
      simulated_quad_->step(refresh_us * 1e-6);
    }
    sleepUntil_us(start_time + iteration * refresh_us);
  }
}

Vector3d protoTranslate(Proto::Message::Vector3d proto) {
  return Vector3d(proto.x(), proto.y(), proto.z());
}

Proto::Message::Vector3d protoTranslate(Vector3d actual) {
  Proto::Message::Vector3d proto;
  proto.set_x(actual[0]);
  proto.set_y(actual[1]);
  proto.set_z(actual[2]);
  return proto;
}

Vector4d protoTranslate(Proto::Message::Vector4d proto) {
  return Vector4d(proto.w(), proto.x(), proto.y(), proto.z());
}

Proto::Message::Vector4d protoTranslate(Vector4d actual) {
  Proto::Message::Vector4d proto;
  proto.set_w(actual[0]);
  proto.set_x(actual[1]);
  proto.set_y(actual[2]);
  proto.set_z(actual[3]);
  return proto;
}

Quaterniond protoTranslate(Proto::Message::Quaterniond proto) {
  return Quaterniond(proto.w(), proto.x(), proto.y(), proto.z());
}

Proto::Message::Quaterniond protoTranslate(Quaterniond actual) {
  Proto::Message::Quaterniond proto;
  proto.set_w(actual.w());
  proto.set_x(actual.x());
  proto.set_y(actual.y());
  proto.set_z(actual.z());
  return proto;
}

QuadState protoTranslate(Proto::Message::QuadState proto) {
  QuadState quad_state;
  quad_state.pos = protoTranslate(proto.pos());
  quad_state.vel = protoTranslate(proto.vel());
  quad_state.orient = protoTranslate(proto.orient());
  quad_state.omega = protoTranslate(proto.omega());
  return quad_state;
}

Proto::Message::QuadState protoTranslate(QuadState actual) {
  Proto::Message::QuadState proto;
  *proto.mutable_pos() = (protoTranslate(actual.pos));
  *proto.mutable_vel() = (protoTranslate(actual.vel));
  *proto.mutable_orient() = (protoTranslate(actual.orient));
  *proto.mutable_omega() = (protoTranslate(actual.omega));
  return proto;
}

void Globals::doCommunicationLoop() {
  while (true) {
    sleep_us(100000); // 100 ms sleep
    while (true) {
      string message = uart_->getMessage();
      if (message.empty()) {
        break;
      }
      handleMessage(message);
    }

    if (environment_ == OnBoard) {
      // Send updates about ourselves..
      Proto::Message proto_message;
      proto_message.set_type(Proto::Message::Type::Message_Type_TYPE_QUAD_UPDATE);
      *proto_message.mutable_prop_input() = protoTranslate(propellers_->input());;
      *proto_message.mutable_quad_state() = protoTranslate(imu_->lastState());
      ostringstream stream;
      proto_message.SerializeToOstream(&stream);
      uart_->writeMessage(stream.str());
      cout << "Sent message with quad update" << endl;
    }
  }
}

void Globals::handleMessage(string const& message) {
  Proto::Message proto_message;
  if (!proto_message.ParseFromString(message)) {
    cerr << "Failed to parse message in Globals::handleMessage" << endl;
    return;
  }
  switch (proto_message.type()) {
    case Proto::Message::Type::Message_Type_TYPE_QUAD_UPDATE: {
      cout << "Handling proto-update message" << endl;
      QuadState quad_state = protoTranslate(proto_message.quad_state());
      Vector4d prop_input = protoTranslate(proto_message.prop_input());
      remote_quad_->setState(quad_state);
      remote_quad_->setPropInput(prop_input);
      break;
    }
    default: {
      cerr << "Unknown message type in Globals::handleMessage" << endl;
      break;
    }
  }
}

