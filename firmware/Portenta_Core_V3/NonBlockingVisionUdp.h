#pragma once
#include <WiFi.h>
#include <netsocket/UDPSocket.h>
#include <stdio.h>
#include <string.h>

// Keep the vision socket nonblocking for both RX and TX. Arduino MbedUDP's
// endPacket() temporarily selects blocking TX with a 1000 ms timeout, longer
// than the controller's 200 ms watchdog. The wire format/ports are unchanged.
class NonBlockingVisionUdp {
public:
  bool begin(uint16_t port) {
    if (socket_.open(WiFi.getNetwork()) != NSAPI_ERROR_OK)
      return false;
    if (socket_.bind(port) != NSAPI_ERROR_OK) {
      socket_.close();
      return false;
    }
    socket_.set_blocking(false);
    socket_.set_timeout(0);
    ready_ = true;
    return true;
  }
  bool sendTo(IPAddress peer, uint16_t port, const uint8_t *data, size_t size) {
    if (!ready_)
      return false;
    char address[16];
    snprintf(address, sizeof(address), "%u.%u.%u.%u", unsigned(peer[0]), unsigned(peer[1]),
             unsigned(peer[2]), unsigned(peer[3]));
    SocketAddress destination(address, port);
    return socket_.sendto(destination, data, size) == int(size);
  }
  int parsePacket() {
    size_ = offset_ = 0;
    if (!ready_)
      return -1;
    int result = socket_.recvfrom(&remote_, buffer_, sizeof(buffer_));
    if (result == NSAPI_ERROR_WOULD_BLOCK)
      return 0;
    if (result <= 0)
      return -1;
    size_ = result;
    return result;
  }
  int read(uint8_t *data, size_t size) {
    size_t remaining = size_ - offset_;
    if (size > remaining)
      size = remaining;
    memcpy(data, buffer_ + offset_, size);
    offset_ += size;
    return size;
  }
  int available() const { return size_ - offset_; }
  IPAddress remoteIP() const {
    nsapi_addr_t address = remote_.get_addr();
    return IPAddress(address.bytes[0], address.bytes[1], address.bytes[2], address.bytes[3]);
  }

protected:
  UDPSocket socket_;

private:
  SocketAddress remote_;
  uint8_t buffer_[508]{};
  size_t size_ = 0, offset_ = 0;
  bool ready_ = false;
};
