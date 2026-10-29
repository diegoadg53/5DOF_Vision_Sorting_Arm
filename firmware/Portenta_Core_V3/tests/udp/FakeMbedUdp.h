#pragma once
#include "Arduino.h"
#include <string.h>
#include <stdio.h>

// Only the socket transport is simulated. The runner compiles the installed
// MbedUdp.cpp unchanged except for replacing its header with this fixture.
constexpr int WIFI_UDP_BUFFER_SIZE = 508;
constexpr int NSAPI_ERROR_OK = 0;
constexpr int NSAPI_ERROR_WOULD_BLOCK = -1001;
constexpr int NSAPI_ERROR_NO_SOCKET = -1002;
using nsapi_error_t = int;
using nsapi_size_or_error_t = int;
extern uint32_t testTime;
struct nsapi_addr_t {
  uint8_t bytes[4];
};

struct IPAddress {
  uint8_t bytes[4]{};
  IPAddress() = default;
  IPAddress(int a, int b, int c, int d) : bytes{uint8_t(a), uint8_t(b), uint8_t(c), uint8_t(d)} {}
  uint8_t operator[](int index) const { return bytes[index]; }
  bool operator==(const IPAddress &other) const {
    return memcmp(bytes, other.bytes, 4) == 0;
  }
  bool operator!=(const IPAddress &other) const {
    return !(*this == other);
  }
  const char *toString() const {
    return "simulated peer";
  }
};
struct SocketAddress {
  IPAddress ip;
  uint16_t port = 0;
  SocketAddress() = default;
  SocketAddress(const char *host, uint16_t p) : port(p) {
    unsigned a = 0, b = 0, c = 0, d = 0;
    sscanf(host, "%u.%u.%u.%u", &a, &b, &c, &d);
    ip = IPAddress(a, b, c, d);
  }
  nsapi_addr_t get_addr() const {
    nsapi_addr_t address;
    memcpy(address.bytes, ip.bytes, 4);
    return address;
  }
  const char *get_ip_address() const {
    return port ? "peer" : nullptr;
  }
  uint16_t get_port() const {
    return port;
  }
};
namespace SocketHelpers {
inline SocketAddress socketAddressFromIpAddress(IPAddress ip, uint16_t port) {
  SocketAddress result;
  result.ip = ip;
  result.port = port;
  return result;
}
inline void gethostbyname(void *, const char *, SocketAddress *) {}
} // namespace SocketHelpers
struct FakeSocket {
  struct Datagram {
    uint8_t data[508]{};
    int size = 0;
    SocketAddress peer;
  } incoming[16];
  int head = 0, tail = 0, receiveCalls = 0, sent = 0, lastSentSize = 0;
  uint8_t lastSent[508]{};
  SocketAddress lastDestination;
  bool open_ = false;
  bool blocking = false, lastSendBlocking = false;
  int timeout = 0, boundPort = 0, sendError = 0;
  int open(void *) {
    open_ = true;
    return 0;
  }
  int bind(uint16_t port) {
    boundPort = port;
    return 0;
  }
  int join_multicast_group(SocketAddress) {
    return 0;
  }
  void close() {
    open_ = false;
  }
  void set_blocking(bool value) { blocking = value; }
  void set_timeout(int value) { timeout = value; }
  void enqueue(const void *data, int size, IPAddress peer) {
    Datagram &d = incoming[tail++];
    memcpy(d.data, data, size);
    d.size = size;
    d.peer.ip = peer;
    d.peer.port = 4220;
  }
  int recvfrom(SocketAddress *peer, uint8_t *data, int capacity) {
    ++receiveCalls;
    if (!open_)
      return NSAPI_ERROR_NO_SOCKET;
    if (head == tail)
      return NSAPI_ERROR_WOULD_BLOCK;
    Datagram &d = incoming[head++];
    *peer = d.peer;
    int size = d.size < capacity ? d.size : capacity;
    memcpy(data, d.data, size);
    return size;
  }
  int sendto(SocketAddress peer, const uint8_t *data, int size) {
    ++sent;
    lastSendBlocking = blocking;
    if (sendError) {
      if (blocking)
        testTime += timeout;
      return sendError;
    }
    lastSentSize = size;
    lastDestination = peer;
    memcpy(lastSent, data, size);
    return size;
  }
};
struct FakeRing {
  uint8_t data[508]{};
  int count = 0, readAt = 0;
  void clear() {
    count = readAt = 0;
  }
  int available() const {
    return count - readAt;
  }
  bool availableForStore() const {
    return count < 508;
  }
  void store_char(uint8_t c) {
    data[count++] = c;
  }
  uint8_t read_char() {
    return data[readAt++];
  }
};
namespace arduino {
class MbedUDP {
public:
  FakeSocket _socket;
  SocketAddress _host, _remoteHost;
  uint8_t *_packet_buffer;
  uint8_t *_current_packet;
  size_t _current_packet_size;
  FakeRing txBuffer;
  void *getNetwork() {
    return nullptr;
  }
  MbedUDP();
  ~MbedUDP();
  uint8_t begin(uint16_t);
  uint8_t beginMulticast(IPAddress, uint16_t);
  void stop();
  int beginPacket(IPAddress, uint16_t);
  int beginPacket(const char *, uint16_t);
  int endPacket();
  size_t write(uint8_t);
  size_t write(const uint8_t *, size_t);
  int parsePacket();
  int available();
  int read();
  int read(uint8_t *, size_t);
  IPAddress remoteIP();
  uint16_t remotePort();
  void flush();
  int peek();
};
} // namespace arduino
using WiFiUDP = arduino::MbedUDP;
