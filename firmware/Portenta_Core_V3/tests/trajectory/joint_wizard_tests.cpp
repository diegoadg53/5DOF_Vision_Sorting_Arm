#include "JointCalibrationWizard.h"
#include <string.h>
extern "C" { volatile uint32_t test_result[3]; }
static uint32_t sends;
static bool failWrite;
static char last[128], logs[4096];
static size_t used;
static bool send(const char *line) { ++sends; snprintf(last, sizeof(last), "%s", line); return !failWrite; }
static void log(const char *line) {
  size_t n = strlen(line); if (n > sizeof(logs)-used-1) n = sizeof(logs)-used-1;
  memcpy(logs+used,line,n); used += n; logs[used] = 0;
}
static void check(bool ok, unsigned id) {
  ++test_result[0]; if (!ok) { ++test_result[1]; if (!test_result[2]) test_result[2] = id; }
}
extern "C" void *_sbrk(int n) {
  static char heap[16384]; static int at;
  if (n < 0 || at+n > int(sizeof(heap))) return (void *)-1;
  void *p = heap+at; at += n; return p;
}
extern "C" __attribute__((used,noinline)) void test_done() { for (;;) asm volatile("nop"); }
extern "C" void test_main() {
  JointCalibrationWizard w; w.bind(send,log); w.begin(1000,42);
  check(w.pending && strstr(last,"J:42,1,BEGIN"), 3000);
  w.receive("J:41,1,1,0,0",1010); check(w.pending,3001);
  w.receive("J:42,1,1,0,0",1020);
  check(!w.pending && w.phase==JC_READY && strstr(logs,"READY"),3002);
  w.receive("O:42,0,-201.100,-24.800,-276.200,-207.100,-95.400,100",1020);
  check(w.statusReceived && w.source==JC_DEFAULTS && w.offsets[0]<-201,3003);
  w.start(1030); check(strstr(last,"J:42,2,MOVE"),3004);
  w.tick(1331); check(strstr(last,"J:42,2,MOVE"),3005); // same ID on retry
  w.receive("J:42,2,2,0,0",1340); w.stop(1350);
  check(strstr(last,"J:42,3,STOP"),3006);
  w.receive("J:42,2,2,0,0",1360); check(w.pending,3007); // old ACK cannot override STOP
  w.receive("J:42,3,5,0,0",1370); check(!w.pending&&w.phase==JC_STOPPED,3008);
  w.accept(2200); check(strstr(last,"J:42,4,CAPTURE"),3009);
  w.receive("J:42,4,6,0,0",2210);
  w.receive("J:42,0,7,7,0",3100); w.accept(3110);
  check(strstr(last,"J:42,5,APPLY"),3010);
  w.receive("J:42,5,8,7,0",3120); w.accept(4000);
  check(strstr(last,"J:42,6,VERIFY"),3011);
  w.receive("J:42,6,9,7,0",4010); w.save(4020);
  check(strstr(last,"J:42,7,SAVE"),3012);
  w.receive("J:42,7,10,7,0",4030);
  check(w.committing() && strstr(last,"J:42,8,COMMIT"),3013);
  w.receive("J:42,8,11,7,0",4040); // intermediate SAVING ACK
  check(w.committing(),3014);
  uint32_t before = sends;
  w.tick(7000); w.cancel(7000); w.start(7000); w.save(7000);
  check(sends==before && w.committing() && !w.failure,3015);
  w.tick(14031); check(sends==before+1 && strstr(last,"J:42,8,COMMIT"),3016);
  w.receive("J:42,8,12,7,0",14040);
  check(!w.pending&&w.phase==JC_SAVED&&w.active,3017);
  w.cancel(14050); check(strstr(last,"J:42,9,CANCEL"),3018);
  w.receive("J:42,9,0,7,0",14060); check(!w.active,3019);
  w.begin(15000,43); w.receive("J:43,1,1,0,0",15010);
  w.tick(15110); check(strstr(last,"J:43,0,PING"),3020);
  w.tick(17100); check(w.failure!=nullptr,3021);
  w.begin(18000,44); failWrite=true; w.tick(18100);
  check(w.failure!=nullptr,3022); failWrite=false;
  w.begin(19000,45); w.receive("J:45,1,1,0,0trailing",19001); check(w.pending,3023);
  w.receive("O:45,0,nan,-24,-276,-207,-95,100",19002); check(!w.statusReceived,3024);
  w.receive("J:45,1,1,0,0",19010); w.cancel(19020);
  w.receive("J:45,2,0,0,0",19030); check(!w.active,3025);
  w.receive("H:45,7,0,201.100,24.800,276.200,0.000,1234",19040);
  check(strstr(logs,"Capture PITCH: raw L/R/elbow=") != nullptr,3026);
  w.begin(20000,46); w.receive("J:46,1,1,0,0",20010);
  w.receive("J:46,0,13,0,13",20100);
  check(w.failure && strstr(w.failure,"UART receive fault") && !w.pending,3027);
  before = sends; w.tick(20400); check(sends==before,3028);
  w.begin(21000,47); w.receive("J:47,1,1,0,0",21010);
  w.start(21020); w.receive("J:47,0,0,0,0",21100); // Older firmware's IDLE/OK abort.
  check(w.failure && strstr(w.failure,"ended unexpectedly") && !w.pending,3029);
  w.begin(22000,48); w.receive("J:48,1,0,0,3",22010); // Rejected BEGIN is immediate.
  check(w.failure && strstr(w.failure,"rejected") && !w.pending,3030);
  w.begin(23000,49); w.receive("J:49,1,1,0,0",23010);
  w.receive("J:49,0,13,0,14",23100);
  check(w.failure && strstr(w.failure,"aborted"),3031);
  w.begin(24000,50); w.receive("J:50,1,1,0,0",24010);
  w.receive("J:50,0,13,0,3",24100);
  check(w.failure && strstr(w.failure,"feedback fault"),3032);
  w.begin(25000,51); w.receive("J:51,1,1,0,0",25010);
  w.receive("J:51,0,13,0,7",25100); // Ordinary sweep stall still permits cancel.
  check(!w.failure && w.active && w.phase==JC_FAULT,3033);
  w.cancel(25110); w.receive("J:51,2,0,0,0",25120);
  check(!w.failure && !w.active,3034);
  test_done();
}
