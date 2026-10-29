#pragma once
#include "RuntimeTuning.h"
#include <string.h>

struct SavedRobotSettings {
  RuntimeTuning tuning;
  calibH homography;
  bool velocityValid=false;
  float velocity=0;
};
inline bool validSavedHomography(const calibH &c) {
  if(!c.valid) return true;
  double h[9];
  for(unsigned i=0;i<9;++i) {
    h[i]=c.H(i/3,i%3); if(!isfinite(h[i])) return false;
  }
  const double det=h[0]*(h[4]*h[8]-h[5]*h[7])-h[1]*(h[3]*h[8]-h[5]*h[6])+h[2]*(h[3]*h[7]-h[4]*h[6]);
  return isfinite(det) && fabs(det)>1e-12;
}
namespace SettingsRecord {
constexpr uint32_t magic=0x33544252, schema=4, committed=0x51A6C03D;
constexpr unsigned size=256, crcOffset=160, profileCrcOffset=156, legacyCrcOffset=116, commitOffset=252;
inline bool supportedSchema(uint32_t version) {return version>=1 && version<=schema;}
inline uint32_t get32(const uint8_t *p) {
  return uint32_t(p[0])|uint32_t(p[1])<<8|uint32_t(p[2])<<16|uint32_t(p[3])<<24;
}
inline void put32(uint8_t *p,uint32_t v) {for(unsigned i=0;i<4;++i)p[i]=uint8_t(v>>(8*i));}
inline uint32_t crc(const uint8_t *p,unsigned n) {
  uint32_t c=0xffffffff;
  while(n--) {c^=*p++;for(unsigned b=0;b<8;++b)c=(c>>1)^(0xedb88320U & (0U-(c&1)));}
  return ~c;
}
inline bool valid(const SavedRobotSettings &s) {
  return !s.tuning.issue() && validSavedHomography(s.homography) &&
         (!s.velocityValid || (isfinite(s.velocity) && s.velocity<-.1f && s.velocity>=-5000));
}
inline void encode(const SavedRobotSettings &s,uint32_t generation,uint8_t out[size]) {
  memset(out,0xff,size); unsigned n=0;
  auto u=[&](uint32_t v){put32(out+n,v);n+=4;};
  auto f=[&](float v){uint32_t bits;memcpy(&bits,&v,4);u(bits);};
  u(magic);u(schema);u(generation);u(crcOffset-16);
  const auto &t=s.tuning;
  u(t.marginMs);u(t.descentMs);u(t.closingLeadMs);u(t.closingDurationMs);u(t.openingMs);
  f(t.approachZ);f(t.heightAdjustment);f(t.contactZ);f(t.referenceX);f(t.testX);
  u(t.fixedTestX);u(t.testYaw);u(t.testBin);u(s.homography.valid ? 1:0);
  for(unsigned i=0;i<9;++i) f(s.homography.valid ? s.homography.H(i/3,i%3):0);
  u(s.velocityValid ? 1:0);f(s.velocityValid ? s.velocity:0);
  f(t.partXOffsetMm);
  const auto &tall=t.tall;
  u(tall.marginMs);u(tall.descentMs);u(tall.closingLeadMs);u(tall.closingDurationMs);u(tall.openingMs);
  f(tall.approachZ);f(tall.heightAdjustment);f(tall.contactZ);f(tall.partXOffsetMm);
  f(tall.id1ExtraXmm);
  put32(out+crcOffset,crc(out,crcOffset));
  put32(out+commitOffset,committed);
}
inline bool decode(const uint8_t in[size],SavedRobotSettings &out,uint32_t &generation,
                   bool requireCommit=true) {
  const uint32_t version=get32(in+4);
  const unsigned checksumOffset=version==1 ? legacyCrcOffset:version==2 ? 120:version==3 ? profileCrcOffset:crcOffset;
  const unsigned commitPosition=version<3 ? 124:commitOffset;
  if(get32(in)!=magic || !supportedSchema(version) || get32(in+12)!=checksumOffset-16 ||
     get32(in+checksumOffset)!=crc(in,checksumOffset) ||
     (requireCommit && get32(in+commitPosition)!=committed)) return false;
  unsigned n=16; SavedRobotSettings s;
  auto u=[&](){uint32_t v=get32(in+n);n+=4;return v;};
  auto f=[&](){uint32_t bits=u();float v;memcpy(&v,&bits,4);return v;};
  auto &t=s.tuning;
  t.marginMs=u();t.descentMs=u();t.closingLeadMs=u();t.closingDurationMs=u();t.openingMs=u();
  t.approachZ=f();t.heightAdjustment=f();t.contactZ=f();t.referenceX=f();t.testX=f();
  t.fixedTestX=u();t.testYaw=u();t.testBin=u();const uint32_t h=u();
  if(h>1) return false;
  s.homography.valid=h;
  for(unsigned i=0;i<9;++i)s.homography.H(i/3,i%3)=f();
  const uint32_t v=u();if(v>1)return false;s.velocityValid=v;s.velocity=f();
  // Version 1 never stored this offset. Preserve its tuning/calibrations and
  // use the current CameraTracking build default for the newly editable field.
  if(version>=2)t.partXOffsetMm=f();
  if(version>=3) {
    auto &p=t.tall;
    p.marginMs=u();p.descentMs=u();p.closingLeadMs=u();p.closingDurationMs=u();p.openingMs=u();
    p.approachZ=f();p.heightAdjustment=f();p.contactZ=f();p.partXOffsetMm=f();
  } else t.copyLegacyToBoth();
  // Schemas 1..3 retain all previous settings/calibrations and default this
  // newly optional correction to zero. Only an explicit save writes schema 4.
  if(version>=4)t.tall.id1ExtraXmm=f();
  if(n!=checksumOffset || !valid(s)) return false;
  out=s;generation=get32(in+8);return true;
}
}

// Backend addresses are relative to an exclusively checked two-sector region.
// Each save erases ONLY the inactive bank. Commit word is programmed last.
template<class Backend> class SettingsJournal {
public:
  explicit SettingsJournal(Backend &io):io_(io) {}
  uint32_t generation=0;
  const char *error="Not opened";
  bool available=false;
  bool load(SavedRobotSettings &out) {
    available=false;active_=-1;generation=0;
    if(!io_.begin()) {error=io_.error;return false;}
    uint8_t records[2][SettingsRecord::size];SavedRobotSettings candidates[2];uint32_t gens[2]{};
    bool good[2]{};
    for(unsigned b=0;b<2;++b) {
      if(!io_.read(b*io_.bankSize,records[b],sizeof(records[b]))) {error="Flash read failed";return false;}
      good[b]=SettingsRecord::decode(records[b],candidates[b],gens[b]);
    }
    if(good[0] || good[1]) {
      active_=good[1] && (!good[0] || int32_t(gens[1]-gens[0])>0) ? 1:0;
      generation=gens[active_];out=candidates[active_];available=true;error=nullptr;return true;
    }
    // Never erase unrecognized existing data. A partial record with our magic
    // and schema belongs to this journal and can be retried after power loss.
    for(unsigned b=0;b<2;++b)
      if(!(SettingsRecord::get32(records[b])==SettingsRecord::magic &&
           SettingsRecord::supportedSchema(SettingsRecord::get32(records[b]+4))) && !blank(b)) {
        error="Reserved Flash contains unrecognized data; nothing erased";return false;
      }
    available=true;error="No valid saved record; using RAM defaults";return false;
  }
  bool save(const SavedRobotSettings &s) {
    if(!available) return false;
    if(!SettingsRecord::valid(s)) {error="Invalid settings/calibration";return false;}
    const unsigned bank=active_==0 ? 1:0;const uint32_t base=bank*io_.bankSize;
    uint8_t record[SettingsRecord::size],check[SettingsRecord::size];
    SettingsRecord::encode(s,generation+1,record);
    SettingsRecord::put32(record+SettingsRecord::commitOffset,0xffffffff);
    if(!io_.erase(base,io_.bankSize) || !io_.program(base,record,sizeof(record)) ||
       !io_.read(base,check,sizeof(check)) || memcmp(record,check,sizeof(record))) {
      error="Flash write/readback failed; previous saved record retained";return false;
    }
    uint8_t commit[4];SettingsRecord::put32(commit,SettingsRecord::committed);
    if(!io_.program(base+SettingsRecord::commitOffset,commit,4) || !io_.sync() ||
       !io_.read(base,check,sizeof(check))) {error="Flash commit failed; reload before retry";return false;}
    SavedRobotSettings verified;uint32_t gen;
    if(!SettingsRecord::decode(check,verified,gen) || gen!=generation+1) {
      error="Saved record verification failed";return false;
    }
    active_=bank;generation=gen;error=nullptr;return true;
  }
private:
  Backend &io_;int active_=-1;
  bool blank(unsigned bank) {
    uint8_t b[128];
    for(uint32_t a=0;a<io_.bankSize;a+=sizeof(b)) {
      if(!io_.read(bank*io_.bankSize+a,b,sizeof(b)))return false;
      for(uint8_t v:b)if(v!=0xff)return false;
    }
    return true;
  }
};
