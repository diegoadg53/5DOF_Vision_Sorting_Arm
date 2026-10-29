#pragma once
#include "SettingsRecord.h"
namespace PortentaStorageLayout {
constexpr uint32_t start=14U*1024*1024, protectedStart=15U*1024*1024;
inline const char *issue(const uint8_t mbr[512],uint64_t flashSize,uint64_t sector) {
  if(!sector || sector%128 || start%sector || start+2*sector>protectedStart || flashSize<protectedStart)
    return "Unsupported QSPI size/alignment";
  if(mbr[510]!=0x55 || mbr[511]!=0xaa)
    return "QSPI MBR missing/unrecognized; no writes permitted";
  for(unsigned i=0;i<4;++i) {
    const uint8_t *p=mbr+446+16*i;
    const uint64_t first=uint64_t(SettingsRecord::get32(p+8))*512;
    const uint64_t count=uint64_t(SettingsRecord::get32(p+12))*512;
    if(count && (!p[4] || first<512 || first+count>flashSize ||
        (first<uint64_t(start)+2*sector && first+count>start)))
      return "QSPI partition overlaps settings region or MBR is invalid; no writes permitted";
  }
  return nullptr;
}
}
