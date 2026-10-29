#pragma once
#include "BlockDevice.h"
#include "SettingsRecord.h"
#include "PortentaStorageLayout.h"

// Current Arduino QSPIFormat layout ends user partition 4 at 14 MiB and
// places memory-mapped WiFi firmware at 15.5 MiB. Reserve two erase sectors
// starting at 14 MiB ONLY after checking the actual MBR and sector contents.
// No partition table, WiFi, OTA, provisioning or user filesystem is formatted.
class PortentaSettingsFlash {
public:
  static constexpr uint32_t start=PortentaStorageLayout::start, protectedStart=PortentaStorageLayout::protectedStart;
  uint32_t bankSize=0;
  const char *error="QSPI not opened";
  bool begin() {
    if(ready_)return true;
    root_=mbed::BlockDevice::get_default_instance();
    if(!root_ || root_->init()) {error="QSPI initialization failed";return false;}
    if(root_->size()<protectedStart) {error="QSPI too small for reserved settings region";return false;}
    const auto sector=root_->get_erase_size(start),program=root_->get_program_size(),readUnit=root_->get_read_size();
    if(!sector || sector%128 || start%sector || start+2*sector>protectedStart ||
       root_->size()<protectedStart || !program || program>4 || 4%program ||
       !readUnit || 128%readUnit || root_->get_erase_size(start+sector)!=sector) {
      error="Unsupported QSPI size/alignment";return false;
    }
    uint8_t mbr[512];
    if(root_->read(mbr,0,sizeof(mbr))) {error="QSPI MBR read failed";return false;}
    if(const char *problem=PortentaStorageLayout::issue(mbr,root_->size(),sector)) {error=problem;return false;}
    bankSize=uint32_t(sector);ready_=true;error=nullptr;return true;
  }
  bool read(uint32_t a,void *p,unsigned n) {return ready_ && a<=2*bankSize && n<=2*bankSize-a && root_->read(p,start+a,n)==0;}
  bool erase(uint32_t a,unsigned n) {return ready_ && a%bankSize==0 && n==bankSize && a<2*bankSize && root_->erase(start+a,n)==0;}
  bool program(uint32_t a,const void *p,unsigned n) {return ready_ && a<=2*bankSize && n<=2*bankSize-a && root_->program(p,start+a,n)==0;}
  bool sync() {return ready_ && root_->sync()==0;}
private:
  mbed::BlockDevice *root_=nullptr;bool ready_=false;
};
