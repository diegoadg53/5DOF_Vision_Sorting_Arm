#pragma once
#include "PickMotionConfig.h"

// WIDE: height<=width/yaw0. TALL: height>width/yaw90.
struct PickupTuning {
  uint32_t marginMs,descentMs,closingLeadMs,closingDurationMs,openingMs;
  float approachZ,heightAdjustment,contactZ,partXOffsetMm;
  float id1ExtraXmm=0; // Used only by TALL, confirmed ID 1, ID-classification AUTO.
  explicit PickupTuning(const AutoConfig &c=AutoConfig{})
      : marginMs(c.pickUpMarginMs),descentMs(c.descentLeadMs),closingLeadMs(c.closingLeadMs),
        closingDurationMs(c.closingDurationMs),openingMs(c.openingDurationMs),approachZ(c.approachZ),
        heightAdjustment(c.pickupHeightAdjustmentMm),contactZ(c.fittedContactZmm),
        partXOffsetMm(CameraTracking::partXOffsetMm) {}
  void apply(AutoConfig &c,float referenceX) const {
    c.pickUpMarginMs=marginMs;c.descentLeadMs=descentMs;c.closingLeadMs=closingLeadMs;
    c.closingDurationMs=closingDurationMs;c.openingDurationMs=openingMs;c.approachZ=approachZ;
    c.pickupHeightAdjustmentMm=heightAdjustment;c.fittedContactZmm=contactZ;c.heightReferenceXmm=referenceX;
    PickMotion::apply(c,referenceX);
  }
  const char *issue(float referenceX) const {
    const uint32_t times[]={marginMs,descentMs,closingLeadMs,closingDurationMs,openingMs};
    for(uint32_t v:times)if(v>60000 || v%10)return "Timing must be 0..60000ms in 10ms steps";
    AutoConfig c;apply(c,referenceX);
    if(!validPickupTiming(c))return "Require positive margin/descent/durations, descent>=close lead, duration>=close lead";
    if(!isfinite(approachZ) || approachZ < -100 || approachZ>500 ||
       !isfinite(heightAdjustment) || fabsf(heightAdjustment)>50 ||
       !isfinite(contactZ) || fabsf(contactZ)>150)return "Geometry outside editing limits";
    if(!isfinite(partXOffsetMm) || fabsf(partXOffsetMm)>100)return "Camera part X offset must be within -100..100mm";
    if(!isfinite(id1ExtraXmm) || fabsf(id1ExtraXmm)>100)return "ID 1 TALL extra X must be within -100..100mm";
    if(approachZ<=PickMotion::beltContactZ(PickMotion::minimumXmm,c) ||
       approachZ<=PickMotion::beltContactZ(PickMotion::maximumXmm,c))return "Approach Z must exceed pickup Z across the whole X range";
    return nullptr;
  }
};

// Inherited fields are WIDE, retaining legacy names for schema-1/2 migration.
struct RuntimeTuning : PickupTuning {
  PickupTuning tall;
  float referenceX=PickMotion::heightReferenceXmm,testX=332;
  uint32_t fixedTestX=0,testYaw=0,testBin=0;
  static RuntimeTuning defaults(){return RuntimeTuning{};}
  PickupTuning &profile(unsigned i){return i ? tall:static_cast<PickupTuning &>(*this);}
  const PickupTuning &profile(unsigned i)const{return i ? tall:static_cast<const PickupTuning &>(*this);}
  void apply(AutoConfig &c,unsigned i=0)const{profile(i).apply(c,referenceX);}
  void copyLegacyToBoth(){tall=profile(0);}
  void resetGeneralDefaults(){
    RuntimeTuning d;referenceX=d.referenceX;testX=d.testX;fixedTestX=d.fixedTestX;testYaw=d.testYaw;testBin=d.testBin;
  }
  const char *issue()const{
    if(!PickMotion::supportsX(referenceX))return "Reference X outside pickup range";
    if(const char *s=profile(0).issue(referenceX))return s;
    if(const char *s=tall.issue(referenceX))return s;
    if(profile(0).id1ExtraXmm!=0)return "ID 1 extra X is supported only in TALL";
    if(fixedTestX>1 || testYaw>2 || testBin>3 || !PickMotion::supportsX(testX))return "Invalid fixed/random test selection";
    return nullptr;
  }
  float extraXmm(uint8_t confirmedId,bool isTall)const{return isTall && confirmedId==1 ? tall.id1ExtraXmm:0;}
  PickupProfilePlan plans(const AutoConfig &base,float unshiftedX,bool cameraPart=true,
                         uint8_t confirmedId=0)const{
    PickupProfilePlan p;
    for(unsigned i=0;i<2;++i){
      p.config[i]=base;apply(p.config[i],i);
      p.x[i]=unshiftedX+(cameraPart ? profile(i).partXOffsetMm+extraXmm(confirmedId,i!=0):0);
      PickMotion::apply(p.config[i],p.x[i]);p.available[i]=PickMotion::supportsX(p.x[i]);
    }
    return p;
  }
};

namespace PickupTuningMenu {
enum Row {Margin,Descent,ClosingLead,ClosingDuration,Opening,ApproachZ,HeightAdjustment,ContactZ,PartXOffset,Id1ExtraX,Count};
inline unsigned count(unsigned profile){return profile ? Count:Id1ExtraX;}
inline const char *name(unsigned r){
  static const char *const names[]={"Approach margin (ms)","Descent lead (ms)","Closing lead (ms)",
    "Closing duration (ms)","Opening duration (ms)","Approach TCP Z (mm)",
    "Pickup height adjustment (mm)","Reference contact Z (mm)","Camera part X offset (mm)",
    "ID 1 TALL extra X (mm; ID classification only)"};
  return r<Count ? names[r]:"?";
}
inline float value(const PickupTuning &p,unsigned r){
  switch(r){case Margin:return p.marginMs;case Descent:return p.descentMs;case ClosingLead:return p.closingLeadMs;
    case ClosingDuration:return p.closingDurationMs;case Opening:return p.openingMs;case ApproachZ:return p.approachZ;
    case HeightAdjustment:return p.heightAdjustment;case ContactZ:return p.contactZ;case PartXOffset:return p.partXOffsetMm;
    case Id1ExtraX:return p.id1ExtraXmm;default:return 0;}
}
inline void adjust(PickupTuning &p,unsigned r,int direction){
  if(!direction)return;float lo=0,hi=60000;
  if(r==ApproachZ){lo=-100;hi=500;}else if(r==HeightAdjustment){lo=-50;hi=50;}
  else if(r==ContactZ){lo=-150;hi=150;}else if(r==PartXOffset || r==Id1ExtraX){lo=-100;hi=100;}
  const float step=r<=Opening ? 10:.5f;
  const float v=fmaxf(lo,fminf(hi,value(p,r)+(direction>0 ? step:-step)));
  switch(r){case Margin:p.marginMs=uint32_t(v);break;case Descent:p.descentMs=uint32_t(v);break;
    case ClosingLead:p.closingLeadMs=uint32_t(v);break;case ClosingDuration:p.closingDurationMs=uint32_t(v);break;
    case Opening:p.openingMs=uint32_t(v);break;case ApproachZ:p.approachZ=v;break;
    case HeightAdjustment:p.heightAdjustment=v;break;case ContactZ:p.contactZ=v;break;case PartXOffset:p.partXOffsetMm=v;break;
    case Id1ExtraX:p.id1ExtraXmm=v;break;}
}
}
namespace TuningMenu {
enum Row {ReferenceX,TestFixedX,TestX,TestYaw,TestBin,Id1TallExtraX,Apply,Save,Reload,Defaults,Count};
inline const char *name(unsigned r){
  static const char *const names[]={"Shared reference X (mm)","Test X: random=0 fixed=1","Test fixed X (mm)",
    "Test yaw: random=0 yaw0=1 yaw90=2","Test bin: random=0 or 1..3",
    "ID 1 TALL extra X (mm; ID classification only)","APPLY TO RAM","SAVE ALL TO FLASH",
    "RELOAD SAVED SETTINGS + CALIBRATIONS","BUILD DEFAULT SHARED/TEST SETTINGS (draft only)"};return r<Count ? names[r]:"?";
}
inline float value(const RuntimeTuning &t,unsigned r){
  switch(r){case ReferenceX:return t.referenceX;case TestFixedX:return t.fixedTestX;case TestX:return t.testX;
    case TestYaw:return t.testYaw;case TestBin:return t.testBin;case Id1TallExtraX:return t.tall.id1ExtraXmm;default:return 0;}
}
inline void adjust(RuntimeTuning &t,unsigned r,int direction){
  if(!direction)return;float lo=0,hi=1,step=1;
  if(r==ReferenceX || r==TestX){lo=PickMotion::minimumXmm;hi=PickMotion::maximumXmm;step=.5f;}
  else if(r==Id1TallExtraX){lo=-100;hi=100;step=.5f;}
  else if(r==TestYaw)hi=2;else if(r==TestBin)hi=3;
  const float v=fmaxf(lo,fminf(hi,value(t,r)+(direction>0 ? step:-step)));
  switch(r){case ReferenceX:t.referenceX=v;break;case TestFixedX:t.fixedTestX=uint32_t(v);break;
    case TestX:t.testX=v;break;case TestYaw:t.testYaw=uint32_t(v);break;case TestBin:t.testBin=uint32_t(v);break;
    case Id1TallExtraX:t.tall.id1ExtraXmm=v;break;}
}
}
