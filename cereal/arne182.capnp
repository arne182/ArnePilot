using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

using Java = import "./include/java.capnp";
$Java.package("ai.comma.openpilot.cereal");
$Java.outerClassname("arne182");

@0xca61a35dedbd6328;

struct ALCAStatus { 
  # ALCA info
  alcaEnabled @0 :Bool;
  alcaDirection @1 :Int8;
  alcaTotalSteps @2 :UInt16;
  alcaError @3 :Bool;
}

struct ALCAState {
  alcaError @0 :Bool;
  alcaCancelling @1 :Bool;
  alcaEnabled @2 :Bool;
  alcaLaneWidth @3 :Float32;
  alcaStep @4 :UInt8;
  alcaTotalSteps @5 :UInt16;
  alcaDirection @6 :Int8;
}

struct LongitudinalPIDTuningCruise {
  kpBP @0 :List(Float32);
  kpV @1 :List(Float32);
  kiBP @2 :List(Float32);
  kiV @3 :List(Float32);
  deadzoneBP @4 :List(Float32);
  deadzoneV @5 :List(Float32);
  }
  
struct ThermalOnlineData {
  cpu0 @0 :UInt16;
  cpu1 @1 :UInt16;
  cpu2 @2 :UInt16;
  cpu3 @3 :UInt16;
  mem @4 :UInt16;
  gpu @5 :UInt16;
  bat @6 :UInt32;
  pa0 @21 :UInt16;

  # not thermal
  freeSpace @7 :Float32;
  batteryPercent @8 :Int16;
  batteryStatus @9 :Text;
  batteryCurrent @15 :Int32;
  batteryVoltage @16 :Int32;
  usbOnline @12 :Bool;

  fanSpeed @10 :UInt16;
  started @11 :Bool;
  startedTs @13 :UInt64;

  thermalStatus @14 :ThermalStatus;
  chargingError @17 :Bool;
  chargingDisabled @18 :Bool;

  memUsedPercent @19 :Int8;
  cpuPerc @20 :Int8;

  enum ThermalStatus {
    green @0;   # all processes run
    yellow @1;  # critical processes run (kill uploader), engage still allowed
    red @2;     # no engage, will disengage
    danger @3;  # immediate process shutdown
  }
}

struct CarEventArne182 @0x9b1657f34caf3ad4 {
  name @0 :EventNameArne182;
  enable @1 :Bool;
  noEntry @2 :Bool;
  warning @3 :Bool;
  userDisable @4 :Bool;
  softDisable @5 :Bool;
  immediateDisable @6 :Bool;
  preEnable @7 :Bool;
  permanent @8 :Bool;

  enum EventNameArne182 @0xbaa8c5d505f727ea {
    # TODO: copy from error list
    longControlDisabled @0;
    longControlBrakeDisabled @1;
    reverseGearArne @2;
    waitingMode @3;
    wrongGearArne @4;
    rightALCbsm @5;
    leftALCbsm @6;
    preventALC @7;
    dfButtonAlert @8;
  }
}


struct CarStateArne182 {
  events @0 :List(CarEventArne182);
}


struct Arne182Status { 
  rightBlindspot @0 :Bool;
  distanceToggle @1 :Float32;
  laneDepartureToggle @2 :Bool;
  accSlowToggle @3 :Bool;
  blindspotside @4 :Float32;
  readdistancelines @5 :Float32;
  gasbuttonstatus @6 :Float32;
  lkMode @7 :Bool;
  leftBlindspot @8 :Bool;
  rightBlindspotD1 @9 :Float32;
  rightBlindspotD2 @10 :Float32;
  leftBlindspotD1 @11 :Float32;
  leftBlindspotD2 @12 :Float32;
  }

struct LiveTrafficData {
  speedLimitValid @0 :Bool;
  speedLimit @1 :Float32;
  speedAdvisoryValid @2 :Bool;
  speedAdvisory @3 :Float32;
}

struct LatControl {
  anglelater @0 :Float32;
}

struct PhantomData {
  status @0 :Bool;
  speed @1 :Float32;
  angle @2 :Float32;
}

struct ManagerData {
  runningProcesses @0 :List(Text);
}

struct SmiskolData {
  mpcTR @0 :Float32;
}

struct DynamicFollowButton {
  status @0 :UInt16;
}

struct EventArne182 {
  # in nanoseconds?
  logMonoTime @0 :UInt64;
  valid @6 :Bool = true;

  union {
    arne182Status @1:Arne182Status;
    liveTrafficData @2:LiveTrafficData;
    latControl @3:LatControl;
    phantomData @4:PhantomData;
    managerData @5:ManagerData;
    thermalonline @7:ThermalOnlineData;
    smiskolData @8 :SmiskolData;
    dynamicFollowButton @9 :DynamicFollowButton;
  }
}
