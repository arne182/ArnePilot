using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

using Java = import "./include/java.capnp";
$Java.package("ai.comma.openpilot.cereal");
$Java.outerClassname("arne182");

@0xca61a35dedbd6328;

struct Arne182Status { 
  blindspot @0 :Bool;
  distanceToggle @1 :Float32;
  laneDepartureToggle @2 :Bool;
  accSlowToggle @3 :Bool;
  blindspotside @4 :Float32;
  readdistancelines @5 :Float32;
  gasbuttonstatus @6 :Float32;
  lkMode @7 :Bool;
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
  }
}
