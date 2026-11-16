// A robot on the Clunkyard core's C++ binding. The core owns tracking,
// geometry and protocol handling; these callbacks choose the tactics.
// It searches with a broad scan, narrows the beam on a contact, and leads
// shots from a line through its recent sightings.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied hull_mounts to_mount(heading pitch)
//   from_mount(heading pitch) mount_toward(point)
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   history(slot) last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) to_chassis(heading pitch attitude)
//   from_chassis(heading pitch attitude) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) debug.log(line) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
#include "clunkyard.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

// Bearing is earned: a scan contact's bearing is off by up to its bearing_error
// (degrees, a fixed share of the arc), so a wide beam finds a robot and a narrow
// one aims at it. Contacts within 120 units are exact.
constexpr double PI=3.14159265358979323846;  // the C++ standard has no pi constant before C++20
constexpr double SEARCH_ARC=40;  // degrees: sweeps the circle in half a second
constexpr double HOLD_WIDTH=60;  // units across at the target: it stays in the beam from tick to tick
constexpr double MIN_ARC=6;      // degrees: the narrowest the beam holds
constexpr int WINDOW=12;         // ticks of sightings the aim averages
constexpr int SIGHTINGS=8;       // narrow sightings in the window before a shot
constexpr int LOOK_EVERY=40;     // ticks between looks around the target, for a nearer robot
constexpr int REACQUIRE=10;      // ticks to look again where a lost target was

// A straight line through the narrowest of the last WINDOW ticks' sightings:
// where the robot is now and how fast it moves.
struct Fit { cy_vec3 position, velocity; int count; };

class Starter : public clunkyard::Robot {
  bool free_flight=false;int dodge_ticks=0,dodges=0,slot=0;
  // Each sighting is off by its bearing_error; the line averages that out.
  std::optional<Fit> fit(int32_t target,int tick) {
    std::vector<clunkyard::Sighting> recent;double best=INFINITY;
    for(const auto &s:tracker.history(target))if(s.tick>tick-WINDOW){recent.push_back(s);best=std::min(best,s.bearing_error);}
    if(recent.size()<2)return std::nullopt;
    std::vector<clunkyard::Sighting> seen;
    for(const auto &s:recent)if(s.bearing_error<=1.5*best)seen.push_back(s);
    if(seen.size()<2)return std::nullopt;
    const double n=seen.size();double t0=0,spread=0;
    for(const auto &s:seen)t0+=s.tick/n;
    for(const auto &s:seen)spread+=(s.tick-t0)*(s.tick-t0);
    double now[3],slope[3];
    for(int axis=0;axis<3;axis++) {
      const auto at=[axis](const clunkyard::Sighting &s){return axis==0?s.position.x:axis==1?s.position.y:s.position.z;};
      double mean=0,along=0;
      for(const auto &s:seen)mean+=at(s)/n;
      for(const auto &s:seen)along+=(s.tick-t0)*(at(s)-mean);
      slope[axis]=along/spread;now[axis]=mean+slope[axis]*(tick-t0);
    }
    const double rate=match.tick_rate();
    return Fit{{now[0],now[1],now[2]},{slope[0]*rate,slope[1]*rate,slope[2]*rate},(int)seen.size()};
  }
  double steer(double towards,double current) { return angles.clamp(angles.angle_diff(towards,current)/45); }
  void on_start() override { free_flight=match.is_3d()&&arena.gravity().z==0; }
  void on_turn() override {
    const auto position=me.position();const auto heading=me.heading(),pitch=me.pitch();
    drive.forces({}, {}, {}, {}, {}, {}, 0.2);
    const auto target=sensors.nearest();const auto cue=tracker.cue();
    const auto last_seen=slot?tracker.last_seen(slot):std::nullopt;const auto tick=clock.tick();
    if(target) {
      slot=target->slot;
      const auto throttle=target->range>350?0.7:target->range<200?-0.4:0.2;
      drive.set(throttle,steer(target->bearing,heading),steer(target->elevation,pitch));
      // Halve the beam each tick until it is HOLD_WIDTH across at the target:
      // narrow enough for a precise bearing, wide enough to keep it in.
      const auto hold=std::max(MIN_ARC,2*std::atan2(HOLD_WIDTH/2,target->range)*180/PI);
      // In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks, widen it
      // for a tick: a nearer robot it finds becomes the target.
      const auto arc=tick%LOOK_EVERY==0?SEARCH_ARC:std::max(hold,std::min(SEARCH_ARC,scanner.last().arc/2));
      scanner.set(target->bearing,arc,target->elevation);
      const auto power=gun.power_for_flight_time(target->range,1);
      // An exact contact: gun.lead is the intercept if the target holds course.
      // A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
      std::optional<clunkyard::Solution> aim;bool ready=true;
      if(target->bearing_error>0) {
        const auto line=fit(target->slot,tick);
        if(line)aim=gun.lead_point(line->position,line->velocity,power);
        ready=line&&aim&&line->count>=SIGHTINGS;
      } else aim=gun.lead(*target,power);
      const auto towards=aim?aim->heading:target->bearing,up=aim?aim->pitch:target->elevation;
      gun.aim(towards,up);const auto now=gun.aim_now();
      if(ready&&std::abs(angles.angle_diff(towards,now.heading))<3&&std::abs(up-now.pitch)<3&&gun.can_fire(power,10))gun.fire(power);
      status.set(ready?"TRACK":"NARROW",target->range);
    } else if(last_seen&&tick-*last_seen<=REACQUIRE) {
      // Lost it: look again where it was last seen, widening the beam, before searching.
      const auto where=*tracker.position(slot);
      const auto bearing=angles.bearing_to(position,where),elevation=free_flight?angles.elevation_to(position,where):0;
      drive.set(0.5,steer(bearing,heading),steer(elevation,pitch));
      scanner.set(bearing,std::min(SEARCH_ARC,scanner.last().arc*2),elevation);
      status.set("REACQUIRE");
    } else if(cue) {
      const auto bearing=angles.bearing_to(position,cue->position),distance=angles.distance(position,cue->position);
      const auto elevation=free_flight?angles.elevation_to(position,cue->position):0;
      const auto error=std::min(cue->bearing_error,30.0);
      const auto across=(tick%5-2)/2.0*error,up=free_flight?((tick/5)%3-1)*error/2:0;
      drive.set(0.7,steer(bearing,heading),steer(elevation,pitch));
      scanner.set(bearing+across,scanner.arc_for_reach(distance+100),elevation+up);
      status.set("CUE",distance);
    } else {
      drive.set(0.5,0.33,steer(0,pitch));
      scanner.set(angles.normalize(scanner.last().heading+36),SEARCH_ARC,free_flight?((tick/10)%3-1)*35:0);
      status.set("SEARCH");
    }
    // A simple dodge: vary it, or predict the shooter.
    bool fired=false;for(const auto &heard:sensors.cues())fired|=heard.source==CY_SOURCE_MUZZLE;
    if(dodge_ticks==0&&fired){dodges++;dodge_ticks=10;}
    if(dodge_ticks>0) {
      // Odd dodges go left, even ones right; turn +1 and strafe -1 go left.
      const bool left=dodges%2==1;dodge_ticks--;
      if(match.control()==CY_CONTROL_FORCES){drive.set(0.5);drive.forces({},left?-1:1);}
      else drive.set(1,left?1:-1);
      status.set(left?"dodge:left":"dodge:right",dodges);
    }
    if(battery.charge()<battery.capacity()/4)drive.set(drive.get_throttle().value_or(0)*0.5);
    if(const auto storm=nav.storm();storm&&storm->margin(position).value_or(-INFINITY)>-60) {
      const auto away=angles.bearing_to(storm->center,position);
      drive.set(0.8,angles.clamp(angles.angle_diff(away,heading)/30));
    }
    if(const auto zone=nav.zone();zone&&zone->margin(position).value_or(INFINITY)<60) {
      const auto bearing=angles.bearing_to(position,zone->center),climb=zone->shape==CY_SHAPE_SPHERE?angles.elevation_to(position,zone->center):0;
      drive.set(0.8,angles.clamp(angles.angle_diff(bearing,heading)/30),angles.clamp((climb-pitch)/30));
    }
  }
};
int main() { Starter robot;robot.run(); }
