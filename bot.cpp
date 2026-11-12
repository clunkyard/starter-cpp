// A robot on the Clunkyard core's C++ binding. The core owns tracking,
// geometry and protocol handling; these callbacks choose the tactics.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range)
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
#include "clunkyard.hpp"
#include <cmath>
#include <cstdint>

class Starter : public clunkyard::Robot {
  bool free_flight=false;
  double steer(double towards,double current) { return angles.clamp(angles.angle_diff(towards,current)/45); }
  void on_start() override { free_flight=match.is_3d()&&arena.gravity().z==0; }
  void on_turn() override {
    const auto position=me.position();const auto heading=me.heading(),pitch=me.pitch();
    drive.forces({}, {}, {}, {}, {}, {}, 0.2);
    const auto target=sensors.nearest();const auto cue=tracker.cue();
    if(target) {
      const auto throttle=target->range>350?0.7:target->range<200?-0.4:0.2;
      drive.set(throttle,steer(target->bearing,heading),steer(target->elevation,pitch));
      scanner.set(target->bearing,20,target->elevation);
      const auto power=gun.power_for_flight_time(target->range,1);const auto aim=gun.lead(*target,power);
      const auto towards=aim?aim->heading:target->bearing,up=aim?aim->pitch:target->elevation;
      gun.aim(towards,up);const auto now=gun.aim_now();
      if(std::abs(angles.angle_diff(towards,now.heading))<3&&std::abs(up-now.pitch)<3&&gun.can_fire(power,10))gun.fire(power);
      status.set("TRACK",target->range);
    } else if(cue) {
      const auto bearing=angles.bearing_to(position,cue->position),distance=angles.distance(position,cue->position);
      const auto elevation=free_flight?angles.elevation_to(position,cue->position):0;
      const auto error=std::min(cue->bearing_error,30.0);const auto tick=clock.tick();
      const auto across=(tick%5-2)/2.0*error,up=free_flight?((tick/5)%3-1)*error/2:0;
      drive.set(0.7,steer(bearing,heading),steer(elevation,pitch));
      scanner.set(bearing+across,scanner.arc_for_reach(distance+100),elevation+up);
      status.set("CUE",distance);
    } else {
      drive.set(0.5,0.33,steer(0,pitch));
      scanner.set(angles.normalize(scanner.last().heading+36),40,free_flight?((clock.tick()/10)%3-1)*35:0);
      status.set("SEARCH");
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
