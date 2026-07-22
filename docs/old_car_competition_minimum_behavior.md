# Old-Car Minimum Competition Behavior

## Intended Result

The minimum old-car competition behavior is deliberately small:

```text
operator enables mission after localization review
  -> navigate between reviewed patrol poses
  -> at each pose, hold translation and rotate for a configured dwell period
  -> continue to the next pose
  -> return to the reviewed home pose on low HP or low projectile allowance
  -> cancel and hold when required state becomes invalid or stale
```

The initial field values are a 10 second patrol spin and a target chassis yaw
rate of 10 rad/s. They are implemented only in the isolated three-point test
profile, where a Nav2 `Spin` action requests 100 rad with matching candidate
Nav2 and serial limits. The profile has passed an initial field smoke test;
the requested value remains a target rather than a precise measurement of the
physical chassis trajectory.

## Current Implementation

| Behavior | Current state |
| --- | --- |
| Navigate a configured patrol route repeatedly | Implemented |
| Explicit or low-HP return home | Implemented |
| Low-projectile return home | Referee field exists; mission condition not implemented |
| Per-waypoint timed spin | Implemented in the isolated three-point candidate through Nav2 `Spin` |
| High-spin localization candidate | Old-car field A/B reported successful; still explicit, not the normal default |
| Real competition-state trigger | Mock/gate exists; lower-controller serial receive parser is missing |
| Reviewed home/patrol poses | Candidate coordinates exist; final competition-map coordinates remain deferred |

The mission sends individual `NavigateToPose` goals and then a Nav2 `Spin`
action. It does not publish velocity commands directly and does not depend on
the `waypoint_follower` wait plugin.

## Required Mission Semantics

Recommended states are:

```text
DISABLED_OR_HOLD
NAVIGATING_PATROL
DWELL_SPIN
RETURNING_HOME
```

Priority must be:

1. mission disabled or required input invalid: cancel navigation, command no
   new motion and remain held;
2. valid low-HP or low-projectile condition: leave patrol/dwell and return
   home;
3. patrol enabled: navigate, dwell/rotate, then advance the route;
4. otherwise hold.

Stale or invalid referee input must not be interpreted as low ammunition and
must not cause blind motion toward home. It closes the safety gate and requires
operator recovery.

## Velocity Ownership

`competition_mission_node` currently owns Nav2 goals, not `/cmd_vel`. It must
not publish directly to the final chassis command topic.

The candidate uses the standard Nav2 `Spin` action. Nav2, the velocity smoother
and serial transport retain the existing command ownership chain. A direct
mission publisher on `/cmd_vel` remains forbidden. Any future command arbiter
would change the topic contract and require separate no-hardware, off-ground
and field validation.

## Remaining Inputs

The lower controller must provide a versioned, bounded and testable receive
frame containing at least the competition fields used by the mission:

- current HP;
- projectile allowance;
- game progress/state;
- a sequence/timestamp or enough information for freshness;
- integrity checking and an explicit protocol version.

The serial layer publishes normalized raw state only. Range/freshness checking
remains in `rm_referee_interface`, and strategic decisions remain in the
mission package.

## Acceptance Order

1. Keep the existing patrol/spin and low-HP tests passing; add low-projectile
   and stale real-state transition tests.
2. Implement and test serial RX framing, fragmentation, corruption, timeout and reconnect with a
   pseudo-terminal while mission is disabled.
3. Re-run patrol/spin/home with normalized competition-state input and no real
   motion before enabling the chassis.
4. Configure reviewed map-frame home/patrol poses for the competition map.
5. Record achieved yaw rate, stopping behavior and localization stability when
   the candidate is used at competition speed.
6. Run a complete mission and then a match-duration soak.

Pursuit, automatic place recognition, right/dual-lidar localization and a new
OctoMap cleanup backend are not required for this minimum behavior.
