# rm_competition_mission

The package runs a BehaviorTree.CPP mission selector and is the sole Phase 3
owner allowed to convert mission choices into the standard Nav2
`/navigate_to_pose` action.

Priority in the default tree:

1. close the safety gate and hold;
2. return home when explicitly requested or HP is below threshold;
3. pursue a fresh validated candidate when explicitly allowed;
4. follow the configured patrol route;
5. hold.

The safe config starts disabled and contains no home or patrol coordinates.
Enabling the node is not the same as enabling motion: the operator must call
`/mission/set_mode`, and required localization, referee and chassis-authority
inputs must all be valid.

No-hardware example inputs are opt-in. They must never be used as real safety
authority.
