# rm_pursuit

`pursuit_goal_planner` validates `/perception/target_track`, transforms the
track into `map`, predicts a short horizon, and publishes a standoff pose on
`/mission/pursuit_goal` with `/mission/pursuit_goal_valid`.

It deliberately does not call Nav2 and never publishes `/cmd_vel`. The
competition mission executor decides whether a valid candidate may become a
`NavigateToPose` action. A stale target immediately invalidates the candidate.

No-hardware boundary test:

```bash
ros2 launch rm_pursuit pursuit.launch.py \
  enable_pursuit_boundary:=true use_mock_target:=true
```

The launch default is disabled. The mock target is never a claim that the
auto-aim/lower-controller target protocol is implemented.
