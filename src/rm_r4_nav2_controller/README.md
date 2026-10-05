# Optional R4 standard Controller

Default OFF (`RM_R4_BUILD_NAV2_CONTROLLER`). Requires the existing optional A08
Follow target and fixed OSQP intake. Implements the standard Nav2 Controller
interface; does not replace its server, actions or lifecycle plumbing.

The original host must bind one owned cycle before its standard compute call
and retrieve the typed result using that same token before creating an atomic
proposal. Pose/Twist and exact setPlan identity must match. Missing or failed
context produces `nav2_core::PlannerException`, with no cached velocity.

There are no subscriptions, publishers, TF lookup, costmap creation, worker,
tracker, prediction/frontend copy, MPPI or output owner. Nonzero speed-limit
requests invalidate R4 and remain unsupported in this slice; the original host
must use its native fallback, with the common owner/lease/current-admission.

This plugin has finite loading/call checks only; no production host is wired.
See [A11 contract](../../docs/dynamic_navigation/r4_standard_controller_adapter.md).
