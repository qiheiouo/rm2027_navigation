# Frozen offline project references

Selected from project commit `b5645ecaf6b4575a6537a0a0a9907f4891955955`.
Per-file original paths and SHA256 are in `provenance.json`.

- `core.py`: existing ROS-independent tracker, Apache-2.0 as declared by its package.
- Two `.msg` files: wire schema read-only references; not registered with main ROS interface generation.
- Geometry contract: exact historical document, retaining original relative links. Those links refer to the source revision's directory tree, not this offline fixture folder.

Only tests load `core.py`. The controller reads public-value snapshots and does not import tracker internals. No node, launch, configuration, MPPI critic, guard or runtime package from the prior route is imported. The new experiment branch was created directly from main.
