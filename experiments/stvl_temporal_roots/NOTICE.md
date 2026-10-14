# Source attribution

The `RootCostProbe` score equation is a minimal adaptation of Nav2 1.1.20
`nav2_mppi_controller/src/critics/cost_critic.cpp`.
Copyright (c) 2022 Samsung Research America, Alexey Budyakov.
Copyright (c) 2023 Open Navigation LLC. Licensed under Apache License 2.0.

Original source: https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critics/cost_critic.cpp

Only polygon checking shortcuts and storage-preserving accumulation differ.
Native optimization, weighting, constraints and retry remain in the installed library.
License text is in `LICENSE.Apache-2.0`.
