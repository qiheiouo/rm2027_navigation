set -e
source /opt/ros/humble/setup.bash
source /sdk/install/setup.bash
/sdk/build/rm_dynamic_obstacle_critic/replay_observation_consumption /out/visible/dynamic/input.bin /out/visible/dynamic/risk.jsonl
/sdk/build/rm_dynamic_obstacle_critic/replay_native_optimizer /out/visible/optimizer/input.bin /out/visible/optimizer/sdk_final_output.jsonl 
/sdk/build/rm_dynamic_obstacle_critic/replay_native_optimizer /out/visible/optimizer/input.bin /out/visible/optimizer/zero_history_output.jsonl --zero-history-each-cycle
/sdk/build/rm_dynamic_obstacle_critic/replay_native_optimizer /out/visible/optimizer/input.bin /out/visible/optimizer/weights_output.jsonl --weights-audit
/sdk/build/rm_dynamic_obstacle_critic/replay_native_optimizer /out/visible/optimizer/input.bin /out/visible/optimizer/uniform_output.jsonl --uniform-weight-probe
/sdk/build/rm_dynamic_obstacle_critic/native_safe_control_witness /out/visible/witness/input.bin /out/visible/witness/output.bin /out/visible/witness/output.jsonl
/sdk/build/rm_dynamic_obstacle_critic/replay_observation_consumption /out/nominal/dynamic/input.bin /out/nominal/dynamic/risk.jsonl
/sdk/build/rm_dynamic_obstacle_critic/replay_native_optimizer /out/nominal/optimizer/input.bin /out/nominal/optimizer/sdk_final_output.jsonl 
/sdk/build/rm_dynamic_obstacle_critic/replay_native_optimizer /out/nominal/optimizer/input.bin /out/nominal/optimizer/zero_history_output.jsonl --zero-history-each-cycle
/sdk/build/rm_dynamic_obstacle_critic/replay_native_optimizer /out/nominal/optimizer/input.bin /out/nominal/optimizer/weights_output.jsonl --weights-audit
/sdk/build/rm_dynamic_obstacle_critic/replay_native_optimizer /out/nominal/optimizer/input.bin /out/nominal/optimizer/uniform_output.jsonl --uniform-weight-probe
/sdk/build/rm_dynamic_obstacle_critic/native_safe_control_witness /out/nominal/witness/input.bin /out/nominal/witness/output.bin /out/nominal/witness/output.jsonl
