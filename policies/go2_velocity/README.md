# Go2 velocity 策略权重

训练产物来自 `unitree_rl_lab` / RSL-RL，**不上传全部中间 checkpoint**（数百 MB），只保留可用成品：

| 路径 | 说明 |
|---|---|
| `2026-09-22_11-04-32/model_11097.pt` | 当前推荐 resume/play checkpoint（~11k iter） |
| `2026-09-22_11-04-32/exported/policy.pt` | TorchScript，Isaac 直推用（`run_isaac_go2_policy.sh` 默认） |
| `2026-09-22_11-04-32/params/` | agent/env/deploy 配置快照 |
| `2026-09-22_10-23-42/model_3098.pt` | 冒烟里程碑（~3k iter） |

默认推理：

```bash
./scripts/run_isaac_go2_policy.sh
# 或
ISAAC_GO2_POLICY=$PWD/policies/go2_velocity/2026-09-22_11-04-32/exported/policy.pt \
ISAAC_GO2_DEPLOY_YAML=$PWD/policies/go2_velocity/2026-09-22_11-04-32/params/deploy.yaml \
  ./scripts/run_isaac_go2_policy.sh
```
