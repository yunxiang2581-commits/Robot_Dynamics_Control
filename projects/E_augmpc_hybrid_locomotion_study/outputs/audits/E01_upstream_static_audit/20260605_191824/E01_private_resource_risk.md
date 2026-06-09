# E01 Private Resource Risk

## Confirmed Or Likely Private Dependencies Mentioned Upstream

| Resource Class | Why It Matters | E01 Judgment |
|---|---|---|
| NGC key / image access | IsaacSim-related container setup can require authenticated image pulls | external setup risk |
| Private robot-description repos | Some Kyon workflows explicitly depend on private git directories | confirmed documentation risk |
| AugMPCEnvs / MPCHive / EigenIPC / MPCViz clones | README references them, but they were not audited here | unresolved public dependency surface |
| Public bundle payload internals | page describes contents, but payloads were not downloaded | unresolved until later step |

## Reproduction Impact

- `Kyon` should not be selected as the first public reproduction target.
- `u24` should not be selected as the first execution route.
- Any “full training reproduction” claim would be premature before validating container contracts and missing framework repos.

## Safe Interpretation

当前公开材料足够支持“路线梳理”和“低风险第一目标选择”，但不足以支持“可以直接在本机无障碍完整复现”的结论。
