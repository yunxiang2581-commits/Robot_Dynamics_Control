# mink Capability vs A Requirements

## Current Interpretation

mink provides a mature implementation. Project A only uses mink as a structural reference.

## Capability Mapping

| mink capability | A requirement | Current state |
|---|---|---|
| configuration / frame pose | A02 site pose | foundation validation retained |
| Jacobian-based tasks | A03/A04/A05 | A03 retained; A04/A05 wrappers |
| FrameTask / PostureTask | IK interface | TODO skeleton |
| limits | IK interface bounds | TODO skeleton |
| mocap viewer target | Viewer interface | TODO skeleton |
| actuator execution | Actuator interface | TODO skeleton |

## Requirement Boundary

A 项目需要自己实现可解释的学习版接口，不直接调用 mink 替代实现。

## Next Requirement

R1: make TargetDefinition load/save/validate reliable enough for A06 output and A05 input.
