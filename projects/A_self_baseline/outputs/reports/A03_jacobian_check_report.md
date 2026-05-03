# A03 Site Jacobian Check Report

## Input
- q source: keyframe:home
- site name: attachment_site
- dq source: unit:shoulder_pan
- dt: 1e-06
- finite difference steps: 1000
- finite difference total time: 0.001
- source MJCF: /home/ubuntu/Robot_Dynamics_Control/shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml

## Model Dimensions
- nq: 6
- nv: 6
- nu: 6

## Velocity Mapping
- Formula: site_velocity = J(q) dq
- J_pos shape: [3, 6]
- J_rot shape: [3, 6]
- linear velocity from Jacobian: [-0.1339978254660598, 0.491999298412482, 0.0]
- linear velocity from finite difference: [-0.1342438027507331, 0.49193221746493543, 0.0]
- finite difference site displacement: [-0.0001342438027507331, 0.0004919322174649354, 0.0]
- linear velocity error: [-0.00024597728467332103, -6.708094754659388e-05, 0.0]
- linear velocity error norm: 0.0002549601500215453

## Angular Velocity
- angular velocity from Jacobian: [0.0, 0.0, 1.0]
- angular velocity from finite difference: [-2.5579578738465883e-14, -3.635869736973811e-14, 0.9999999999174406]
- angular velocity error: [-2.5579578738465883e-14, -3.635869736973811e-14, -8.255940375789805e-11]
- angular velocity error norm: 8.255941572667138e-11
- angular check status: minimal_rotation_matrix_log_check

## Error Interpretation
- Small-step velocity mapping error checks mj_jacSite API, site id, dq dimension, and velocity mapping direction.
- Large-displacement linearization error uses the initial J(q0)dq over a longer total displacement.
- A larger error at larger displacement is expected and does not by itself mean the Jacobian API is wrong.

## Multi-Step Linearization Sweep
- trace JSON: /home/ubuntu/Robot_Dynamics_Control/projects/A_self_baseline/outputs/cache/A03_multi_step_trace.json
- error figure: /home/ubuntu/Robot_Dynamics_Control/projects/A_self_baseline/outputs/figures/A03_multi_step_linearization_error.png

| fd_steps | displacement_norm | linear_error_norm | angular_error_norm |
| --- | --- | --- | --- |
| 1 | 5.099203141356328e-07 | 2.548208741719915e-07 | 3.005631947365029e-10 |
| 2 | 1.019840628366838e-06 | 5.099673401076392e-07 | 1.3452687553127028e-10 |
| 5 | 2.54960157096452e-06 | 1.2747646329124877e-06 | 1.0703837961269294e-10 |
| 10 | 5.099203141952081e-06 | 2.549624858760158e-06 | 1.1328812758044985e-10 |
| 20 | 1.0198406283696791e-05 | 5.0992024078793856e-06 | 8.23668542114791e-11 |
| 50 | 2.5496015707062347e-05 | 1.2748007617378252e-05 | 8.038506768299134e-11 |
| 100 | 5.099203139792176e-05 | 2.5496016311725294e-05 | 8.393313049221885e-11 |
| 200 | 0.00010198406266835684 | 5.099203058248237e-05 | 8.417466074513405e-11 |
| 500 | 0.0002549601544399021 | 0.000127480077756432 | 8.263163713900873e-11 |
| 1000 | 0.0005099202929448349 | 0.0002549601500215453 | 8.255941572667138e-11 |

## Boundary
- A03 checks site Jacobian velocity mapping.
- A03 does not solve IK.
- A03 does not build QP.
- A03 does not run controller or simulation loop.
