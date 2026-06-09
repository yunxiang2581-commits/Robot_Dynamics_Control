# E04 Download Plan

| Priority | Resource | Purpose | Expected Files | Download Method | Risk | Proceed? | Missing Now |
|---|---|---|---|---|---|---|---:|
| P0 | bundles/centauro/d2026_02_21_h14_m01_s10-CentauroCloopPartialUbNoWheels_FakePosTrackingEnv | Centauro rosbag visualization candidate | 38 files, 46593971 bytes expected | wget per-file best-effort against Hugging Face resolve URLs | xet / large-file transfer, transient SSL / CAS bridge failures possible | yes | 33 |
| P1 | bundles/centauro/d2026_03_07_h19_m22_s30-CentauroCloopPartialNoYawUb_FakePosTrackingEnv | Centauro eval example candidate | 37 files, 7693198 bytes expected | wget per-file best-effort against Hugging Face resolve URLs | xet / large-file transfer, transient SSL / CAS bridge failures possible | yes | 37 |
| P2 | bundles/b2w/d2026_03_28_h11_m11_s07-B2WPartialCloopWheels_FakePosTrackingEnv | B2W public candidate | 38 files, 33976540 bytes expected | wget per-file best-effort against Hugging Face resolve URLs | xet / large-file transfer, transient SSL / CAS bridge failures possible | yes | 38 |
| P3 | bundles/centauro/d2026_01_19_h14_m54_s32-CentauroCloopPartialNoYawUbPercep_FakePosTrackEnvPhaseControl | Other Centauro bundle | 38 files, 16113032 bytes expected | wget per-file best-effort against Hugging Face resolve URLs | xet / large-file transfer, transient SSL / CAS bridge failures possible | yes | 38 |

## Strategy

- User override: attempt all remaining gaps across P0-P3.
- Use `wget` per file.
- Keep existing successful files.
- Record every failure with per-file reason and continue to later files.
