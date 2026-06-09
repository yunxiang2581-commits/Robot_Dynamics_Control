# Project E Host Readiness Report

## OS
- kernel: `uname_result(system='Linux', node='ubuntu-Tower', release='6.17.0-35-generic', version='#35-Ubuntu SMP PREEMPT_DYNAMIC Tue May 26 13:10:28 UTC 2026', machine='x86_64')`

```text
PRETTY_NAME="Ubuntu 25.10"
NAME="Ubuntu"
VERSION_ID="25.10"
VERSION="25.10 (Questing Quokka)"
VERSION_CODENAME=questing
ID=ubuntu
ID_LIKE=debian
HOME_URL="https://www.ubuntu.com/"
SUPPORT_URL="https://help.ubuntu.com/"
BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
UBUNTU_CODENAME=questing
LOGO=ubuntu-logo
```

## Python
- executable: `/usr/bin/python3`
- version: `3.13.7`

## Container Runtimes
- docker: `/usr/bin/docker`
- docker version: `Docker version 29.4.1, build 055a478`
- apptainer: `missing`
- apptainer version: `not found in PATH`
- singularity: `missing`
- singularity version: `not found in PATH`

## GPU Tooling
- nvcc: `missing`
- nvcc version: `not found in PATH`

### nvidia-smi Preview
```text
Fri Jun  5 19:27:41 2026       
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 595.71.05              Driver Version: 595.71.05      CUDA Version: 13.2     |
+-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
|                                         |                        |               MIG M. |
|=========================================+========================+======================|
|   0  NVIDIA GeForce RTX 3090        Off |   00000000:02:00.0  On |                  N/A |
| 30%   40C    P8             39W /  350W |     456MiB /  24576MiB |     38%      Default |
|                                         |                        |                  N/A |
+-----------------------------------------+------------------------+----------------------+
```

## Display And Device Environment
- DISPLAY: `:0.0`
- NVIDIA_VISIBLE_DEVICES: `unset`
- CUDA_VISIBLE_DEVICES: `unset`

## Summary
- This report is read-only and does not install or start anything.
- Use it to judge whether container-first reproduction is plausible on the current host.
