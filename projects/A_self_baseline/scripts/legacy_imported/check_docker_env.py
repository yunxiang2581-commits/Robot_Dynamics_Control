from __future__ import annotations

import os
import sys


def main() -> int:
    print("Python:", sys.version)
    print("Working directory:", os.getcwd())
    print("MUJOCO_GL:", os.environ.get("MUJOCO_GL"))

    import numpy as np
    import scipy
    import matplotlib
    import pandas as pd
    import casadi
    import osqp
    import mujoco
    import pinocchio as pin

    print("numpy:", np.__version__)
    print("scipy:", scipy.__version__)
    print("matplotlib:", matplotlib.__version__)
    print("pandas:", pd.__version__)
    print("casadi:", casadi.__version__)
    print("osqp:", osqp.__version__)
    print("mujoco:", mujoco.__version__)
    print("pinocchio:", pin.__version__)
    print("Docker environment check: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
