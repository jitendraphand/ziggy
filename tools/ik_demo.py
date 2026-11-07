# SPDX-License-Identifier: Apache-2.0
"""Print IK residuals for household reach targets and a reachable-target benchmark."""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ziggy.analysis import household_reach, ik_benchmark
print("household residual mm (<5 = reached):", json.dumps(household_reach("l"), indent=1))
print("benchmark:", ik_benchmark(100))
