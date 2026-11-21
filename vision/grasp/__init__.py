"""Click-to-grasp pipeline groundwork (dry run only, no motor access anywhere in this package).

detect.py      open-vocabulary detection (YOLO-World) + outline (MobileSAM / GrabCut) + grasp yaw
locate.py      pixel -> 3D on the table plane (overview or wrist camera), reach filter via the existing FK/IK
grasp_plan.py  pure-software grasp sequence -> joint waypoints (no execution)
Import as `grasp` with ~/humanoid/vision on sys.path (same convention as handeye_plan, robot_fk).
"""
import sys
from pathlib import Path

VISION_DIR = Path(__file__).resolve().parents[1]
GRASP_DIR = Path(__file__).resolve().parent
if str(VISION_DIR) not in sys.path:      # lets the modules import robot_fk / handeye_plan / detector
    sys.path.insert(0, str(VISION_DIR))
