#!/usr/bin/env python3
"""Compatibility alias: use the same complete RTAB-Map + YOLO supervisor."""
from pathlib import Path
import os
import sys
root=Path(__file__).resolve().parents[1]
os.execv('/usr/bin/python3',['/usr/bin/python3',str(root/'scripts/run_live.py'),*sys.argv[1:]])
