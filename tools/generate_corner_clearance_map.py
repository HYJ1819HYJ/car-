#!/usr/bin/env python3
"""Host CLI for the same corner generator used automatically by navigation."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/maze_bridge/src'))
from maze_bridge.corner_clearance import detect_corner_regions, detect_wall_end_regions, generate, main

if __name__ == '__main__':
    main()
