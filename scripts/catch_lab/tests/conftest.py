import sys
from pathlib import Path

# tests.synth, catch_lab 둘 다 import 되게
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
