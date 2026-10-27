import sys
from pathlib import Path

# Let the tests import the src package without installing anything.
sys.path.insert(0, str(Path(__file__).parent))
