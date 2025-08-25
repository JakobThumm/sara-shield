# Re-export the extension module at package import (optional but convenient)
from .safety_shield_py import *  # noqa

# Or, if you prefer explicit:
# from . import safety_shield_py
# __all__ = ["safety_shield_py"]
