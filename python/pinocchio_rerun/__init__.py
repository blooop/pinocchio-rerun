try:
	from .pinocchio_rerun_pywrap import *  # type: ignore  # noqa: F401,F403
	from .pinocchio_rerun_pywrap import __version__  # type: ignore  # noqa: F401
except ImportError:  # pragma: no cover
	__version__ = "0.0.0"
