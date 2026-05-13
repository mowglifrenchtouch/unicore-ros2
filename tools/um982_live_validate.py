#!/usr/bin/env python3
# Compatibility wrapper for older integrations.
# Prefer tools/unicore_live_validate.py for new deployments.

try:
    from .unicore_live_validate import *  # type: ignore # noqa: F401,F403
    from .unicore_live_validate import main  # type: ignore
except ImportError:
    from unicore_live_validate import *  # noqa: F401,F403
    from unicore_live_validate import main


if __name__ == "__main__":
    raise SystemExit(main())
