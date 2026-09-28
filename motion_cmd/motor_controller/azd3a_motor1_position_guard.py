"""Standard Motor 1 entry point; implementation remains compatibility-safe."""

from .azd3a_axis1_command_guard import main

__all__ = ["main"]


if __name__ == "__main__":
    main()
