"""Standard Motor 2 entry point; implementation remains compatibility-safe."""

from .azd3a_axis2_velocity_guard import main

__all__ = ["main"]


if __name__ == "__main__":
    main()
