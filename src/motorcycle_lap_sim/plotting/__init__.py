"""Plotting helpers, kept separate from numerical geometry."""

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .track_plot import plot_track

__all__ = ["plot_track"]


def __getattr__(name: str) -> Any:
    """Lazily expose plotting helpers without pre-importing executable modules."""
    if name == "plot_track":
        from .track_plot import plot_track

        return plot_track
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
