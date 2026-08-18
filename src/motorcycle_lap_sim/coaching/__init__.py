"""Rider-facing coaching-event extraction."""

from .events import (
    CoachingEvent,
    EventDetectionConfig,
    detect_corner_regions,
    extract_coaching_events,
)

__all__ = [
    "CoachingEvent",
    "EventDetectionConfig",
    "detect_corner_regions",
    "extract_coaching_events",
]
