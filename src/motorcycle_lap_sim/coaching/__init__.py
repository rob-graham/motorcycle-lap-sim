"""Rider-facing coaching-event extraction."""

from .events import CoachingEvent, EventDetectionConfig, extract_coaching_events

__all__ = ["CoachingEvent", "EventDetectionConfig", "extract_coaching_events"]
