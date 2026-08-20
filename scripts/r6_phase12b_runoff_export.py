"""Compatibility entry point for the retained Mallala run-off exporter."""

from motorcycle_lap_sim.runoff.retained_export import *  # noqa: F403
from motorcycle_lap_sim.runoff.retained_export import (
    _event_set_id,
    _require_retained_acceptance_provenance,
    _require_retained_seed_counts,
)


if __name__ == "__main__":
    main()  # noqa: F405
