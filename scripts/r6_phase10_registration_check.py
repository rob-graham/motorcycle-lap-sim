"""Reproducible Phase 10 Mallala R6 multi-lap registration diagnostic."""

import argparse
import csv
import math
from pathlib import Path

import numpy as np

from motorcycle_lap_sim.telemetry import (
    chainage_progress_diagnostics,
    cross_lap_envelope,
    fit_rigid_registration,
    gps_quality_mask,
    lap_slices,
    load_aim_gps_quality,
    load_aim_workbook,
    peer_trajectory_deviation,
    require_time_alignment,
)
from motorcycle_lap_sim.telemetry.map_match import Rigid2DTransform, map_match_nearest
from motorcycle_lap_sim.track import Track, sample_track, sample_track_stations


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("workbook", type=Path)
    parser.add_argument("--track", type=Path,
                        default=Path("examples/tracks/mallala_reference.yaml"))
    parser.add_argument("--telemetry-sheet", default="Updated")
    parser.add_argument("--quality-sheet", default="R6MallalaP4")
    parser.add_argument("--track-spacing-m", type=float, default=0.5)
    parser.add_argument("--peer-retain-fraction", type=float, default=0.95)
    parser.add_argument("--trim-fraction", type=float, default=0.85)
    parser.add_argument("--max-iterations", type=int, default=120)
    parser.add_argument("--backward-tolerance-m", type=float, default=0.25)
    parser.add_argument("--heading-bin-deg", type=float, default=30.0)
    parser.add_argument("--min-satellites", type=float, default=None)
    parser.add_argument("--max-position-accuracy-m", type=float, default=None)
    parser.add_argument("--max-speed-accuracy-mps", type=float, default=None)
    parser.add_argument("--envelope-csv", type=Path, default=None)
    return parser


def _interior_positive_laps(session):
    """Return positive lap-ID runs bounded by data on both sides."""
    count = len(session.time_s)
    return tuple(lap for lap in lap_slices(session)
                 if lap.start_index > 0 and lap.stop_index < count)


def _normalise_angle(angle_rad):
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def _initial_transform(session, laps, track):
    starts = np.asarray([lap.start_index for lap in laps], dtype=int)
    headings = np.asarray(session.heading_rad[starts], dtype=float)
    if not np.all(np.isfinite(headings)):
        raise ValueError("lap-start GPS heading must be finite for automatic initial registration")
    mean_heading = math.atan2(float(np.mean(np.sin(headings))),
                              float(np.mean(np.cos(headings))))
    bearing = _normalise_angle(mean_heading + track.start_pose.heading_rad)
    return Rigid2DTransform(
        float(np.median(session.east_m[starts])),
        float(np.median(session.north_m[starts])),
        bearing,
    )


def _pooled_indices(laps):
    return np.concatenate([
        np.arange(lap.start_index, lap.stop_index, dtype=int) for lap in laps
    ])


def _peer_mask(peer_values, retain_fraction):
    if not math.isfinite(retain_fraction) or not 0.5 <= retain_fraction <= 1.0:
        raise ValueError("peer-retain-fraction must be between 0.5 and 1.0")
    values = np.asarray(peer_values, dtype=float)
    if retain_fraction == 1.0:
        threshold = float(np.max(values))
        return np.ones(len(values), dtype=bool), threshold
    threshold = float(np.quantile(values, retain_fraction))
    return values <= threshold, threshold


def _print_heading_summary(headings_rad, peer_deviation_m, quality, indices, bin_width_deg):
    if not math.isfinite(bin_width_deg) or bin_width_deg <= 0 or bin_width_deg > 360:
        raise ValueError("heading-bin-deg must be finite and in (0, 360]")
    headings_deg = np.mod(np.degrees(np.asarray(headings_rad, dtype=float)), 360.0)
    peer = np.asarray(peer_deviation_m, dtype=float)
    edges = np.arange(0.0, 360.0 + bin_width_deg, bin_width_deg)
    if edges[-1] < 360.0:
        edges = np.r_[edges, 360.0]
    print("heading_peer_deviation_summary:")
    for lower, upper in zip(edges[:-1], edges[1:]):
        selected = (headings_deg >= lower) & (headings_deg < upper)
        if not np.any(selected):
            continue
        source = indices[selected]
        print(
            f"  heading_deg={lower:.1f}:{upper:.1f} count={np.count_nonzero(selected)} "
            f"peer_median_m={np.median(peer[selected]):.6f} "
            f"peer_p95_m={np.percentile(peer[selected], 95.0):.6f} "
            f"nsat_median={np.nanmedian(quality.satellites[source]):.3f} "
            f"position_accuracy_median_m={np.nanmedian(quality.position_accuracy_m[source]):.9f} "
            f"speed_accuracy_median_mps={np.nanmedian(quality.speed_accuracy_mps[source]):.9f}")


def _circular_flag_runs(mask):
    """Return contiguous true index runs, merging a closed-track seam run."""
    values = np.asarray(mask, dtype=bool)
    if values.ndim != 1:
        raise ValueError("sector mask must be one-dimensional")
    indices = np.flatnonzero(values)
    if len(indices) == 0:
        return ()
    if np.all(values):
        return (indices,)

    breaks = np.flatnonzero(np.diff(indices) > 1)
    runs = [run for run in np.split(indices, breaks + 1) if len(run)]
    if values[0] and values[-1] and len(runs) > 1:
        runs[0] = np.concatenate((runs[-1], runs[0]))
        runs.pop()
    return tuple(runs)


def _bin_run_edges(chainage_m, run, total_length_m):
    chainage = np.asarray(chainage_m, dtype=float)
    indices = np.asarray(run, dtype=int)
    start = int(indices[0])
    stop = int(indices[-1])
    if start <= stop:
        lower = 0.0 if start == 0 else 0.5 * (chainage[start - 1] + chainage[start])
        upper = total_length_m if stop == len(chainage) - 1 else 0.5 * (chainage[stop] + chainage[stop + 1])
        return float(lower), float(upper)

    # Wrapped run: report the interval as high-chainage:start then 0:low-chainage.
    lower = 0.5 * (chainage[start - 1] + chainage[start])
    upper = 0.5 * (chainage[stop] + chainage[stop + 1])
    return float(lower), float(upper)


def _require_converged_registration(result):
    if not result.converged:
        raise RuntimeError(
            "rigid registration did not converge; refusing downstream map matching and CSV output")


def _corridor_diagnostics(track, offset_envelope, eligible_mask):
    """Compare complete-lap envelope evidence with nominal model widths."""
    reference = sample_track_stations(track, offset_envelope.chainage_m)
    eligible = np.asarray(eligible_mask, dtype=bool)
    if eligible.shape != offset_envelope.chainage_m.shape:
        raise ValueError("corridor eligibility mask must match envelope bins")

    median = offset_envelope.median
    p10 = offset_envelope.p10
    p90 = offset_envelope.p90
    finite = np.isfinite(median) & np.isfinite(p10) & np.isfinite(p90)
    eligible &= finite

    median_excess = np.full_like(median, np.nan, dtype=float)
    median_excess[eligible] = np.maximum.reduce((
        median[eligible] - reference.width_left_m[eligible],
        -reference.width_right_m[eligible] - median[eligible],
        np.zeros(np.count_nonzero(eligible), dtype=float),
    ))
    percentile_touches_outside = np.zeros(len(median), dtype=bool)
    percentile_fully_outside = np.zeros(len(median), dtype=bool)
    percentile_touches_outside[eligible] = (
        (p90[eligible] > reference.width_left_m[eligible])
        | (p10[eligible] < -reference.width_right_m[eligible]))
    percentile_fully_outside[eligible] = (
        (p10[eligible] > reference.width_left_m[eligible])
        | (p90[eligible] < -reference.width_right_m[eligible]))
    median_outside = eligible & (median_excess > 0.0)

    eligible_count = int(np.count_nonzero(eligible))
    print(f"corridor_eligible_bins={eligible_count}/{len(median)}")
    print(f"median_outside_model_corridor_bins={np.count_nonzero(median_outside)}/{eligible_count}")
    print(f"p10_p90_touches_outside_model_corridor_bins="
          f"{np.count_nonzero(percentile_touches_outside)}/{eligible_count}")
    print(f"p10_p90_fully_outside_model_corridor_bins="
          f"{np.count_nonzero(percentile_fully_outside)}/{eligible_count}")

    if eligible_count == 0:
        print("maximum_median_model_corridor_excess_m=not_available")
        print("maximum_median_model_corridor_excess_chainage_m=not_available")
    else:
        eligible_indices = np.flatnonzero(eligible)
        worst = int(eligible_indices[np.argmax(median_excess[eligible])])
        print(f"maximum_median_model_corridor_excess_m={median_excess[worst]:.9f}")
        print(f"maximum_median_model_corridor_excess_chainage_m={offset_envelope.chainage_m[worst]:.9f}")

    for label, mask in (
            ("median_outside_model_corridor_sector_m", median_outside),
            ("p10_p90_fully_outside_model_corridor_sector_m", percentile_fully_outside)):
        for run in _circular_flag_runs(mask):
            lower, upper = _bin_run_edges(offset_envelope.chainage_m, run, track.total_length_m)
            run_values = median_excess[run]
            local_worst = int(run[np.nanargmax(run_values)])
            print(f"{label}={lower:.3f}:{upper:.3f} bins={len(run)} "
                  f"maximum_median_excess_m={median_excess[local_worst]:.6f} "
                  f"at_chainage_m={offset_envelope.chainage_m[local_worst]:.3f}")
    print("corridor_note=only bins containing every selected lap are eligible; consistent measured offsets beyond nominal model width indicate local reference-geometry mismatch, not automatic rider off-track classification")
    return reference, eligible, median_excess, percentile_touches_outside


def _write_envelope_csv(path, offset_envelope, speed_envelope,
                        corridor_reference, corridor_eligible, median_excess,
                        percentile_touches_outside):
    if not np.allclose(offset_envelope.chainage_m, speed_envelope.chainage_m,
                       rtol=0.0, atol=1e-12):
        raise ValueError("offset and speed envelope chainage grids do not match")
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow((
            "chainage_m",
            "model_width_left_m", "model_width_right_m",
            "offset_lap_count", "offset_median_m", "offset_p10_m", "offset_p90_m",
            "offset_min_m", "offset_max_m", "corridor_evidence_complete",
            "median_model_corridor_excess_m", "p10_p90_touches_outside_model_corridor",
            "speed_lap_count", "speed_median_mps", "speed_p10_mps", "speed_p90_mps",
            "speed_min_mps", "speed_max_mps",
        ))
        for index, chainage in enumerate(offset_envelope.chainage_m):
            eligible = bool(corridor_eligible[index])
            writer.writerow((
                chainage,
                corridor_reference.width_left_m[index], corridor_reference.width_right_m[index],
                offset_envelope.lap_count[index], offset_envelope.median[index],
                offset_envelope.p10[index], offset_envelope.p90[index],
                offset_envelope.minimum[index], offset_envelope.maximum[index], int(eligible),
                median_excess[index] if eligible else "",
                int(percentile_touches_outside[index]) if eligible else "",
                speed_envelope.lap_count[index], speed_envelope.median[index],
                speed_envelope.p10[index], speed_envelope.p90[index],
                speed_envelope.minimum[index], speed_envelope.maximum[index],
            ))


def main(argv=None):
    args = build_parser().parse_args(argv)
    session = load_aim_workbook(args.workbook, sheet_name=args.telemetry_sheet)
    quality = load_aim_gps_quality(args.workbook, sheet_name=args.quality_sheet)
    require_time_alignment(session.time_s, quality)
    track = Track.from_yaml(args.track)
    sampled_track = sample_track(track, args.track_spacing_m)

    laps = _interior_positive_laps(session)
    if len(laps) < 2:
        raise ValueError("multi-lap registration requires at least two interior positive lap runs")
    print(f"workbook={args.workbook}")
    print(f"track={args.track}")
    print(f"track_length_m={track.total_length_m:.9f}")
    print(f"lap_ids={','.join(str(lap.lap_id) for lap in laps)}")
    print(f"lap_count={len(laps)}")

    lap_east = [session.east_m[lap.start_index:lap.stop_index] for lap in laps]
    lap_north = [session.north_m[lap.start_index:lap.stop_index] for lap in laps]
    peer = peer_trajectory_deviation(lap_east, lap_north)
    peer_values = np.concatenate(peer.median_nearest_distance_m)
    indices = _pooled_indices(laps)
    peer_valid, peer_threshold = _peer_mask(peer_values, args.peer_retain_fraction)

    quality_full = gps_quality_mask(
        quality,
        min_satellites=args.min_satellites,
        max_position_accuracy_m=args.max_position_accuracy_m,
        max_speed_accuracy_mps=args.max_speed_accuracy_mps,
    )
    quality_valid = quality_full[indices]
    registration_valid = peer_valid & quality_valid
    print(f"peer_retain_fraction={args.peer_retain_fraction:.6f}")
    print(f"peer_threshold_m={peer_threshold:.9f}")
    print(f"peer_retained={np.count_nonzero(peer_valid)}/{len(peer_valid)}")
    print(f"quality_retained={np.count_nonzero(quality_valid)}/{len(quality_valid)}")
    print(f"registration_valid={np.count_nonzero(registration_valid)}/{len(registration_valid)}")
    if not np.any(registration_valid):
        raise ValueError("registration mask rejected all complete-lap samples")

    initial = _initial_transform(session, laps, track)
    print(f"initial_origin_east_m={initial.origin_east_m:.9f}")
    print(f"initial_origin_north_m={initial.origin_north_m:.9f}")
    print(f"initial_bearing_deg={math.degrees(initial.local_x_bearing_rad):.9f}")

    result = fit_rigid_registration(
        session.east_m[indices], session.north_m[indices], sampled_track, initial,
        valid_mask=registration_valid, trim_fraction=args.trim_fraction,
        max_iterations=args.max_iterations)
    print(f"registration_converged={str(result.converged).lower()}")
    print(f"registration_iterations={result.iterations}")
    print(f"registration_final_translation_delta_m={result.final_translation_delta_m:.9g}")
    print(f"registration_final_bearing_delta_deg={math.degrees(result.final_bearing_delta_rad):.9g}")
    _require_converged_registration(result)

    transform = result.transform
    print(f"origin_east_m={transform.origin_east_m:.9f}")
    print(f"origin_north_m={transform.origin_north_m:.9f}")
    print(f"local_x_bearing_deg={math.degrees(transform.local_x_bearing_rad):.9f}")
    print(f"registration_inliers={np.count_nonzero(result.inlier_mask)}/{len(result.inlier_mask)}")
    print(f"registration_rms_residual_m={result.rms_residual_m:.9f}")
    print(f"registration_median_residual_m={result.median_residual_m:.9f}")
    print(f"registration_p95_residual_m={result.p95_residual_m:.9f}")

    lap_chainage = []
    lap_offset = []
    lap_speed = []
    for lap, peer_lap in zip(laps, peer.median_nearest_distance_m):
        start, stop = lap.start_index, lap.stop_index
        x_m, y_m = transform.world_to_local(session.east_m[start:stop], session.north_m[start:stop])
        match = map_match_nearest(x_m, y_m, sampled_track)
        progress = chainage_progress_diagnostics(
            match.chainage_m, track.total_length_m,
            backward_tolerance_m=args.backward_tolerance_m)
        residual = match.reference_distance_m
        peer_peak = int(np.argmax(peer_lap))
        source_index = start + peer_peak
        print(
            f"lap_id={lap.lap_id} samples={stop - start} "
            f"residual_rms_m={np.sqrt(np.mean(residual ** 2)):.9f} "
            f"residual_median_m={np.median(residual):.9f} "
            f"residual_p95_m={np.percentile(residual, 95.0):.9f} "
            f"backward_steps={progress.backward_step_count} "
            f"largest_backward_m={progress.largest_backward_step_m:.9f} "
            f"net_progress_m={progress.net_progress_m:.9f} "
            f"peer_peak_m={peer_lap[peer_peak]:.9f} "
            f"peer_peak_chainage_m={match.chainage_m[peer_peak]:.9f} "
            f"peer_peak_heading_deg={math.degrees(session.heading_rad[source_index]):.6f} "
            f"peer_peak_nsat={quality.satellites[source_index]:.3f} "
            f"peer_peak_position_accuracy_m={quality.position_accuracy_m[source_index]:.9f} "
            f"peer_peak_speed_accuracy_mps={quality.speed_accuracy_mps[source_index]:.9f}")
        lap_chainage.append(match.chainage_m)
        lap_offset.append(match.lateral_offset_m)
        lap_speed.append(session.speed_mps[start:stop])

    _print_heading_summary(session.heading_rad[indices], peer_values, quality, indices,
                           args.heading_bin_deg)
    print("heading_note=heading bins are association diagnostics; track sector and rider line are confounders")

    offset_envelope = cross_lap_envelope(
        lap_chainage, lap_offset, track.total_length_m, bin_width_m=10.0)
    speed_envelope = cross_lap_envelope(
        lap_chainage, lap_speed, track.total_length_m, bin_width_m=10.0)
    complete_offset_bins = offset_envelope.lap_count == len(laps)
    complete_speed_bins = speed_envelope.lap_count == len(laps)
    print(f"envelope_bins={len(offset_envelope.chainage_m)}")
    print(f"offset_envelope_bins_with_all_laps={np.count_nonzero(complete_offset_bins)}")
    print(f"speed_envelope_bins_with_all_laps={np.count_nonzero(complete_speed_bins)}")
    corridor_reference, corridor_eligible, median_excess, percentile_touches_outside = (
        _corridor_diagnostics(track, offset_envelope, complete_offset_bins))
    if args.envelope_csv is not None:
        _write_envelope_csv(
            args.envelope_csv, offset_envelope, speed_envelope,
            corridor_reference, corridor_eligible, median_excess,
            percentile_touches_outside)
        print(f"envelope_csv={args.envelope_csv}")


if __name__ == "__main__":
    main()
