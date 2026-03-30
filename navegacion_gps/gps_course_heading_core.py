from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Optional

from navegacion_gps.heading_math import (
    circular_mean_deg,
    normalize_yaw_deg,
    shortest_angular_distance_deg,
)


@dataclass(frozen=True)
class GpsFixSample:
    lat: float
    lon: float
    stamp_s: float


@dataclass(frozen=True)
class CourseHeadingEstimate:
    valid: bool
    reason: str
    yaw_deg: Optional[float]
    distance_m: float
    speed_mps: float
    steer_deg: Optional[float]
    yaw_rate_rps: float
    latest_fix_age_s: Optional[float]
    sample_dt_s: Optional[float]
    candidate_count: int
    heading_dispersion_deg: Optional[float]
    mean_yaw_deg: Optional[float]


@dataclass(frozen=True)
class HeadingCandidate:
    yaw_deg: float
    distance_m: float
    sample_dt_s: float
    speed_error_mps: float


@dataclass(frozen=True)
class HeadingOffsetCompensation:
    yaw_deg: float
    correction_deg: float
    vx_antenna_mps: float
    vy_antenna_mps: float


def compensate_heading_for_sensor_offset(
    *,
    antenna_yaw_deg: float,
    speed_mps: float,
    yaw_rate_rps: float,
    offset_x_m: float,
    offset_y_m: float,
) -> HeadingOffsetCompensation:
    vx_antenna_mps = float(speed_mps) - (float(yaw_rate_rps) * float(offset_y_m))
    vy_antenna_mps = float(yaw_rate_rps) * float(offset_x_m)
    correction_deg = math.degrees(math.atan2(vy_antenna_mps, vx_antenna_mps))
    return HeadingOffsetCompensation(
        yaw_deg=normalize_yaw_deg(float(antenna_yaw_deg) - correction_deg),
        correction_deg=float(correction_deg),
        vx_antenna_mps=float(vx_antenna_mps),
        vy_antenna_mps=float(vy_antenna_mps),
    )


def ll_delta_to_north_east_m(
    lat: float,
    lon: float,
    ref_lat: float,
    ref_lon: float,
) -> tuple[float, float]:
    meters_per_deg_lat = 111_320.0
    cos_lat = max(1.0e-6, abs(math.cos(math.radians(float(ref_lat)))))
    meters_per_deg_lon = meters_per_deg_lat * cos_lat
    north_m = (float(lat) - float(ref_lat)) * meters_per_deg_lat
    east_m = (float(lon) - float(ref_lon)) * meters_per_deg_lon
    return north_m, east_m


def ros_yaw_deg_from_north_east(north_m: float, east_m: float) -> float:
    return normalize_yaw_deg(math.degrees(math.atan2(float(north_m), float(east_m))))


class GpsCourseHeadingEstimator:
    def __init__(
        self,
        *,
        min_distance_m: float = 2.5,
        min_speed_mps: float = 0.8,
        max_abs_steer_deg: float = 6.0,
        max_abs_yaw_rate_rps: float = 0.12,
        max_fix_age_s: float = 0.5,
        sample_dt_min_s: float = 0.05,
        sample_dt_max_s: float = 4.0,
        max_pair_distance_base_m: float = 0.10,
        max_pair_distance_speed_gain: float = 1.5,
        max_pair_speed_error_mps: float = 0.75,
        heading_change_base_deg: float = 3.0,
        heading_change_yaw_rate_gain: float = 1.0,
        candidates: int = 5,
        max_heading_dispersion_deg: float = 4.0,
        history_window_s: float = 12.0,
    ) -> None:
        self.min_distance_m = max(0.01, float(min_distance_m))
        self.min_speed_mps = max(0.0, float(min_speed_mps))
        self.max_abs_steer_deg = max(0.0, float(max_abs_steer_deg))
        self.max_abs_yaw_rate_rps = max(0.0, float(max_abs_yaw_rate_rps))
        self.max_fix_age_s = max(0.01, float(max_fix_age_s))
        self.sample_dt_min_s = max(1.0e-3, float(sample_dt_min_s))
        self.sample_dt_max_s = max(self.sample_dt_min_s, float(sample_dt_max_s))
        self.max_pair_distance_base_m = max(0.0, float(max_pair_distance_base_m))
        self.max_pair_distance_speed_gain = max(0.0, float(max_pair_distance_speed_gain))
        self.max_pair_speed_error_mps = max(0.0, float(max_pair_speed_error_mps))
        self.heading_change_base_deg = max(0.0, float(heading_change_base_deg))
        self.heading_change_yaw_rate_gain = max(0.0, float(heading_change_yaw_rate_gain))
        self.candidates = max(3, int(candidates))
        self.max_heading_dispersion_deg = max(0.0, float(max_heading_dispersion_deg))
        self.history_window_s = max(self.max_fix_age_s, float(history_window_s))
        self._fixes: Deque[GpsFixSample] = deque()

    def add_fix(self, lat: float, lon: float, stamp_s: float) -> None:
        if not (
            math.isfinite(float(lat))
            and math.isfinite(float(lon))
            and math.isfinite(float(stamp_s))
        ):
            return
        sample = GpsFixSample(lat=float(lat), lon=float(lon), stamp_s=float(stamp_s))
        self._fixes.append(sample)
        self._trim_history(now_s=sample.stamp_s)

    def estimate(
        self,
        *,
        now_s: float,
        speed_mps: float,
        steer_deg: Optional[float],
        steer_valid: bool,
        yaw_rate_rps: float,
    ) -> CourseHeadingEstimate:
        if not math.isfinite(float(now_s)):
            return self._invalid(
                reason="invalid_clock",
                speed_mps=speed_mps,
                steer_deg=steer_deg,
                yaw_rate_rps=yaw_rate_rps,
                latest_fix_age_s=None,
            )

        self._trim_history(now_s=float(now_s))
        if not self._fixes:
            return self._invalid(
                reason="no_fix",
                speed_mps=speed_mps,
                steer_deg=steer_deg,
                yaw_rate_rps=yaw_rate_rps,
                latest_fix_age_s=None,
            )

        latest = self._fixes[-1]
        latest_fix_age_s = max(0.0, float(now_s) - latest.stamp_s)
        if latest_fix_age_s > self.max_fix_age_s:
            return self._invalid(
                reason="stale_fix",
                speed_mps=speed_mps,
                steer_deg=steer_deg,
                yaw_rate_rps=yaw_rate_rps,
                latest_fix_age_s=latest_fix_age_s,
            )

        if not math.isfinite(float(speed_mps)) or float(speed_mps) < self.min_speed_mps:
            return self._invalid(
                reason="speed_below_threshold",
                speed_mps=speed_mps,
                steer_deg=steer_deg,
                yaw_rate_rps=yaw_rate_rps,
                latest_fix_age_s=latest_fix_age_s,
            )

        if not steer_valid or steer_deg is None or not math.isfinite(float(steer_deg)):
            return self._invalid(
                reason="steer_invalid",
                speed_mps=speed_mps,
                steer_deg=steer_deg,
                yaw_rate_rps=yaw_rate_rps,
                latest_fix_age_s=latest_fix_age_s,
            )

        if abs(float(steer_deg)) > self.max_abs_steer_deg:
            return self._invalid(
                reason="steer_too_high",
                speed_mps=speed_mps,
                steer_deg=steer_deg,
                yaw_rate_rps=yaw_rate_rps,
                latest_fix_age_s=latest_fix_age_s,
            )

        if not math.isfinite(float(yaw_rate_rps)) or abs(float(yaw_rate_rps)) > self.max_abs_yaw_rate_rps:
            return self._invalid(
                reason="yaw_rate_too_high",
                speed_mps=speed_mps,
                steer_deg=steer_deg,
                yaw_rate_rps=yaw_rate_rps,
                latest_fix_age_s=latest_fix_age_s,
            )

        raw_candidates: list[HeadingCandidate] = []
        saw_sample_before_latest = False
        saw_dt_in_range = False
        saw_pair_distance_too_high = False
        saw_pair_speed_inconsistent = False
        saw_heading_change_too_high = False
        expected_speed_mps = abs(float(speed_mps))
        for sample in self._fixes:
            if sample.stamp_s >= latest.stamp_s:
                continue
            saw_sample_before_latest = True
            sample_dt_s = float(latest.stamp_s - sample.stamp_s)
            if (
                (not math.isfinite(sample_dt_s))
                or sample_dt_s < self.sample_dt_min_s
                or sample_dt_s > self.sample_dt_max_s
            ):
                continue
            saw_dt_in_range = True
            north_m, east_m = ll_delta_to_north_east_m(
                lat=latest.lat,
                lon=latest.lon,
                ref_lat=sample.lat,
                ref_lon=sample.lon,
            )
            distance_m = math.hypot(north_m, east_m)
            if distance_m < self.min_distance_m:
                continue
            max_pair_distance_m = (
                self.max_pair_distance_base_m
                + self.max_pair_distance_speed_gain * expected_speed_mps * sample_dt_s
            )
            if distance_m > max_pair_distance_m:
                saw_pair_distance_too_high = True
                continue
            gps_speed_pair_mps = distance_m / max(sample_dt_s, 1.0e-6)
            speed_error_mps = abs(gps_speed_pair_mps - expected_speed_mps)
            if speed_error_mps > self.max_pair_speed_error_mps:
                saw_pair_speed_inconsistent = True
                continue
            yaw_deg = ros_yaw_deg_from_north_east(north_m=north_m, east_m=east_m)
            raw_candidates.append(
                HeadingCandidate(
                    yaw_deg=float(yaw_deg),
                    distance_m=float(distance_m),
                    sample_dt_s=float(sample_dt_s),
                    speed_error_mps=float(speed_error_mps),
                )
            )

        if not raw_candidates:
            if saw_sample_before_latest and not saw_dt_in_range:
                reason = "sample_dt_out_of_range"
            elif saw_pair_distance_too_high:
                reason = "pair_distance_too_high"
            elif saw_pair_speed_inconsistent:
                reason = "pair_speed_inconsistent"
            else:
                reason = "distance_below_threshold"
            return self._invalid(
                reason=reason,
                speed_mps=speed_mps,
                steer_deg=steer_deg,
                yaw_rate_rps=yaw_rate_rps,
                latest_fix_age_s=latest_fix_age_s,
            )

        raw_candidates.sort(key=lambda item: (item.sample_dt_s, item.speed_error_mps))
        anchor = raw_candidates[0]
        filtered_candidates: list[HeadingCandidate] = [anchor]
        for candidate in raw_candidates[1:]:
            max_heading_change_deg = self.heading_change_base_deg + (
                self.heading_change_yaw_rate_gain
                * math.degrees(abs(float(yaw_rate_rps)) * candidate.sample_dt_s)
            )
            heading_delta_deg = abs(
                shortest_angular_distance_deg(anchor.yaw_deg, candidate.yaw_deg)
            )
            if heading_delta_deg > max_heading_change_deg:
                saw_heading_change_too_high = True
                continue
            filtered_candidates.append(candidate)

        filtered_candidates.sort(key=lambda item: (item.speed_error_mps, item.sample_dt_s))
        selected_candidates = filtered_candidates[: self.candidates]
        candidate_count = len(selected_candidates)
        if candidate_count < 3:
            return self._invalid(
                reason=(
                    "heading_change_too_high"
                    if saw_heading_change_too_high and len(raw_candidates) >= 3
                    else "insufficient_heading_candidates"
                ),
                speed_mps=speed_mps,
                steer_deg=steer_deg,
                yaw_rate_rps=yaw_rate_rps,
                latest_fix_age_s=latest_fix_age_s,
                candidate_count=candidate_count,
            )

        mean_yaw_deg = circular_mean_deg([item.yaw_deg for item in selected_candidates])
        if mean_yaw_deg is None:
            return self._invalid(
                reason="heading_dispersion_too_high",
                speed_mps=speed_mps,
                steer_deg=steer_deg,
                yaw_rate_rps=yaw_rate_rps,
                latest_fix_age_s=latest_fix_age_s,
                candidate_count=candidate_count,
            )
        heading_dispersion_deg = max(
            abs(shortest_angular_distance_deg(mean_yaw_deg, item.yaw_deg))
            for item in selected_candidates
        )
        if heading_dispersion_deg > self.max_heading_dispersion_deg:
            return self._invalid(
                reason="heading_dispersion_too_high",
                speed_mps=speed_mps,
                steer_deg=steer_deg,
                yaw_rate_rps=yaw_rate_rps,
                latest_fix_age_s=latest_fix_age_s,
                candidate_count=candidate_count,
                heading_dispersion_deg=heading_dispersion_deg,
                mean_yaw_deg=mean_yaw_deg,
            )

        primary_candidate = min(
            selected_candidates, key=lambda item: (item.speed_error_mps, item.sample_dt_s)
        )
        return CourseHeadingEstimate(
            valid=True,
            reason="ok",
            yaw_deg=float(mean_yaw_deg),
            distance_m=float(primary_candidate.distance_m),
            speed_mps=float(speed_mps),
            steer_deg=float(steer_deg),
            yaw_rate_rps=float(yaw_rate_rps),
            latest_fix_age_s=float(latest_fix_age_s),
            sample_dt_s=float(primary_candidate.sample_dt_s),
            candidate_count=candidate_count,
            heading_dispersion_deg=float(heading_dispersion_deg),
            mean_yaw_deg=float(mean_yaw_deg),
        )

    def _trim_history(self, *, now_s: float) -> None:
        threshold_s = float(now_s) - self.history_window_s
        while self._fixes and self._fixes[0].stamp_s < threshold_s:
            self._fixes.popleft()

    @staticmethod
    def _invalid(
        *,
        reason: str,
        speed_mps: float,
        steer_deg: Optional[float],
        yaw_rate_rps: float,
        latest_fix_age_s: Optional[float],
        candidate_count: int = 0,
        heading_dispersion_deg: Optional[float] = None,
        mean_yaw_deg: Optional[float] = None,
    ) -> CourseHeadingEstimate:
        return CourseHeadingEstimate(
            valid=False,
            reason=str(reason),
            yaw_deg=None,
            distance_m=0.0,
            speed_mps=float(speed_mps) if math.isfinite(float(speed_mps)) else 0.0,
            steer_deg=(
                float(steer_deg)
                if steer_deg is not None and math.isfinite(float(steer_deg))
                else None
            ),
            yaw_rate_rps=(
                float(yaw_rate_rps) if math.isfinite(float(yaw_rate_rps)) else 0.0
            ),
            latest_fix_age_s=latest_fix_age_s,
            sample_dt_s=None,
            candidate_count=max(0, int(candidate_count)),
            heading_dispersion_deg=(
                float(heading_dispersion_deg)
                if heading_dispersion_deg is not None
                and math.isfinite(float(heading_dispersion_deg))
                else None
            ),
            mean_yaw_deg=(
                float(mean_yaw_deg)
                if mean_yaw_deg is not None and math.isfinite(float(mean_yaw_deg))
                else None
            ),
        )
