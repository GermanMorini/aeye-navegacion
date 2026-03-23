import math
import random
from pathlib import Path

from navegacion_gps.pixhawk_odometry import (
    DelayedMessageQueue,
    add_gaussian_noise,
    diag_covariance,
    random_walk_step,
    wrap_angle,
)


def test_gaussian_noise_is_deterministic_with_seed() -> None:
    seed = 1234
    rng_a = random.Random(seed)
    rng_b = random.Random(seed)

    sample_a = add_gaussian_noise(10.0, 0.5, rng_a)
    sample_b = add_gaussian_noise(10.0, 0.5, rng_b)

    assert math.isclose(sample_a, sample_b, rel_tol=0.0, abs_tol=1.0e-12)


def test_random_walk_step_is_deterministic_with_seed() -> None:
    seed = 99
    rng_a = random.Random(seed)
    rng_b = random.Random(seed)

    next_a = random_walk_step(
        current_value=0.0,
        sigma_per_sqrt_s=0.1,
        dt_s=0.02,
        rng=rng_a,
    )
    next_b = random_walk_step(
        current_value=0.0,
        sigma_per_sqrt_s=0.1,
        dt_s=0.02,
        rng=rng_b,
    )

    assert math.isclose(next_a, next_b, rel_tol=0.0, abs_tol=1.0e-12)


def test_delay_queue_respects_release_time() -> None:
    queue = DelayedMessageQueue()
    queue.push(release_time_s=1.0, msg="first")
    queue.push(release_time_s=2.0, msg="second")

    assert queue.pop_ready(now_s=0.9) == []
    assert queue.pop_ready(now_s=1.0) == ["first"]
    assert queue.pop_ready(now_s=1.5) == []
    assert queue.pop_ready(now_s=2.0) == ["second"]


def test_diag_covariance_writes_expected_diagonal_indices() -> None:
    covariance = diag_covariance(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

    assert len(covariance) == 36
    assert covariance[0] == 1.0
    assert covariance[7] == 2.0
    assert covariance[14] == 3.0
    assert covariance[21] == 4.0
    assert covariance[28] == 5.0
    assert covariance[35] == 6.0


def test_wrap_angle_and_noise_stay_finite_for_zero_velocity_case() -> None:
    rng = random.Random(7)
    yaw = wrap_angle(20.0 * math.pi)
    noisy = add_gaussian_noise(0.0, 0.0, rng)
    drift = random_walk_step(
        current_value=0.0,
        sigma_per_sqrt_s=0.05,
        dt_s=0.0,
        rng=rng,
    )

    assert math.isfinite(yaw)
    assert math.isfinite(noisy)
    assert math.isfinite(drift)


def test_pixhawk_defaults_are_less_confident_for_ekf_weighting() -> None:
    source_path = Path(__file__).resolve().parents[1] / "navegacion_gps" / "pixhawk_odometry.py"
    source_contents = source_path.read_text(encoding="utf-8")

    assert 'self.declare_parameter("pose_covariance_xy", 1.0)' in source_contents
    assert 'self.declare_parameter("twist_covariance_vx", 1.0)' in source_contents
    assert 'self.declare_parameter("twist_covariance_vy", 1.0)' in source_contents
