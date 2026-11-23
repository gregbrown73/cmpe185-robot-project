"""Monte Carlo localization for the CMPE 185 delivery robot."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, exp, hypot, isfinite, log, pi, sin, sqrt
from random import Random

from odometry import Pose2D, wrap_angle


@dataclass
class Particle:
    x: float
    y: float
    theta: float
    weight: float


@dataclass
class RecoveryHypothesis:
    pose: Pose2D
    log_weight: float
    measurement_score: float
    score_sum: float = 0.0
    score_count: int = 0

    def mean_measurement_score(self) -> float:
        if self.score_count > 0:
            return self.score_sum / self.score_count
        return self.measurement_score


def select_localization_pose(odometry_pose, localizer, localization_mode: str):
    """Return the pose used by navigation without changing baseline behavior."""
    if localization_mode == "mcl":
        return localizer.get_estimate()
    if localization_mode in {"odometry", "odometry_baseline"}:
        return odometry_pose
    raise ValueError(
        "localization.mode must be 'odometry_baseline', 'odometry', or 'mcl'"
    )


class StudentLocalizer:
    """Particle-filter localization with LiDAR/map weighting and global recovery.

    Recovery uses a small set of scan-match hypotheses only while the normal
    particle belief is lost or globally uncertain.  Multi-view LiDAR evidence
    updates those hypotheses, and the 300-particle cloud is rebuilt from their
    posterior so the MCL visualization reflects which modes are actually
    gaining or losing probability.
    """

    TRACKING_POSITION_SPREAD_M = 0.20
    TRACKING_HEADING_SPREAD_RAD = 0.30
    LOST_POSITION_SPREAD_M = 0.75
    LOST_HEADING_QUALITY_SCALE_RAD = 0.80

    RECOVERY_EVIDENCE_TRANSLATION_M = 0.035
    RECOVERY_RESOLVE_TRANSLATION_M = 0.18
    RECOVERY_RESOLVE_OBSERVATIONS = 3
    RECOVERY_RESOLVE_TOP_PROBABILITY = 0.80
    RECOVERY_RESOLVE_MAX_SECOND_PROBABILITY = 0.20
    RECOVERY_RESOLVE_MIN_MEAN_SCORE_MARGIN = 0.05
    RECOVERY_AMBIGUOUS_TRANSLATION_M = 0.35
    RECOVERY_AMBIGUOUS_OBSERVATIONS = 10
    RECOVERY_AMBIGUOUS_MIN_STRONG_MASS = 0.80
    RECOVERY_AMBIGUOUS_STRONG_RATIO = 0.60
    RECOVERY_WEAK_MODE_PROBABILITY = 0.005
    RECOVERY_WEAK_MODE_MIN_OBSERVATIONS = 10
    RECOVERY_EFFECTIVE_BEAMS_PER_VIEW = 8.0
    SCAN_MATCH_HEADING_COUNT = 16
    SCAN_MATCH_REFINE_COUNT = 64
    SCAN_MATCH_MAX_CANDIDATES = 12

    def __init__(
        self,
        initial_pose: tuple[float, float, float],
        particle_count: int = 300,
        initial_position_std_m: float = 0.45,
        initial_heading_std_rad: float = 0.50,
        inflation_radius_m: float | None = None,
        motion_distance_noise_fraction: float = 0.05,
        motion_heading_noise_fraction: float = 0.05,
        ess_resample_fraction: float = 0.50,
        lost_measurement_threshold: float = 0.45,
        recovery_measurement_threshold: float = 0.70,
        lost_measurement_updates: int = 3,
        recovery_reinjection_fraction: float = 0.25,
        maximum_lidar_beams: int = 60,
        measurement_sigma_m: float = 0.12,
        measurement_outlier_weight: float = 0.05,
        lidar_min_range_m: float = 0.05,
        lidar_max_range_m: float = 4.0,
    ) -> None:
        if particle_count <= 0:
            raise ValueError("particle_count must be positive")
        if initial_position_std_m < 0.0 or initial_heading_std_rad < 0.0:
            raise ValueError("initialization standard deviations must be non-negative")
        if inflation_radius_m is not None and inflation_radius_m < 0.0:
            raise ValueError("inflation_radius_m must be non-negative")
        if motion_distance_noise_fraction < 0.0 or motion_heading_noise_fraction < 0.0:
            raise ValueError("motion noise fractions must be non-negative")
        if not 0.0 < ess_resample_fraction <= 1.0:
            raise ValueError("ess_resample_fraction must be in (0, 1]")
        if not 0.0 <= lost_measurement_threshold <= 1.0:
            raise ValueError("lost_measurement_threshold must be in [0, 1]")
        if not 0.0 <= recovery_measurement_threshold <= 1.0:
            raise ValueError("recovery_measurement_threshold must be in [0, 1]")
        if lost_measurement_updates <= 0:
            raise ValueError("lost_measurement_updates must be positive")
        if not 0.0 <= recovery_reinjection_fraction <= 1.0:
            raise ValueError("recovery_reinjection_fraction must be in [0, 1]")
        if maximum_lidar_beams <= 0:
            raise ValueError("maximum_lidar_beams must be positive")
        if measurement_sigma_m <= 0.0:
            raise ValueError("measurement_sigma_m must be positive")
        if not 0.0 < measurement_outlier_weight < 1.0:
            raise ValueError("measurement_outlier_weight must be in (0, 1)")
        if lidar_min_range_m < 0.0 or lidar_max_range_m <= lidar_min_range_m:
            raise ValueError("LiDAR range limits are invalid")

        self.initial_pose = initial_pose
        self.particle_count = particle_count
        self.initial_position_std_m = initial_position_std_m
        self.initial_heading_std_rad = initial_heading_std_rad
        self.inflation_radius_m = inflation_radius_m
        self.motion_distance_noise_fraction = motion_distance_noise_fraction
        self.motion_heading_noise_fraction = motion_heading_noise_fraction
        self.ess_resample_fraction = ess_resample_fraction
        self.lost_measurement_threshold = lost_measurement_threshold
        self.recovery_measurement_threshold = recovery_measurement_threshold
        self.lost_measurement_updates = lost_measurement_updates
        self.recovery_reinjection_fraction = recovery_reinjection_fraction
        self.maximum_lidar_beams = maximum_lidar_beams
        self.measurement_sigma_m = measurement_sigma_m
        self.measurement_outlier_weight = measurement_outlier_weight
        self.lidar_min_range_m = lidar_min_range_m
        self.lidar_max_range_m = lidar_max_range_m

        self.random = Random(185)
        self.particles: list[Particle] = []
        self._distance_field_grid = None
        self._distance_field: list[list[float]] | None = None

        self._particle_estimate = Pose2D(*initial_pose)
        self._state = "LOCALIZING"
        self._confidence = 0.0
        self._ess = 0.0
        self._position_spread_m = 0.0
        self._heading_spread_rad = 0.0
        self._measurement_score = 0.0
        self._just_recovered = False

        self._recovery_count = 0
        self._recovery_hypotheses: list[RecoveryHypothesis] = []
        self._reset_runtime_state(global_pending=False)

    def _reset_runtime_state(self, *, global_pending: bool) -> None:
        """Reset transient localization/recovery state without changing particles."""
        self._just_recovered = False
        self._bad_measurement_count = 0
        self._recovery_active = False
        self._recovery_started_from_loss = False
        self._recovery_ambiguous = False
        self._global_initialization_pending = global_pending
        self._recovery_finishing = False
        self._recovery_translation_m = 0.0
        self._recovery_translation_since_evidence_m = 0.0
        self._recovery_observation_count = 0
        self._good_tracking_updates = 0
        self._low_confidence_updates = 0

    def _reset_tracking_counters(self) -> None:
        self._good_tracking_updates = 0
        self._low_confidence_updates = 0

    # ------------------------------------------------------------------
    # Initialization and map helpers
    # ------------------------------------------------------------------

    def _free_space_grid(self, occupancy_grid):
        if self.inflation_radius_m is None:
            self.inflation_radius_m = float(
                getattr(occupancy_grid, "inflation_radius_m", 0.0)
            )
        if self.inflation_radius_m < 0.0:
            raise ValueError("occupancy-grid inflation radius must be non-negative")
        free_grid = occupancy_grid.inflated(self.inflation_radius_m)
        free_cells = [
            (row, col)
            for row in range(free_grid.height)
            for col in range(free_grid.width)
            if not free_grid.occupied(row, col)
        ]
        if not free_cells:
            raise ValueError("occupancy grid has no valid free cells")
        return free_grid, free_cells

    def _random_free_pose(self, free_grid, free_cells) -> Particle:
        row, col = self.random.choice(free_cells)
        x, y = free_grid.cell_to_world(row, col)
        return Particle(x, y, self.random.uniform(-pi, pi), 0.0)

    def _nominal_particle(self, free_grid, free_cells) -> Particle:
        for _ in range(100):
            x = self.random.gauss(self.initial_pose[0], self.initial_position_std_m)
            y = self.random.gauss(self.initial_pose[1], self.initial_position_std_m)
            theta = wrap_angle(
                self.random.gauss(self.initial_pose[2], self.initial_heading_std_rad)
            )
            if not free_grid.world_occupied(x, y):
                return Particle(x, y, theta, 0.0)
        return self._random_free_pose(free_grid, free_cells)

    def initialize(self, occupancy_grid, initialization_mode: str = "nominal") -> None:
        if initialization_mode not in {"nominal", "global_free_space"}:
            raise ValueError(
                "initialization_mode must be 'nominal' or 'global_free_space'"
            )

        free_grid, free_cells = self._free_space_grid(occupancy_grid)
        if initialization_mode == "nominal":
            samples = [
                self._nominal_particle(free_grid, free_cells)
                for _ in range(self.particle_count)
            ]
        else:
            samples = [
                self._random_free_pose(free_grid, free_cells)
                for _ in range(self.particle_count)
            ]

        weight = 1.0 / self.particle_count
        self.particles = [
            Particle(sample.x, sample.y, wrap_angle(sample.theta), weight)
            for sample in samples
        ]
        self._state = "LOCALIZING"
        self._confidence = 0.0
        self._measurement_score = 0.0
        self._recovery_count = 0
        self._recovery_hypotheses = []
        self._reset_runtime_state(
            global_pending=initialization_mode == "global_free_space"
        )
        self._refresh_diagnostics(advance_state=False)
        self._particle_estimate = self._weighted_pose(self.particles)

    def _build_distance_field(self, occupancy_grid) -> list[list[float]]:
        occupied_cells = [
            (row, col)
            for row in range(occupancy_grid.height)
            for col in range(occupancy_grid.width)
            if occupancy_grid.occupied(row, col)
        ]
        if not occupied_cells:
            return [
                [float("inf") for _ in range(occupancy_grid.width)]
                for _ in range(occupancy_grid.height)
            ]
        resolution = occupancy_grid.resolution
        return [
            [
                min(
                    hypot(row - obstacle_row, col - obstacle_col) * resolution
                    for obstacle_row, obstacle_col in occupied_cells
                )
                for col in range(occupancy_grid.width)
            ]
            for row in range(occupancy_grid.height)
        ]

    def _ensure_distance_field(self, occupancy_grid) -> None:
        if self._distance_field_grid is not occupancy_grid:
            self._distance_field_grid = occupancy_grid
            self._distance_field = self._build_distance_field(occupancy_grid)

    def _distance_to_obstacle(self, occupancy_grid, x: float, y: float) -> float:
        row, col = occupancy_grid.world_to_cell(x, y)
        if not occupancy_grid.in_bounds(row, col):
            # An endpoint outside the map is not a valid obstacle hit.  Returning
            # zero here would incorrectly give it the maximum likelihood.
            return float("inf")
        assert self._distance_field is not None
        return self._distance_field[row][col]

    # ------------------------------------------------------------------
    # LiDAR likelihood and particle-filter update
    # ------------------------------------------------------------------

    def _select_scan_points(self, scan_points_robot) -> list[tuple[float, float]]:
        valid_points = []
        for x_forward, y_left in scan_points_robot:
            if not isfinite(x_forward) or not isfinite(y_left):
                continue
            range_m = hypot(x_forward, y_left)
            if range_m < self.lidar_min_range_m:
                continue
            if range_m >= 0.99 * self.lidar_max_range_m:
                continue
            valid_points.append((x_forward, y_left))

        if len(valid_points) <= self.maximum_lidar_beams:
            return valid_points
        if self.maximum_lidar_beams == 1:
            return [valid_points[len(valid_points) // 2]]

        last = len(valid_points) - 1
        return [
            valid_points[round(index * last / (self.maximum_lidar_beams - 1))]
            for index in range(self.maximum_lidar_beams)
        ]

    def _endpoint_likelihood(self, distance_to_obstacle: float) -> float:
        gaussian = exp(-0.5 * (distance_to_obstacle / self.measurement_sigma_m) ** 2)
        return self.measurement_outlier_weight + (
            1.0 - self.measurement_outlier_weight
        ) * gaussian

    def _particle_measurement(self, particle, scan_points_robot, occupancy_grid):
        if not scan_points_robot or occupancy_grid.world_occupied(particle.x, particle.y):
            floor = self.measurement_outlier_weight
            return floor, len(scan_points_robot) * log(floor)

        c = cos(particle.theta)
        s = sin(particle.theta)
        total_score = 0.0
        total_log_likelihood = 0.0
        for x_forward, y_left in scan_points_robot:
            world_x = particle.x + c * x_forward - s * y_left
            world_y = particle.y + s * x_forward + c * y_left
            distance = self._distance_to_obstacle(occupancy_grid, world_x, world_y)
            likelihood = self._endpoint_likelihood(distance)
            total_score += likelihood
            total_log_likelihood += log(max(likelihood, 1.0e-12))
        return total_score / len(scan_points_robot), total_log_likelihood

    def _particle_measurement_score(
        self, particle, scan_points_robot, occupancy_grid
    ) -> float:
        """Compatibility helper used by tests and recovery scan matching."""
        return self._particle_measurement(
            particle, scan_points_robot, occupancy_grid
        )[0]

    def _apply_measurement_update(self, selected_scan, occupancy_grid) -> None:
        scores = []
        log_likelihoods = []
        prior_weights = [max(particle.weight, 1.0e-300) for particle in self.particles]
        prior_total = sum(prior_weights)

        for particle in self.particles:
            score, log_likelihood = self._particle_measurement(
                particle, selected_scan, occupancy_grid
            )
            scores.append(score)
            log_likelihoods.append(log_likelihood)

        self._measurement_score = sum(
            prior * score for prior, score in zip(prior_weights, scores)
        ) / prior_total

        # A simple mean likelihood is useful as a diagnostic, but using that
        # mean directly as the particle weight throws away most of the evidence
        # in a multi-beam scan.  Combine beam likelihoods in log space.  Temper
        # the product to roughly 20 independent beams so an unexpected obstacle
        # does not make one scan excessively sharp.
        likelihood_scale = min(1.0, 20.0 / len(selected_scan))
        log_weights = [
            log(prior) + likelihood_scale * measurement_log
            for prior, measurement_log in zip(prior_weights, log_likelihoods)
        ]
        maximum_log_weight = max(log_weights)
        for particle, log_weight in zip(self.particles, log_weights):
            particle.weight = exp(log_weight - maximum_log_weight)
        self._normalize_weights()

    def _normalize_weights(self) -> None:
        if not self.particles:
            return
        total = sum(particle.weight for particle in self.particles)
        if not isfinite(total) or total <= 1.0e-300:
            equal_weight = 1.0 / len(self.particles)
            for particle in self.particles:
                particle.weight = equal_weight
            return
        for particle in self.particles:
            particle.weight /= total

    def _effective_sample_size(self) -> float:
        self._normalize_weights()
        denominator = sum(particle.weight * particle.weight for particle in self.particles)
        return 0.0 if denominator <= 0.0 else 1.0 / denominator

    def _systematic_resample(self) -> None:
        if not self.particles:
            return
        self._normalize_weights()
        cumulative = []
        running = 0.0
        for particle in self.particles:
            running += particle.weight
            cumulative.append(running)

        spacing = 1.0 / len(self.particles)
        sample = self.random.uniform(0.0, spacing)
        source_index = 0
        resampled = []
        for _ in self.particles:
            while source_index < len(cumulative) - 1 and sample > cumulative[source_index]:
                source_index += 1
            source = self.particles[source_index]
            resampled.append(Particle(source.x, source.y, source.theta, spacing))
            sample += spacing
        self.particles = resampled

    def _propagate_particle(self, particle, delta_motion) -> Particle:
        delta_distance, delta_heading = delta_motion
        distance_std = abs(delta_distance) * self.motion_distance_noise_fraction
        heading_std = abs(delta_heading) * self.motion_heading_noise_fraction
        noisy_distance = delta_distance + self.random.gauss(0.0, distance_std)
        noisy_heading = delta_heading + self.random.gauss(0.0, heading_std)
        midpoint_heading = particle.theta + 0.5 * noisy_heading
        return Particle(
            particle.x + noisy_distance * cos(midpoint_heading),
            particle.y + noisy_distance * sin(midpoint_heading),
            wrap_angle(particle.theta + noisy_heading),
            particle.weight,
        )

    # ------------------------------------------------------------------
    # Kidnapped-robot recovery
    # ------------------------------------------------------------------

    def _scan_match_candidates(
        self,
        occupancy_grid,
        free_grid,
        free_cells,
        scan_points_robot,
        maximum_candidates: int | None = None,
    ) -> list[tuple[float, Particle]]:
        """Return several distinct map-wide LiDAR pose proposals.

        This is used only to initialize global/losing recovery modes.  Later
        translated observations update those modes and rebuild the ordinary
        particle representation from their posterior probabilities.
        """
        if not scan_points_robot:
            return []

        if len(scan_points_robot) > 24:
            last = len(scan_points_robot) - 1
            coarse_scan = [
                scan_points_robot[round(index * last / 23)]
                for index in range(24)
            ]
        else:
            coarse_scan = scan_points_robot

        # Recovery must not permanently lose the true pose because the coarse
        # proposal grid skipped its cell.  The map is only 60 x 80, so checking
        # every valid free cell is still inexpensive enough for the occasional
        # global-recovery event and is much more reliable than striding through
        # the free-cell list.
        maximum_candidates = maximum_candidates or self.SCAN_MATCH_MAX_CANDIDATES
        heading_count = self.SCAN_MATCH_HEADING_COUNT
        coarse = []
        for row, col in free_cells:
            x, y = free_grid.cell_to_world(row, col)
            for heading_index in range(heading_count):
                theta = -pi + 2.0 * pi * heading_index / heading_count
                particle = Particle(x, y, theta, 0.0)
                score = self._particle_measurement_score(
                    particle, coarse_scan, occupancy_grid
                )
                coarse.append((score, particle))

        coarse.sort(key=lambda item: item[0], reverse=True)
        half_cell = 0.5 * occupancy_grid.resolution
        heading_step = pi / 16.0
        refined = []
        for _, source in coarse[: self.SCAN_MATCH_REFINE_COUNT]:
            for dx in (-half_cell, 0.0, half_cell):
                for dy in (-half_cell, 0.0, half_cell):
                    for dtheta in (-heading_step, 0.0, heading_step):
                        particle = Particle(
                            source.x + dx,
                            source.y + dy,
                            wrap_angle(source.theta + dtheta),
                            0.0,
                        )
                        if free_grid.world_occupied(particle.x, particle.y):
                            continue
                        score = self._particle_measurement_score(
                            particle, scan_points_robot, occupancy_grid
                        )
                        refined.append((score, particle))

        ranked = sorted(refined or coarse, key=lambda item: item[0], reverse=True)
        selected: list[tuple[float, Particle]] = []
        for score, particle in ranked:
            distinct = all(
                hypot(particle.x - other.x, particle.y - other.y) >= 0.40
                or abs(wrap_angle(particle.theta - other.theta)) >= 0.40
                for _, other in selected
            )
            if distinct:
                selected.append((score, particle))
                if len(selected) >= maximum_candidates:
                    break
        return selected

    def _sample_near_candidate(self, candidate, free_grid) -> Particle:
        position_std = max(0.04, 0.75 * free_grid.resolution)
        for _ in range(50):
            particle = Particle(
                candidate.x + self.random.gauss(0.0, position_std),
                candidate.y + self.random.gauss(0.0, position_std),
                wrap_angle(candidate.theta + self.random.gauss(0.0, 0.12)),
                0.0,
            )
            if not free_grid.world_occupied(particle.x, particle.y):
                return particle
        return Particle(candidate.x, candidate.y, candidate.theta, 0.0)

    @staticmethod
    def _allocate_counts(probabilities: list[float], total: int) -> list[int]:
        """Allocate ``total`` samples proportionally without losing particles."""
        if not probabilities or total <= 0:
            return [0 for _ in probabilities]
        raw = [probability * total for probability in probabilities]
        counts = [int(value) for value in raw]
        remainder = total - sum(counts)
        order = sorted(
            range(len(raw)),
            key=lambda index: raw[index] - counts[index],
            reverse=True,
        )
        for index in order[:remainder]:
            counts[index] += 1
        return counts

    def _rebuild_recovery_particles(
        self,
        occupancy_grid,
        *,
        global_fraction: float = 0.0,
    ) -> None:
        """Rebuild the particle cloud from the recovery posterior.

        Broad random particles are useful only when a global search is first
        seeded.  After translated observations begin, particles are allocated
        directly from the hypothesis probabilities so weak modes visibly shrink
        instead of being kept alive artificially.
        """
        free_grid, free_cells = self._free_space_grid(occupancy_grid)
        probabilities = self._recovery_probabilities()

        if not self._recovery_hypotheses or not probabilities:
            samples = [
                self._random_free_pose(free_grid, free_cells)
                for _ in range(self.particle_count)
            ]
        else:
            global_count = min(
                self.particle_count,
                max(0, round(global_fraction * self.particle_count)),
            )
            mode_count = self.particle_count - global_count
            counts = self._allocate_counts(probabilities, mode_count)
            samples = [
                self._random_free_pose(free_grid, free_cells)
                for _ in range(global_count)
            ]
            for count, hypothesis in zip(counts, self._recovery_hypotheses):
                candidate = Particle(
                    hypothesis.pose.x,
                    hypothesis.pose.y,
                    hypothesis.pose.theta,
                    0.0,
                )
                samples.extend(
                    self._sample_near_candidate(candidate, free_grid)
                    for _ in range(count)
                )

        equal_weight = 1.0 / self.particle_count
        self.particles = [
            Particle(sample.x, sample.y, wrap_angle(sample.theta), equal_weight)
            for sample in samples[: self.particle_count]
        ]

    def _start_global_search(
        self,
        occupancy_grid,
        selected_scan,
        *,
        started_from_loss: bool,
    ) -> None:
        """Seed several map-wide hypotheses from the current LiDAR scan.

        The same search is used for an uncertain start and for kidnapped-robot
        recovery.  The only difference is the public state while evidence is
        still incomplete: an uncertain start is LOCALIZING, while a confirmed
        kidnapped mismatch is LOST until one or more scan-consistent modes are
        established.
        """
        free_grid, free_cells = self._free_space_grid(occupancy_grid)
        candidates = self._scan_match_candidates(
            occupancy_grid, free_grid, free_cells, selected_scan
        )

        if candidates:
            best_score = candidates[0][0]
            self._recovery_hypotheses = [
                RecoveryHypothesis(
                    Pose2D(particle.x, particle.y, particle.theta),
                    8.0 * (score - best_score),
                    score,
                    score_sum=score,
                    score_count=1,
                )
                for score, particle in candidates
            ]
        else:
            self._recovery_hypotheses = []

        self._rebuild_recovery_particles(
            occupancy_grid,
            global_fraction=self.recovery_reinjection_fraction,
        )
        self._bad_measurement_count = 0
        self._recovery_active = True
        self._recovery_started_from_loss = started_from_loss
        self._recovery_ambiguous = False
        self._global_initialization_pending = False
        self._recovery_finishing = False
        if started_from_loss:
            self._recovery_count += 1
        self._recovery_translation_m = 0.0
        self._recovery_translation_since_evidence_m = 0.0
        self._recovery_observation_count = 0
        self._reset_tracking_counters()
        self._state = "LOST" if started_from_loss else "LOCALIZING"
        self._refresh_diagnostics(advance_state=False)
        self._particle_estimate = self._best_recovery_pose()

    @staticmethod
    def _propagate_pose(pose: Pose2D, delta_motion) -> Pose2D:
        delta_distance, delta_heading = delta_motion
        midpoint_heading = pose.theta + 0.5 * delta_heading
        return Pose2D(
            pose.x + delta_distance * cos(midpoint_heading),
            pose.y + delta_distance * sin(midpoint_heading),
            wrap_angle(pose.theta + delta_heading),
        )

    def _recovery_probabilities(self) -> list[float]:
        if not self._recovery_hypotheses:
            return []
        maximum = max(hypothesis.log_weight for hypothesis in self._recovery_hypotheses)
        unnormalized = [
            exp(hypothesis.log_weight - maximum)
            for hypothesis in self._recovery_hypotheses
        ]
        total = sum(unnormalized)
        return [weight / total for weight in unnormalized]

    def _best_recovery_pose(self) -> Pose2D:
        if not self._recovery_hypotheses:
            return self._weighted_pose(self.particles)
        best = max(self._recovery_hypotheses, key=lambda hypothesis: hypothesis.log_weight)
        return Pose2D(best.pose.x, best.pose.y, best.pose.theta)

    def _score_recovery_view(self, selected_scan, occupancy_grid) -> list[float]:
        """Accumulate one translated LiDAR view into recovery-mode weights."""
        likelihood_scale = min(
            1.0, self.RECOVERY_EFFECTIVE_BEAMS_PER_VIEW / len(selected_scan)
        )
        for hypothesis in self._recovery_hypotheses:
            score, log_likelihood = self._particle_measurement(
                Particle(
                    hypothesis.pose.x,
                    hypothesis.pose.y,
                    hypothesis.pose.theta,
                    1.0,
                ),
                selected_scan,
                occupancy_grid,
            )
            hypothesis.measurement_score = score
            hypothesis.score_sum += score
            hypothesis.score_count += 1
            hypothesis.log_weight += likelihood_scale * log_likelihood

        self._recovery_hypotheses.sort(
            key=lambda hypothesis: hypothesis.log_weight,
            reverse=True,
        )
        return self._recovery_probabilities()

    def _prune_negligible_recovery_modes(
        self, probabilities: list[float]
    ) -> list[float]:
        """Drop only modes that are negligible after several independent views."""
        if (
            self._recovery_observation_count < self.RECOVERY_WEAK_MODE_MIN_OBSERVATIONS
            or len(probabilities) <= 2
        ):
            return probabilities

        best_mean_score = max(
            hypothesis.mean_measurement_score()
            for hypothesis in self._recovery_hypotheses
        )
        keep = [
            index
            for index, (probability, hypothesis) in enumerate(
                zip(probabilities, self._recovery_hypotheses)
            )
            if (
                probability >= self.RECOVERY_WEAK_MODE_PROBABILITY
                or hypothesis.mean_measurement_score()
                >= best_mean_score - self.RECOVERY_RESOLVE_MIN_MEAN_SCORE_MARGIN
            )
        ]
        if len(keep) < 2 or len(keep) == len(self._recovery_hypotheses):
            return probabilities

        self._recovery_hypotheses = [
            self._recovery_hypotheses[index] for index in keep
        ]
        return self._recovery_probabilities()

    def _classify_recovery(self, probabilities: list[float]) -> str:
        """Classify the current posterior as searching, resolved, or ambiguous."""
        if not probabilities or not self._recovery_hypotheses:
            return "searching"

        ranked = self._recovery_hypotheses
        top_probability = probabilities[0]
        second_probability = probabilities[1] if len(probabilities) > 1 else 0.0
        top_mean_score = ranked[0].mean_measurement_score()
        second_mean_score = (
            ranked[1].mean_measurement_score() if len(ranked) > 1 else 0.0
        )
        score_margin = top_mean_score - second_mean_score
        score_separates_modes = (
            len(ranked) == 1
            or score_margin >= self.RECOVERY_RESOLVE_MIN_MEAN_SCORE_MARGIN
        )
        resolved = (
            self._recovery_translation_m >= self.RECOVERY_RESOLVE_TRANSLATION_M
            and self._recovery_observation_count >= self.RECOVERY_RESOLVE_OBSERVATIONS
            and top_mean_score >= self.recovery_measurement_threshold
            and top_probability >= self.RECOVERY_RESOLVE_TOP_PROBABILITY
            and second_probability <= self.RECOVERY_RESOLVE_MAX_SECOND_PROBABILITY
            and score_separates_modes
        )
        if resolved:
            self._recovery_ambiguous = False
            return "resolved"

        strong_floor = self.RECOVERY_AMBIGUOUS_STRONG_RATIO * top_probability
        strong_mass = sum(
            probability for probability in probabilities if probability >= strong_floor
        )
        score_tied = (
            len(ranked) > 1
            and second_mean_score
            >= top_mean_score - self.RECOVERY_RESOLVE_MIN_MEAN_SCORE_MARGIN
        )
        probability_multimodal = (
            top_probability < 0.75
            and sum(
                probability >= strong_floor for probability in probabilities
            )
            >= 2
            and strong_mass >= self.RECOVERY_AMBIGUOUS_MIN_STRONG_MASS
        )
        multimodal = (
            self._recovery_translation_m >= self.RECOVERY_AMBIGUOUS_TRANSLATION_M
            and self._recovery_observation_count
            >= self.RECOVERY_AMBIGUOUS_OBSERVATIONS
            and top_mean_score >= self.recovery_measurement_threshold
            and (probability_multimodal or score_tied)
        )
        if multimodal:
            self._recovery_ambiguous = True

        return "ambiguous" if self._recovery_ambiguous else "searching"

    def _update_recovery_hypotheses(
        self, delta_motion, selected_scan, occupancy_grid
    ) -> str:
        """Propagate recovery modes and add evidence after translated views."""
        delta_distance = abs(delta_motion[0])
        self._recovery_translation_m += delta_distance
        self._recovery_translation_since_evidence_m += delta_distance
        for hypothesis in self._recovery_hypotheses:
            hypothesis.pose = self._propagate_pose(hypothesis.pose, delta_motion)

        if not self._recovery_hypotheses or not selected_scan:
            return "searching"
        if (
            self._recovery_translation_since_evidence_m
            < self.RECOVERY_EVIDENCE_TRANSLATION_M
        ):
            return "searching"

        self._recovery_translation_since_evidence_m = 0.0
        self._recovery_observation_count += 1
        probabilities = self._score_recovery_view(selected_scan, occupancy_grid)
        probabilities = self._prune_negligible_recovery_modes(probabilities)
        self._rebuild_recovery_particles(occupancy_grid)
        return self._classify_recovery(probabilities)


    def _finish_recovery(self, occupancy_grid) -> None:
        """Collapse the particle cloud around a uniquely supported pose."""
        winner = self._best_recovery_pose()
        free_grid, _ = self._free_space_grid(occupancy_grid)
        candidate = Particle(winner.x, winner.y, winner.theta, 0.0)
        equal_weight = 1.0 / self.particle_count
        self.particles = [
            Particle(sample.x, sample.y, wrap_angle(sample.theta), equal_weight)
            for sample in (
                self._sample_near_candidate(candidate, free_grid)
                for _ in range(self.particle_count)
            )
        ]
        self._recovery_active = False
        self._recovery_started_from_loss = False
        self._recovery_ambiguous = False
        self._recovery_finishing = True
        self._recovery_hypotheses = []
        self._reset_tracking_counters()
        self._state = "LOCALIZING"
        self._particle_estimate = winner

    # ------------------------------------------------------------------
    # State, confidence, and estimates
    # ------------------------------------------------------------------

    def _weighted_pose(self, particles) -> Pose2D:
        if not particles:
            return self._particle_estimate
        total = sum(max(0.0, particle.weight) for particle in particles)
        if total <= 0.0:
            return self._particle_estimate
        x = sum(max(0.0, p.weight) * p.x for p in particles) / total
        y = sum(max(0.0, p.weight) * p.y for p in particles) / total
        sin_heading = sum(max(0.0, p.weight) * sin(p.theta) for p in particles)
        cos_heading = sum(max(0.0, p.weight) * cos(p.theta) for p in particles)
        return Pose2D(x, y, atan2(sin_heading, cos_heading))

    def _update_loss_detection(self) -> None:
        if self._recovery_active:
            self._bad_measurement_count = 0
        elif self._measurement_score < self.lost_measurement_threshold:
            self._bad_measurement_count += 1
        else:
            self._bad_measurement_count = 0

    def _refresh_diagnostics(self, *, advance_state: bool = True) -> None:
        if not self.particles:
            self._ess = 0.0
            self._position_spread_m = 0.0
            self._heading_spread_rad = 0.0
            self._confidence = 0.0
            if advance_state:
                self._state = "LOCALIZING"
            return

        self._normalize_weights()
        self._ess = self._effective_sample_size()
        mean_pose = self._weighted_pose(self.particles)
        position_variance = sum(
            particle.weight
            * ((particle.x - mean_pose.x) ** 2 + (particle.y - mean_pose.y) ** 2)
            for particle in self.particles
        )
        heading_variance = sum(
            particle.weight * wrap_angle(particle.theta - mean_pose.theta) ** 2
            for particle in self.particles
        )
        self._position_spread_m = sqrt(max(0.0, position_variance))
        self._heading_spread_rad = sqrt(max(0.0, heading_variance))

        position_quality = max(
            0.0, min(1.0, 1.0 - self._position_spread_m / self.LOST_POSITION_SPREAD_M)
        )
        heading_quality = max(
            0.0,
            min(
                1.0,
                1.0
                - self._heading_spread_rad / self.LOST_HEADING_QUALITY_SCALE_RAD,
            ),
        )
        self._confidence = (
            0.40 * position_quality
            + 0.30 * heading_quality
            + 0.30 * self._measurement_score
        )

        if not advance_state:
            return

        good_tracking_update = (
            self._confidence >= 0.65
            and self._position_spread_m <= self.TRACKING_POSITION_SPREAD_M
            and self._heading_spread_rad <= self.TRACKING_HEADING_SPREAD_RAD
        )
        if good_tracking_update:
            self._good_tracking_updates += 1
        else:
            self._good_tracking_updates = 0

        if self._confidence < 0.25:
            self._low_confidence_updates += 1
        else:
            self._low_confidence_updates = 0

        previous_state = self._state
        if self._bad_measurement_count >= self.lost_measurement_updates:
            self._state = "LOST"
        elif self._position_spread_m > self.LOST_POSITION_SPREAD_M:
            self._state = "LOST"
        elif self._low_confidence_updates >= 10:
            self._state = "LOST"
        elif self._good_tracking_updates >= 3:
            self._state = "TRACKING"
        else:
            self._state = "LOCALIZING"

        if self._recovery_finishing and self._state == "TRACKING":
            self._just_recovered = previous_state != "TRACKING"
            self._recovery_finishing = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(
        self,
        odometry_pose: Pose2D,
        delta_motion: tuple[float, float],
        scan_points_robot: list[tuple[float, float]],
        occupancy_grid,
    ) -> Pose2D:
        self._just_recovered = False
        self.particles = [
            self._propagate_particle(particle, delta_motion)
            for particle in self.particles
        ]
        selected_scan = self._select_scan_points(scan_points_robot)

        if selected_scan and occupancy_grid is not None and self.particles:
            self._ensure_distance_field(occupancy_grid)

            if self._global_initialization_pending:
                self._start_global_search(
                    occupancy_grid,
                    selected_scan,
                    started_from_loss=False,
                )

            if self._recovery_active:
                recovery_status = self._update_recovery_hypotheses(
                    delta_motion, selected_scan, occupancy_grid
                )
                if recovery_status == "ambiguous" or self._recovery_ambiguous:
                    self._state = "LOCALIZING"
                elif self._recovery_started_from_loss:
                    self._state = "LOST"
                else:
                    self._state = "LOCALIZING"
                self._measurement_score = (
                    self._recovery_hypotheses[0].measurement_score
                    if self._recovery_hypotheses
                    else 0.0
                )
                self._refresh_diagnostics(advance_state=False)
                self._particle_estimate = self._best_recovery_pose()
                if recovery_status == "resolved":
                    self._finish_recovery(occupancy_grid)
                return self.get_estimate()

            self._apply_measurement_update(selected_scan, occupancy_grid)
            self._update_loss_detection()

            if (
                self._bad_measurement_count >= self.lost_measurement_updates
            ):
                self._start_global_search(
                    occupancy_grid,
                    selected_scan,
                    started_from_loss=True,
                )
                return self.get_estimate()

            self._ess = self._effective_sample_size()
            if self._ess < self.ess_resample_fraction * len(self.particles):
                self._systematic_resample()

        self._refresh_diagnostics()
        self._particle_estimate = self._weighted_pose(self.particles)
        return self.get_estimate()

    def get_state(self) -> str:
        return self._state

    def get_confidence(self) -> float:
        return self._confidence

    def get_estimate(self) -> Pose2D:
        return Pose2D(
            self._particle_estimate.x,
            self._particle_estimate.y,
            self._particle_estimate.theta,
        )

    def get_recovery_hypotheses(self) -> list[tuple[float, Pose2D]]:
        probabilities = self._recovery_probabilities()
        return [
            (probability, Pose2D(hypothesis.pose.x, hypothesis.pose.y, hypothesis.pose.theta))
            for probability, hypothesis in zip(probabilities, self._recovery_hypotheses)
        ]

    def get_diagnostics(self) -> dict:
        return {
            "ess": self._ess,
            "position_spread_m": self._position_spread_m,
            "heading_spread_rad": self._heading_spread_rad,
            "measurement_score": self._measurement_score,
            "just_recovered": self._just_recovered,
        }

    def get_debug_diagnostics(self) -> dict:
        """Extra fields kept separate from the fixed Task 1 diagnostics API."""
        diagnostics = self.get_diagnostics()
        diagnostics.update(
            {
                "bad_measurement_count": self._bad_measurement_count,
                "recovery_count": self._recovery_count,
                "recovery_hypothesis_count": len(self._recovery_hypotheses),
                "recovery_translation_m": self._recovery_translation_m,
                "recovery_observation_count": self._recovery_observation_count,
                "recovery_active": self._recovery_active,
                "recovery_ambiguous": self._recovery_ambiguous,
            }
        )
        return diagnostics
