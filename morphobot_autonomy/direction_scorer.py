#!/usr/bin/env python3

import math
from typing import Dict, List

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class DirectionScorer(Node):

    def __init__(self):
        super().__init__('direction_scorer')

        # =====================================================
        # TOPICS
        # =====================================================

        self.declare_parameter(
            'input_topic',
            '/directional_clearance'
        )

        self.declare_parameter(
            'output_topic',
            '/navigation_decision'
        )

        # =====================================================
        # DIRECTION PROFILE
        # =====================================================

        self.declare_parameter(
            'min_angle_deg',
            -90.0
        )

        self.declare_parameter(
            'bin_width_deg',
            5.0
        )

        # =====================================================
        # CLEARANCE
        # =====================================================

        # Maximum LiDAR clearance used for normalization.

        self.declare_parameter(
            'max_clearance_m',
            5.0
        )

        # Front path is considered blocked below this distance.

        self.declare_parameter(
            'front_blocked_threshold_m',
            2.0
        )

        # Basic LiDAR clearance threshold.

        self.declare_parameter(
            'safe_clearance_threshold_m',
            1.0
        )

        # =====================================================
        # MANEUVER CLEARANCE
        # =====================================================

        # This is stricter than the basic safe-bin threshold.
        #
        # A LiDAR bin may be considered locally clear at 1.0 m,
        # but that does NOT automatically make it suitable for
        # an actual UGV avoidance maneuver.
        #
        # A candidate maneuver corridor must satisfy this
        # stronger threshold.

        self.declare_parameter(
            'maneuver_clearance_threshold_m',
            1.5
        )

        # =====================================================
        # UGV MANEUVERING RANGE
        # =====================================================

        # Positive = LEFT
        # Negative = RIGHT

        self.declare_parameter(
            'min_avoidance_angle_deg',
            -60.0
        )

        self.declare_parameter(
            'max_avoidance_angle_deg',
            60.0
        )

        # =====================================================
        # FRONT REGION
        # =====================================================

        self.declare_parameter(
            'front_half_width_deg',
            15.0
        )

        # =====================================================
        # CONTINUOUS FREE-SPACE REQUIREMENT
        # =====================================================

        # Number of consecutive maneuver-safe bins required
        # to form a usable avoidance corridor.

        self.declare_parameter(
            'min_consecutive_safe_bins',
            3
        )

        # =====================================================
        # SIDE DECISION
        # =====================================================

        self.declare_parameter(
            'side_decision_margin',
            0.05
        )

        # =====================================================
        # SYMMETRIC OBSTACLE TIE BREAKING
        # =====================================================

        self.declare_parameter(
            'tie_break_enabled',
            True
        )

        self.declare_parameter(
            'default_tie_break_side',
            'LEFT'
        )

        self.declare_parameter(
            'exact_tie_epsilon',
            1e-3
        )

        # =====================================================
        # SIDE-SCORE WEIGHTS
        # =====================================================

        self.declare_parameter(
            'side_mean_weight',
            0.50
        )

        self.declare_parameter(
            'side_min_weight',
            0.25
        )

        self.declare_parameter(
            'side_width_weight',
            0.25
        )

        # =====================================================
        # HEADING SCORE WEIGHTS
        # =====================================================

        self.declare_parameter(
            'clearance_weight',
            0.60
        )

        self.declare_parameter(
            'forward_weight',
            0.30
        )

        self.declare_parameter(
            'turn_weight',
            0.10
        )

        # =====================================================
        # READ PARAMETERS
        # =====================================================

        self.input_topic = str(
            self.get_parameter(
                'input_topic'
            ).value
        )

        self.output_topic = str(
            self.get_parameter(
                'output_topic'
            ).value
        )

        self.min_angle_deg = float(
            self.get_parameter(
                'min_angle_deg'
            ).value
        )

        self.bin_width_deg = float(
            self.get_parameter(
                'bin_width_deg'
            ).value
        )

        self.max_clearance = float(
            self.get_parameter(
                'max_clearance_m'
            ).value
        )

        self.front_blocked_threshold = float(
            self.get_parameter(
                'front_blocked_threshold_m'
            ).value
        )

        self.safe_clearance_threshold = float(
            self.get_parameter(
                'safe_clearance_threshold_m'
            ).value
        )

        self.maneuver_clearance_threshold = float(
            self.get_parameter(
                'maneuver_clearance_threshold_m'
            ).value
        )

        self.min_avoidance_angle = float(
            self.get_parameter(
                'min_avoidance_angle_deg'
            ).value
        )

        self.max_avoidance_angle = float(
            self.get_parameter(
                'max_avoidance_angle_deg'
            ).value
        )

        self.front_half_width = float(
            self.get_parameter(
                'front_half_width_deg'
            ).value
        )

        self.min_consecutive_safe_bins = int(
            self.get_parameter(
                'min_consecutive_safe_bins'
            ).value
        )

        self.side_decision_margin = float(
            self.get_parameter(
                'side_decision_margin'
            ).value
        )

        self.tie_break_enabled = bool(
            self.get_parameter(
                'tie_break_enabled'
            ).value
        )

        self.default_tie_break_side = str(
            self.get_parameter(
                'default_tie_break_side'
            ).value
        ).upper()

        self.exact_tie_epsilon = float(
            self.get_parameter(
                'exact_tie_epsilon'
            ).value
        )

        self.side_mean_weight = float(
            self.get_parameter(
                'side_mean_weight'
            ).value
        )

        self.side_min_weight = float(
            self.get_parameter(
                'side_min_weight'
            ).value
        )

        self.side_width_weight = float(
            self.get_parameter(
                'side_width_weight'
            ).value
        )

        self.w_clearance = float(
            self.get_parameter(
                'clearance_weight'
            ).value
        )

        self.w_forward = float(
            self.get_parameter(
                'forward_weight'
            ).value
        )

        self.w_turn = float(
            self.get_parameter(
                'turn_weight'
            ).value
        )

        # =====================================================
        # PARAMETER VALIDATION
        # =====================================================

        if self.max_clearance <= 0.0:
            self.max_clearance = 5.0

        if self.bin_width_deg <= 0.0:
            self.bin_width_deg = 5.0

        if self.min_consecutive_safe_bins < 1:
            self.min_consecutive_safe_bins = 1

        if self.safe_clearance_threshold <= 0.0:
            self.safe_clearance_threshold = 1.0

        if self.maneuver_clearance_threshold <= 0.0:
            self.maneuver_clearance_threshold = 1.5

        if (
            self.maneuver_clearance_threshold
            < self.safe_clearance_threshold
        ):
            self.get_logger().warning(
                'Maneuver clearance threshold is below '
                'basic safe clearance threshold. '
                'Using basic safe threshold.'
            )

            self.maneuver_clearance_threshold = (
                self.safe_clearance_threshold
            )

        if self.exact_tie_epsilon < 0.0:
            self.exact_tie_epsilon = 1e-3

        if self.default_tie_break_side not in (
            'LEFT',
            'RIGHT'
        ):
            self.default_tie_break_side = 'LEFT'

        # =====================================================
        # NORMALIZE SIDE WEIGHTS
        # =====================================================

        side_weight_sum = (
            self.side_mean_weight
            + self.side_min_weight
            + self.side_width_weight
        )

        if side_weight_sum <= 0.0:

            self.side_mean_weight = 0.50
            self.side_min_weight = 0.25
            self.side_width_weight = 0.25

        else:

            self.side_mean_weight /= side_weight_sum
            self.side_min_weight /= side_weight_sum
            self.side_width_weight /= side_weight_sum

        # =====================================================
        # ROS INTERFACES
        # =====================================================

        self.subscriber = self.create_subscription(
            Float32MultiArray,
            self.input_topic,
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            self.output_topic,
            10
        )

        # =====================================================
        # STARTUP INFORMATION
        # =====================================================

        self.get_logger().info(
            'Direction scorer started'
        )

        self.get_logger().info(
            'Navigation decision layout: '
            '[best_heading, best_score, left_score, '
            'right_score, front_clearance, path_blocked, '
            'direction_safe, confidence, decision_ambiguous]'
        )

        self.get_logger().info(
            f'Input topic: {self.input_topic}'
        )

        self.get_logger().info(
            f'Output topic: {self.output_topic}'
        )

        self.get_logger().info(
            f'Avoidance range: '
            f'{self.min_avoidance_angle:+.1f}° '
            f'to '
            f'{self.max_avoidance_angle:+.1f}°'
        )

        self.get_logger().info(
            f'Basic safe clearance: '
            f'{self.safe_clearance_threshold:.2f} m'
        )

        self.get_logger().info(
            f'Maneuver clearance: '
            f'{self.maneuver_clearance_threshold:.2f} m'
        )

        self.get_logger().info(
            f'Minimum corridor width: '
            f'{self.min_consecutive_safe_bins} bins '
            f'('
            f'{self.min_consecutive_safe_bins * self.bin_width_deg:.1f}°'
            f')'
        )

        self.get_logger().info(
            f'Side decision margin: '
            f'{self.side_decision_margin:.3f}'
        )

        self.get_logger().info(
            f'Tie-break: '
            f'{"ENABLED" if self.tie_break_enabled else "DISABLED"} '
            f'| default={self.default_tie_break_side}'
        )

    # =========================================================
    # MAIN CALLBACK
    # =========================================================

    def callback(
        self,
        msg: Float32MultiArray
    ) -> None:

        clearances = list(msg.data)

        # -----------------------------------------------------
        # Validate input.
        # -----------------------------------------------------

        if not clearances:

            self.get_logger().warning(
                'Received empty directional clearance profile'
            )

            return

        # =====================================================
        # BUILD HEADINGS
        # =====================================================

        headings = [
            self.min_angle_deg
            + index * self.bin_width_deg
            for index in range(len(clearances))
        ]

        # =====================================================
        # HEADING SCORES
        # =====================================================

        scores = [
            self.calculate_heading_score(
                angle_deg,
                clearance
            )
            for angle_deg, clearance
            in zip(
                headings,
                clearances
            )
        ]

        # =====================================================
        # FRONT CLEARANCE
        # =====================================================

        front_values = [
            clearance
            for angle_deg, clearance
            in zip(
                headings,
                clearances
            )
            if abs(angle_deg)
            <= self.front_half_width
        ]

        if front_values:

            front_clearance = min(
                front_values
            )

        else:

            center_index = min(
                range(len(headings)),
                key=lambda i: abs(
                    headings[i]
                )
            )

            front_clearance = clearances[
                center_index
            ]

        # =====================================================
        # FORWARD PATH STATUS
        # =====================================================

        path_blocked = (
            front_clearance
            <= self.front_blocked_threshold
        )

        # =====================================================
        # SIDE CORRIDOR ANALYSIS
        # =====================================================

        left_result = self.analyze_side(
            headings=headings,
            clearances=clearances,
            scores=scores,
            side='LEFT'
        )

        right_result = self.analyze_side(
            headings=headings,
            clearances=clearances,
            scores=scores,
            side='RIGHT'
        )

        left_score = left_result['side_score']
        right_score = right_result['side_score']

        left_available = left_result['available']
        right_available = right_result['available']

        # =====================================================
        # DEFAULT DECISION
        # =====================================================

        best_heading = 0.0
        best_score = 0.0

        direction_safe = False
        decision_ambiguous = False

        confidence = 0.0

        # =====================================================
        # CASE 1 — FORWARD PATH CLEAR
        # =====================================================

        if not path_blocked:

            center_index = min(
                range(len(headings)),
                key=lambda i: abs(
                    headings[i]
                )
            )

            best_heading = 0.0

            best_score = scores[
                center_index
            ]

            direction_safe = (
                clearances[center_index]
                >= self.safe_clearance_threshold
            )

            confidence = 1.0

            decision_ambiguous = False

        # =====================================================
        # CASE 2 — FRONT PATH BLOCKED
        # =====================================================

        else:

            # -------------------------------------------------
            # CASE 2A:
            # Neither side has a valid maneuver corridor.
            # -------------------------------------------------

            if (
                not left_available
                and
                not right_available
            ):

                best_heading = (
                    self.get_least_bad_heading(
                        headings,
                        clearances
                    )
                )

                best_score = (
                    self.get_score_for_heading(
                        best_heading,
                        headings,
                        scores
                    )
                )

                direction_safe = False

                decision_ambiguous = False

                confidence = 0.0

                self.get_logger().warning(
                    'NO MANEUVER CORRIDOR | '
                    f'left_min='
                    f'{left_result["minimum_clearance"]:.2f} m | '
                    f'right_min='
                    f'{right_result["minimum_clearance"]:.2f} m'
                )

            # -------------------------------------------------
            # CASE 2B:
            # LEFT only.
            # -------------------------------------------------

            elif (
                left_available
                and
                not right_available
            ):

                best_heading = (
                    self.get_best_heading_from_corridor(
                        left_result
                    )
                )

                best_score = (
                    left_result[
                        'best_heading_score'
                    ]
                )

                direction_safe = True

                decision_ambiguous = False

                confidence = 1.0

            # -------------------------------------------------
            # CASE 2C:
            # RIGHT only.
            # -------------------------------------------------

            elif (
                right_available
                and
                not left_available
            ):

                best_heading = (
                    self.get_best_heading_from_corridor(
                        right_result
                    )
                )

                best_score = (
                    right_result[
                        'best_heading_score'
                    ]
                )

                direction_safe = True

                decision_ambiguous = False

                confidence = 1.0

            # -------------------------------------------------
            # CASE 2D:
            # BOTH SIDES AVAILABLE.
            # -------------------------------------------------

            else:

                side_difference = abs(
                    left_score
                    - right_score
                )

                # =============================================
                # TIE / NEAR-TIE
                # =============================================

                if (
                    self.tie_break_enabled
                    and
                    side_difference
                    < self.side_decision_margin
                ):

                    # -----------------------------------------
                    # Select side.
                    # -----------------------------------------

                    if (
                        side_difference
                        <= self.exact_tie_epsilon
                    ):

                        chosen_side = (
                            self.default_tie_break_side
                        )

                    elif left_score > right_score:

                        chosen_side = 'LEFT'

                    else:

                        chosen_side = 'RIGHT'

                    # -----------------------------------------
                    # Select corresponding corridor.
                    # -----------------------------------------

                    if chosen_side == 'LEFT':

                        selected_result = left_result
                        selected_score = left_score

                    else:

                        selected_result = right_result
                        selected_score = right_score

                    best_heading = (
                        self.get_best_heading_from_corridor(
                            selected_result
                        )
                    )

                    best_score = (
                        selected_result[
                            'best_heading_score'
                        ]
                    )

                    # -----------------------------------------
                    # The corridor itself is safe.
                    # The two sides are simply similar.
                    # -----------------------------------------

                    direction_safe = True

                    decision_ambiguous = False

                    # Corridor quality, not side superiority,
                    # is used as confidence.

                    confidence = max(
                        0.0,
                        min(
                            1.0,
                            selected_score
                        )
                    )

                    self.get_logger().info(
                        f'TIE-BREAK | '
                        f'left={left_score:.3f} | '
                        f'right={right_score:.3f} | '
                        f'chosen={chosen_side} | '
                        f'heading={best_heading:+.1f}° | '
                        f'confidence={confidence:.3f}'
                    )

                # =============================================
                # LEFT CLEARLY BETTER
                # =============================================

                elif left_score > right_score:

                    best_heading = (
                        self.get_best_heading_from_corridor(
                            left_result
                        )
                    )

                    best_score = (
                        left_result[
                            'best_heading_score'
                        ]
                    )

                    direction_safe = True

                    decision_ambiguous = False

                    confidence = (
                        left_score
                        - right_score
                    ) / max(
                        left_score,
                        1e-6
                    )

                    confidence = max(
                        0.0,
                        min(
                            1.0,
                            confidence
                        )
                    )

                # =============================================
                # RIGHT CLEARLY BETTER
                # =============================================

                else:

                    best_heading = (
                        self.get_best_heading_from_corridor(
                            right_result
                        )
                    )

                    best_score = (
                        right_result[
                            'best_heading_score'
                        ]
                    )

                    direction_safe = True

                    decision_ambiguous = False

                    confidence = (
                        right_score
                        - left_score
                    ) / max(
                        right_score,
                        1e-6
                    )

                    confidence = max(
                        0.0,
                        min(
                            1.0,
                            confidence
                        )
                    )

        # =====================================================
        # SAFETY OVERRIDE
        # =====================================================

        if not direction_safe:
            confidence = 0.0

        # =====================================================
        # PUBLISH
        # =====================================================

        # [0] best_heading
        # [1] best_score
        # [2] left_score
        # [3] right_score
        # [4] front_clearance
        # [5] path_blocked
        # [6] direction_safe
        # [7] confidence
        # [8] decision_ambiguous

        output = Float32MultiArray()

        output.data = [
            float(best_heading),
            float(best_score),
            float(left_score),
            float(right_score),
            float(front_clearance),
            float(path_blocked),
            float(direction_safe),
            float(confidence),
            float(decision_ambiguous)
        ]

        self.publisher.publish(
            output
        )

        # =====================================================
        # DEBUG
        # =====================================================

        self.get_logger().info(
            f'heading={best_heading:+.1f}° | '
            f'score={best_score:.3f} | '
            f'left={left_score:.3f} | '
            f'right={right_score:.3f} | '
            f'front={front_clearance:.2f} m | '
            f'blocked={path_blocked} | '
            f'safe={direction_safe} | '
            f'confidence={confidence:.3f} | '
            f'ambiguous={decision_ambiguous}'
        )

    # =========================================================
    # ANALYZE ONE SIDE
    # =========================================================

    def analyze_side(
        self,
        headings: List[float],
        clearances: List[float],
        scores: List[float],
        side: str
    ) -> Dict:

        # -----------------------------------------------------
        # Determine angular region.
        # -----------------------------------------------------

        if side == 'LEFT':

            region_indices = [
                i
                for i, angle in enumerate(headings)
                if (
                    self.side_start('LEFT')
                    <= angle
                    <= self.max_avoidance_angle
                )
            ]

        else:

            region_indices = [
                i
                for i, angle in enumerate(headings)
                if (
                    self.min_avoidance_angle
                    <= angle
                    <= self.side_end('RIGHT')
                )
            ]

        # -----------------------------------------------------
        # No bins available.
        # -----------------------------------------------------

        if not region_indices:

            return self.empty_side_result()

        # -----------------------------------------------------
        # Determine maneuver-safe bins.
        #
        # IMPORTANT:
        #
        # The old algorithm used only:
        #
        #     clearance >= 1.0 m
        #
        # which could classify your obstacle ring as an
        # available corridor.
        #
        # A maneuver corridor now requires the stronger
        # maneuver_clearance_threshold.
        # -----------------------------------------------------

        maneuver_safe_mask = []

        for i in region_indices:

            maneuver_safe_mask.append(
                clearances[i]
                >= self.maneuver_clearance_threshold
            )

        # =====================================================
        # FIND CONSECUTIVE MANEUVER-SAFE RUNS
        # =====================================================

        runs: List[List[int]] = []

        current_run: List[int] = []

        for local_index, is_safe in enumerate(
            maneuver_safe_mask
        ):

            if is_safe:

                current_run.append(
                    region_indices[local_index]
                )

            else:

                if current_run:

                    runs.append(
                        current_run
                    )

                current_run = []

        # Final run.

        if current_run:

            runs.append(
                current_run
            )

        # -----------------------------------------------------
        # Keep only sufficiently wide corridors.
        # -----------------------------------------------------

        valid_runs = [
            run
            for run in runs
            if len(run)
            >= self.min_consecutive_safe_bins
        ]

        # =====================================================
        # NO VALID CORRIDOR
        # =====================================================

        if not valid_runs:

            return self.empty_side_result()

        # =====================================================
        # SCORE EACH VALID CORRIDOR
        # =====================================================

        corridor_results = []

        total_region_bins = max(
            len(region_indices),
            1
        )

        for run in valid_runs:

            run_clearances = [
                clearances[i]
                for i in run
            ]

            run_scores = [
                scores[i]
                for i in run
            ]

            # -------------------------------------------------
            # Mean clearance.
            # -------------------------------------------------

            mean_clearance = (
                sum(run_clearances)
                / len(run_clearances)
            )

            mean_clearance_normalized = min(
                mean_clearance
                / self.max_clearance,
                1.0
            )

            # -------------------------------------------------
            # Minimum clearance.
            # -------------------------------------------------

            minimum_clearance = min(
                run_clearances
            )

            minimum_clearance_normalized = min(
                minimum_clearance
                / self.max_clearance,
                1.0
            )

            # -------------------------------------------------
            # Corridor width.
            # -------------------------------------------------

            width_ratio = min(
                len(run)
                / total_region_bins,
                1.0
            )

            # -------------------------------------------------
            # Corridor quality.
            # -------------------------------------------------

            side_score = (
                self.side_mean_weight
                * mean_clearance_normalized
                +
                self.side_min_weight
                * minimum_clearance_normalized
                +
                self.side_width_weight
                * width_ratio
            )

            # -------------------------------------------------
            # Best individual heading inside corridor.
            # -------------------------------------------------

            best_local_index = max(
                range(len(run)),
                key=lambda j: (
                    run_scores[j]
                    -
                    0.02
                    * (
                        abs(
                            headings[run[j]]
                        )
                        / 60.0
                    )
                )
            )

            best_index = run[
                best_local_index
            ]

            corridor_results.append(
                {
                    'indices': run,

                    'side_score':
                        side_score,

                    'best_heading':
                        headings[best_index],

                    'best_heading_score':
                        scores[best_index],

                    'mean_clearance':
                        mean_clearance,

                    'minimum_clearance':
                        minimum_clearance,

                    'width_ratio':
                        width_ratio
                }
            )

        # =====================================================
        # SELECT BEST CORRIDOR ON THIS SIDE
        # =====================================================

        best_corridor = max(
            corridor_results,
            key=lambda item: item[
                'side_score'
            ]
        )

        return {
            'available': True,

            'side_score':
                best_corridor[
                    'side_score'
                ],

            'best_heading':
                best_corridor[
                    'best_heading'
                ],

            'best_heading_score':
                best_corridor[
                    'best_heading_score'
                ],

            'mean_clearance':
                best_corridor[
                    'mean_clearance'
                ],

            'minimum_clearance':
                best_corridor[
                    'minimum_clearance'
                ],

            'width_ratio':
                best_corridor[
                    'width_ratio'
                ],

            'corridor_count':
                len(corridor_results)
        }

    # =========================================================
    # SIDE REGION HELPERS
    # =========================================================

    def side_start(
        self,
        side: str
    ) -> float:

        if side == 'LEFT':
            return self.front_half_width

        return self.min_avoidance_angle

    def side_end(
        self,
        side: str
    ) -> float:

        if side == 'RIGHT':
            return -self.front_half_width

        return self.max_avoidance_angle

    # =========================================================
    # EMPTY SIDE RESULT
    # =========================================================

    def empty_side_result(self) -> Dict:

        return {
            'available': False,
            'side_score': 0.0,
            'best_heading': 0.0,
            'best_heading_score': 0.0,
            'mean_clearance': 0.0,
            'minimum_clearance': 0.0,
            'width_ratio': 0.0,
            'corridor_count': 0
        }

    # =========================================================
    # HEADING SCORE
    # =========================================================

    def calculate_heading_score(
        self,
        angle_deg: float,
        clearance: float
    ) -> float:

        angle_rad = math.radians(
            angle_deg
        )

        # -----------------------------------------------------
        # Clearance component.
        # -----------------------------------------------------

        clearance_clamped = min(
            max(
                clearance,
                0.0
            ),
            self.max_clearance
        )

        clearance_score = (
            clearance_clamped
            / self.max_clearance
        )

        # -----------------------------------------------------
        # Forward preference.
        # -----------------------------------------------------

        forward_score = max(
            math.cos(angle_rad),
            0.0
        )

        # -----------------------------------------------------
        # Turning cost.
        # -----------------------------------------------------

        turn_cost = (
            abs(angle_deg)
            / 90.0
        )

        # -----------------------------------------------------
        # Final heading score.
        # -----------------------------------------------------

        return (
            self.w_clearance
            * clearance_score
            +
            self.w_forward
            * forward_score
            -
            self.w_turn
            * turn_cost
        )

    # =========================================================
    # BEST HEADING FROM CORRIDOR
    # =========================================================

    def get_best_heading_from_corridor(
        self,
        corridor: Dict
    ) -> float:

        return float(
            corridor['best_heading']
        )

    # =========================================================
    # LEAST-BAD DIAGNOSTIC HEADING
    # =========================================================

    def get_least_bad_heading(
        self,
        headings: List[float],
        clearances: List[float]
    ) -> float:

        maneuver_indices = [
            i
            for i, angle in enumerate(headings)
            if (
                self.min_avoidance_angle
                <= angle
                <= self.max_avoidance_angle
                and
                abs(angle)
                > self.front_half_width
            )
        ]

        if not maneuver_indices:

            return 0.0

        best_index = max(
            maneuver_indices,
            key=lambda i: clearances[i]
        )

        return float(
            headings[best_index]
        )

    # =========================================================
    # GET SCORE FOR HEADING
    # =========================================================

    def get_score_for_heading(
        self,
        heading: float,
        headings: List[float],
        scores: List[float]
    ) -> float:

        if not headings:

            return 0.0

        index = min(
            range(len(headings)),
            key=lambda i: abs(
                headings[i]
                - heading
            )
        )

        return float(
            scores[index]
        )


def main(args=None):

    rclpy.init(args=args)

    node = DirectionScorer()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()