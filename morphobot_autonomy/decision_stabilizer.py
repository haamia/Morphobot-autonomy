#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class DecisionStabilizer(Node):

    def __init__(self):

        super().__init__('decision_stabilizer')


        self.declare_parameter(
            'input_topic',
            '/navigation_decision'
        )

        self.declare_parameter(
            'output_topic',
            '/stable_navigation_decision'
        )

        

        self.declare_parameter(
            'side_switch_margin',
            0.10
        )

        
        self.declare_parameter(
            'max_heading_change_deg',
            10.0
        )

        self.declare_parameter(
            'heading_change_threshold_deg',
            5.0
        )

        self.declare_parameter(
            'side_switch_persistence',
            5
        )


        self.declare_parameter(
            'min_switch_confidence',
            0.30
        )


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

        self.side_switch_margin = float(
            self.get_parameter(
                'side_switch_margin'
            ).value
        )

        self.max_heading_change = float(
            self.get_parameter(
                'max_heading_change_deg'
            ).value
        )

        self.heading_change_threshold = float(
            self.get_parameter(
                'heading_change_threshold_deg'
            ).value
        )

        self.side_switch_persistence = int(
            self.get_parameter(
                'side_switch_persistence'
            ).value
        )

        self.min_switch_confidence = float(
            self.get_parameter(
                'min_switch_confidence'
            ).value
        )

        # -----------------------------------------------------
        # Protect against invalid parameter values.
        # -----------------------------------------------------

        if self.side_switch_persistence < 1:
            self.side_switch_persistence = 1

        self.min_switch_confidence = max(
            0.0,
            min(
                1.0,
                self.min_switch_confidence
            )
        )


        self.previous_heading = 0.0
        self.previous_side = 'NONE'
        self.has_previous_decision = False


        self.pending_side = 'NONE'
        self.pending_heading = 0.0
        self.pending_count = 0


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
            'Decision stabilizer started'
        )

        self.get_logger().info(
            'Input layout: '
            '[best_heading, best_score, left_score, '
            'right_score, front_clearance, path_blocked, '
            'direction_safe, confidence, decision_ambiguous]'
        )

        self.get_logger().info(
            f'Side switch margin: '
            f'{self.side_switch_margin:.3f}'
        )

        self.get_logger().info(
            f'Max same-side heading change: '
            f'{self.max_heading_change:.1f}°'
        )

        self.get_logger().info(
            f'Side switch persistence: '
            f'{self.side_switch_persistence} cycles'
        )

        self.get_logger().info(
            f'Minimum switch confidence: '
            f'{self.min_switch_confidence:.3f}'
        

    def callback(
        self,
        msg: Float32MultiArray
    ) -> None:

        data = list(msg.data)


        if len(data) < 9:

            self.get_logger().warning(
                'Navigation decision must contain at least 9 values.'
            )

            return

        # =====================================================
        # READ PHASE-2C VALUES
        # =====================================================

        candidate_heading = float(data[0])
        candidate_score = float(data[1])

        left_score = float(data[2])
        right_score = float(data[3])

        front_clearance = float(data[4])

        path_blocked = bool(data[5])
        direction_safe = bool(data[6])

        confidence = float(data[7])

        decision_ambiguous = bool(data[8])

        # =====================================================
        # CASE 1 — CLEAR PATH
        # =====================================================

        if not path_blocked:

            stable_heading = 0.0

            stable_safe = direction_safe
            stable_ambiguous = False

            stable_status = 0.0
            # 0 = CLEAR

            self.previous_heading = 0.0
            self.previous_side = 'NONE'
            self.has_previous_decision = True

            # Clear any pending switch.
            self.reset_pending_switch()

            self.publish_decision(
                stable_heading,
                candidate_score,
                left_score,
                right_score,
                front_clearance,
                path_blocked,
                stable_safe,
                confidence,
                stable_ambiguous,
                stable_status
            )

            return

        # =====================================================
        # CASE 2 — BLOCKED BUT NOT DECISIVE
        # =====================================================

        if (
            not direction_safe
            or decision_ambiguous
        ):

            # An ambiguous or unsafe decision is not allowed
            # to accumulate persistence.
            self.reset_pending_switch()

            if (
                self.has_previous_decision
                and self.previous_side != 'NONE'
            ):

                stable_heading = self.previous_heading

                stable_status = 2.0
                # 2 = HOLD_PREVIOUS

                stable_safe = True

            else:

                stable_heading = 0.0

                stable_status = 3.0
                # 3 = WAIT_FOR_DECISION

                stable_safe = False

            self.publish_decision(
                stable_heading,
                candidate_score,
                left_score,
                right_score,
                front_clearance,
                path_blocked,
                stable_safe,
                confidence,
                True,
                stable_status
            )

            return

        # =====================================================
        # CASE 3 — BLOCKED + SAFE + DECISIVE
        # =====================================================

        candidate_side = self.get_side(
            candidate_heading
        )

        # =====================================================
        # FIRST DECISIVE DECISION
        # =====================================================

        if (
            not self.has_previous_decision
            or self.previous_side == 'NONE'
        ):

            stable_heading = candidate_heading

            stable_status = 1.0
            # 1 = NEW_AVOIDANCE

            self.reset_pending_switch()


        elif candidate_side == self.previous_side:

            # Candidate agrees with current side.
            self.reset_pending_switch()

            stable_heading = self.limit_heading_change(
                self.previous_heading,
                candidate_heading
            )

            stable_status = 4.0
            # 4 = CONTINUE_SAME_SIDE


        elif candidate_side == 'NONE':

            self.reset_pending_switch()

            stable_heading = self.previous_heading

            stable_status = 6.0
            # 6 = HOLD_SIDE


        else:

            current_side_score = self.get_side_score(
                candidate_side,
                left_score,
                right_score
            )

            previous_side_score = self.get_side_score(
                self.previous_side,
                left_score,
                right_score
            )

            score_advantage = (
                current_side_score
                - previous_side_score
            )

            # -------------------------------------------------
            # Diagnostic information
            # -------------------------------------------------

            self.get_logger().info(
                f'SIDE CHANGE REQUEST: '
                f'{self.previous_side} -> {candidate_side} | '
                f'previous_score={previous_side_score:.3f} | '
                f'new_score={current_side_score:.3f} | '
                f'advantage={score_advantage:.3f} | '
                f'confidence={confidence:.3f}'
            )

            # =================================================
            # GATE 1 — SCORE ADVANTAGE
            # =================================================

            if (
                score_advantage
                < self.side_switch_margin
            ):

                self.reset_pending_switch()

                stable_heading = self.previous_heading

                stable_status = 6.0
                # 6 = HOLD_SIDE

            # =================================================
            # GATE 2 — CONFIDENCE
            # =================================================

            elif (
                confidence
                < self.min_switch_confidence
            ):

             

                self.reset_pending_switch()

                stable_heading = self.previous_heading

                stable_status = 6.0
                # 6 = HOLD_SIDE

                self.get_logger().info(
                    f'SWITCH REJECTED BY CONFIDENCE: '
                    f'{confidence:.3f} < '
                    f'{self.min_switch_confidence:.3f}'
                )


            else:

           

                if self.pending_side != candidate_side:

                    self.pending_side = candidate_side
                    self.pending_heading = candidate_heading
                    self.pending_count = 1

                else:

                    self.pending_count += 1

                    self.pending_heading = candidate_heading

                self.get_logger().info(
                    f'PENDING SIDE SWITCH: '
                    f'{self.previous_side} -> '
                    f'{candidate_side} | '
                    f'persistence='
                    f'{self.pending_count}/'
                    f'{self.side_switch_persistence}'
                )

                # -------------------------------------------------
                # Persistence requirement satisfied
                # -------------------------------------------------

                if (
                    self.pending_count
                    >= self.side_switch_persistence
                ):

                    # -------------------------------------------------
                    # Genuine side change.
                    #
                    # Do NOT apply the same-side heading limiter.
                    # -------------------------------------------------

                    stable_heading = self.pending_heading

                    stable_status = 5.0
                    # 5 = SWITCH_SIDE

                    self.get_logger().info(
                        f'SIDE SWITCH ACCEPTED: '
                        f'{self.previous_side} -> '
                        f'{candidate_side}'
                    )

                    self.reset_pending_switch()

                else:

                    stable_heading = self.previous_heading

                    stable_status = 6.0
                    # 6 = HOLD_SIDE



        self.previous_heading = stable_heading

        self.previous_side = self.get_side(
            stable_heading
        )

        self.has_previous_decision = True

        

        self.publish_decision(
            stable_heading,
            candidate_score,
            left_score,
            right_score,
            front_clearance,
            path_blocked,
            True,
            confidence,
            False,
            stable_status
        )


    def get_side(
        self,
        heading: float
    ) -> str:

        if (
            heading
            > self.heading_change_threshold
        ):

            return 'LEFT'

        if (
            heading
            < -self.heading_change_threshold
        ):

            return 'RIGHT'

        return 'NONE'


    def get_side_score(
        self,
        side: str,
        left_score: float,
        right_score: float
    ) -> float:

        if side == 'LEFT':
            return left_score

        if side == 'RIGHT':
            return right_score

        return 0.0


    def reset_pending_switch(self) -> None:

        self.pending_side = 'NONE'
        self.pending_heading = 0.0
        self.pending_count = 0


    def limit_heading_change(
        self,
        previous_heading: float,
        requested_heading: float
    ) -> float:

        difference = (
            requested_heading
            - previous_heading
        )

        if (
            abs(difference)
            <= self.max_heading_change
        ):

            return requested_heading

        direction = (
            1.0
            if difference > 0.0
            else -1.0
        )

        return (
            previous_heading
            + direction
            * self.max_heading_change
        )


    def publish_decision(
        self,
        heading: float,
        score: float,
        left_score: float,
        right_score: float,
        front_clearance: float,
        path_blocked: bool,
        direction_safe: bool,
        confidence: float,
        ambiguous: bool,
        stable_status: float
    ) -> None:

        output = Float32MultiArray()

        output.data = [
            float(heading),
            float(score),
            float(left_score),
            float(right_score),
            float(front_clearance),
            float(path_blocked),
            float(direction_safe),
            float(confidence),
            float(ambiguous),
            float(stable_status)
        ]

        self.publisher.publish(output)


        status_names = {
            0.0: 'CLEAR',
            1.0: 'NEW_AVOIDANCE',
            2.0: 'HOLD_PREVIOUS',
            3.0: 'WAIT_FOR_DECISION',
            4.0: 'CONTINUE_SAME_SIDE',
            5.0: 'SWITCH_SIDE',
            6.0: 'HOLD_SIDE',
        }

        status_name = status_names.get(
            stable_status,
            'UNKNOWN'
        )

        self.get_logger().info(
            f'stable_heading={heading:+.1f}° | '
            f'blocked={path_blocked} | '
            f'safe={direction_safe} | '
            f'confidence={confidence:.3f} | '
            f'ambiguous={ambiguous} | '
            f'status={status_name}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = DecisionStabilizer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()