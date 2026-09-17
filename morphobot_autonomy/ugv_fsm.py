#!/usr/bin/env python3

import rclpy

from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray



class FSMState(Enum):

    CRUISE = 0
    CAUTION = 1
    AVOID = 2
    VERIFY = 3
    RECOVERY = 4
    STOP = 5


class UGVFSM(Node):

    def __init__(self):

        super().__init__('ugv_fsm')



        self.declare_parameter(
            'input_topic',
            '/stable_navigation_decision'
        )

        self.declare_parameter(
            'output_topic',
            '/ugv_fsm_state'
        )

        

        self.declare_parameter(
            'verify_clearance',
            2.0
        )


        self.declare_parameter(
            'minimum_forward_clearance_for_verify',
            2.20
        )

       

        self.declare_parameter(
            'verify_heading_tolerance_deg',
            10.0
        )

        self.declare_parameter(
            'verify_cycles',
            3
        )


        self.declare_parameter(
            'minimum_avoid_confidence',
            0.30
        )

        # Minimum heading magnitude required to enter AVOID.

        self.declare_parameter(
            'minimum_avoid_heading_deg',
            5.0
        )

        # =====================================================
        # AVOIDANCE SIDE LOCK
        # =====================================================

        # Once AVOID begins, LEFT or RIGHT is locked.
        #
        # This prevents:
        #
        # LEFT -> RIGHT
        #
        # or
        #
        # RIGHT -> LEFT
        #
        # switching in the middle of one obstacle maneuver.

        self.declare_parameter(
            'avoidance_side_lock',
            True
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

        self.verify_clearance = float(
            self.get_parameter(
                'verify_clearance'
            ).value
        )

        self.minimum_forward_clearance_for_verify = float(
            self.get_parameter(
                'minimum_forward_clearance_for_verify'
            ).value
        )

        self.verify_heading_tolerance = float(
            self.get_parameter(
                'verify_heading_tolerance_deg'
            ).value
        )

        self.verify_cycles_required = int(
            self.get_parameter(
                'verify_cycles'
            ).value
        )

        self.minimum_avoid_confidence = float(
            self.get_parameter(
                'minimum_avoid_confidence'
            ).value
        )

        self.minimum_avoid_heading = float(
            self.get_parameter(
                'minimum_avoid_heading_deg'
            ).value
        )

        self.avoidance_side_lock = bool(
            self.get_parameter(
                'avoidance_side_lock'
            ).value
        )


        if self.verify_clearance < 0.0:

            self.verify_clearance = 2.0

        if self.minimum_forward_clearance_for_verify < 0.0:

            self.minimum_forward_clearance_for_verify = 2.20

        # Never permit the Phase 4 verify threshold to be
        # lower than the normal verify clearance.

        self.minimum_forward_clearance_for_verify = max(
            self.verify_clearance,
            self.minimum_forward_clearance_for_verify
        )

        if self.verify_heading_tolerance < 0.0:

            self.verify_heading_tolerance = 10.0

        if self.verify_cycles_required < 1:

            self.verify_cycles_required = 1

        if self.minimum_avoid_confidence < 0.0:

            self.minimum_avoid_confidence = 0.30

        if self.minimum_avoid_heading < 0.0:

            self.minimum_avoid_heading = 5.0


        self.state = FSMState.CRUISE

        self.verify_counter = 0

        self.last_heading = 0.0

       

        self.locked_avoid_side = 0

        # Last meaningful non-zero heading while avoiding.

        self.last_avoid_heading = 0.0


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


        self.get_logger().info(
            '=================================================='
        )

        self.get_logger().info(
            'UGV FSM started'
        )

        self.get_logger().info(
            f'Input topic: {self.input_topic}'
        )

        self.get_logger().info(
            f'Output topic: {self.output_topic}'
        )

        self.get_logger().info(
            f'Verify clearance: '
            f'{self.verify_clearance:.2f} m'
        )

        self.get_logger().info(
            f'Minimum forward clearance for VERIFY: '
            f'{self.minimum_forward_clearance_for_verify:.2f} m'
        )

        self.get_logger().info(
            f'Verify heading tolerance: '
            f'{self.verify_heading_tolerance:.1f}°'
        )

        self.get_logger().info(
            f'Verify cycles: '
            f'{self.verify_cycles_required}'
        )

        self.get_logger().info(
            f'Minimum avoidance heading: '
            f'{self.minimum_avoid_heading:.1f}°'
        )

        self.get_logger().info(
            f'Confidence parameter retained for monitoring: '
            f'{self.minimum_avoid_confidence:.3f}'
        )

        self.get_logger().info(
            'CAUTION -> AVOID uses direction_safe + '
            'ambiguity + heading'
        )

        self.get_logger().info(
            f'Avoidance side lock: '
            f'{self.avoidance_side_lock}'
        )

        self.get_logger().info(
            '=================================================='
        )


    def callback(
        self,
        msg: Float32MultiArray
    ) -> None:

        data = list(
            msg.data
        )

        if len(data) < 10:

            self.get_logger().warning(
                'Stable navigation decision must contain '
                'at least 10 values.'
            )

            return

        heading = float(
            data[0]
        )

        front_clearance = max(
            0.0,
            float(
                data[4]
            )
        )

        path_blocked = bool(
            data[5]
        )

        direction_safe = bool(
            data[6]
        )

        confidence = max(
            0.0,
            min(
                1.0,
                float(
                    data[7]
                )
            )
        )

        ambiguous = bool(
            data[8]
        )

        stability_status = int(
            data[9]
        )

        # =====================================================
        # MEMORY
        # =====================================================

        self.last_heading = heading

        # =====================================================
        # FSM
        # =====================================================

        if self.state == FSMState.CRUISE:

            self.handle_cruise(
                heading,
                front_clearance,
                path_blocked,
                direction_safe,
                confidence,
                ambiguous
            )

        elif self.state == FSMState.CAUTION:

            self.handle_caution(
                heading,
                path_blocked,
                direction_safe,
                confidence,
                ambiguous
            )

        elif self.state == FSMState.AVOID:

            heading = self.handle_avoid(
                heading,
                front_clearance,
                path_blocked
            )

        elif self.state == FSMState.VERIFY:

            self.handle_verify(
                front_clearance,
                path_blocked
            )

        elif self.state == FSMState.RECOVERY:

            self.handle_recovery(
                path_blocked,
                direction_safe,
                confidence,
                ambiguous
            )

        elif self.state == FSMState.STOP:

            # STOP is terminal for the current phase.

            pass

        # =====================================================
        # PUBLISH FSM STATE
        # =====================================================

        self.publish_state(
            heading,
            front_clearance,
            path_blocked,
            direction_safe,
            confidence,
            ambiguous,
            stability_status
        )

    # =========================================================
    # CRUISE
    # =========================================================

    def handle_cruise(
        self,
        heading,
        front_clearance,
        path_blocked,
        direction_safe,
        confidence,
        ambiguous
    ):

        # Any forward blockage moves the robot to CAUTION.

        if path_blocked:

            self.transition(
                FSMState.CAUTION
            )

    # =========================================================
    # CAUTION
    # =========================================================

    def handle_caution(
        self,
        heading,
        path_blocked,
        direction_safe,
        confidence,
        ambiguous
    ):

        # =====================================================
        # OBSTACLE DISAPPEARED
        # =====================================================

        if not path_blocked:

            self.transition(
                FSMState.CRUISE
            )

            return

        # =====================================================
        # VALID AVOIDANCE DIRECTION
        # =====================================================

        # Confidence is NOT a hard safety gate.
        #
        # direction_safe:
        #     a usable corridor exists.
        #
        # ambiguous:
        #     decision is unresolved.
        #
        # heading:
        #     enough steering is available to perform
        #     an actual maneuver.

        if (
            direction_safe
            and
            not ambiguous
            and
            abs(heading)
            >
            self.minimum_avoid_heading
        ):

            self.get_logger().info(
                f'Avoidance direction accepted | '
                f'heading={heading:+.1f}° | '
                f'confidence={confidence:.3f}'
            )

            # Lock LEFT or RIGHT before entering AVOID.

            self.lock_avoidance_side(
                heading
            )

            self.transition(
                FSMState.AVOID
            )

            return

        if (
            not direction_safe
            or
            ambiguous
        ):

            self.get_logger().warning(
                f'No safe avoidance direction | '
                f'heading={heading:+.1f}° | '
                f'confidence={confidence:.3f} | '
                f'ambiguous={ambiguous}'
            )

            self.transition(
                FSMState.STOP
            )


    def handle_avoid(
        self,
        heading,
        front_clearance,
        path_blocked
    ):
        """
        Phase 4 obstacle-bypass transition logic.

        The robot remains in AVOID while it is still performing
        the bypass.

        AVOID -> VERIFY requires all three:

            1. sufficient forward clearance
            2. heading nearly straight
            3. forward path no longer blocked
        """


        heading = (
            self.apply_avoidance_side_lock(
                heading
            )
        )

       

        forward_clearance_required = max(
            self.verify_clearance,
            self.minimum_forward_clearance_for_verify
        )

        obstacle_clear = (
            front_clearance
            >=
            forward_clearance_required
        )

        heading_aligned = (
            abs(heading)
            <=
            self.verify_heading_tolerance
        )

        path_is_clear = (
            not path_blocked
        )


        if (
            obstacle_clear
            and
            heading_aligned
            and
            path_is_clear
        ):

            self.verify_counter = 0

            self.get_logger().info(
                '=================================================='
            )

            self.get_logger().info(
                'AVOIDANCE MANEUVER CLEAR'
            )

            self.get_logger().info(
                f'Front clearance={front_clearance:.2f} m'
            )

            self.get_logger().info(
                f'Heading={heading:+.1f}°'
            )

            self.get_logger().info(
                'Transitioning AVOID -> VERIFY'
            )

            self.get_logger().info(
                '=================================================='
            )

            self.transition(
                FSMState.VERIFY
            )

        return heading

    # =========================================================
    # VERIFY
    # =========================================================

    def handle_verify(
        self,
        front_clearance,
        path_blocked
    ):

        # =====================================================
        # OBSTACLE RETURNED
        # =====================================================

        if (
            front_clearance
            <
            self.verify_clearance
            or
            path_blocked
        ):

            self.verify_counter = 0

            self.get_logger().warning(
                'VERIFY failed: obstacle detected again'
            )

            self.transition(
                FSMState.AVOID
            )

            return

        # =====================================================
        # CLEAR CONDITION PERSISTS
        # =====================================================

        self.verify_counter += 1

        self.get_logger().info(
            f'VERIFY clear cycle '
            f'{self.verify_counter}/'
            f'{self.verify_cycles_required}'
        )

        if (
            self.verify_counter
            >=
            self.verify_cycles_required
        ):

            self.verify_counter = 0


            self.clear_avoidance_side_lock()

            self.get_logger().info(
                'Obstacle bypass verified -> CRUISE'
            )

            self.transition(
                FSMState.CRUISE
            )


    def handle_recovery(
        self,
        path_blocked,
        direction_safe,
        confidence,
        ambiguous
    ):

        # =====================================================
        # PHASE 5 PLACEHOLDER
        # =====================================================
        #
        # Actual odometry-based stuck detection and recovery
        # will be implemented in Phase 5.
        #
        # Planned recovery actions:
        #
        #   - detect no-motion condition
        #   - reverse
        #   - controlled turn
        #   - retry avoidance
        #   - recovery timeout
        #   - retry limit
        #
        # =====================================================

        if not path_blocked:

            self.transition(
                FSMState.CRUISE
            )

            return

        if (
            direction_safe
            and
            not ambiguous
        ):

            self.transition(
                FSMState.CAUTION
            )

    # =========================================================
    # LOCK AVOIDANCE SIDE
    # =========================================================

    def lock_avoidance_side(
        self,
        heading
    ):

        if not self.avoidance_side_lock:

            return

        # Ignore an effectively straight heading.

        if (
            abs(heading)
            <=
            self.minimum_avoid_heading
        ):

            return

        # =====================================================
        # + heading = LEFT
        # - heading = RIGHT
        # =====================================================

        self.locked_avoid_side = (
            1
            if heading > 0.0
            else -1
        )

        self.last_avoid_heading = (
            heading
        )

        side_name = (
            'LEFT'
            if self.locked_avoid_side > 0
            else 'RIGHT'
        )

        self.get_logger().info(
            f'AVOIDANCE SIDE LOCKED -> '
            f'{side_name} | '
            f'heading={heading:+.1f}°'
        )


    def apply_avoidance_side_lock(
        self,
        heading
    ):
        """
        Maintain the selected avoidance side.

        Allowed:

            LEFT:
                +30 -> +25 -> +15 -> +5 -> 0

            RIGHT:
                -30 -> -25 -> -15 -> -5 -> 0

        Not allowed:

            LEFT -> RIGHT

            RIGHT -> LEFT
        """

        # =====================================================
        # LOCK DISABLED
        # =====================================================

        if not self.avoidance_side_lock:

            return heading

        # =====================================================
        # NO LOCK
        # =====================================================

        if self.locked_avoid_side == 0:

            return heading

        # =====================================================
        # ALLOW STRAIGHTENING
        # =====================================================

        # This is an important Phase 4 change.
        #
        # A locked LEFT maneuver is allowed to approach 0°.
        # A locked RIGHT maneuver is also allowed to approach 0°.
        #
        # This is necessary for the robot to straighten after
        # bypassing the obstacle.

        if (
            abs(heading)
            <=
            self.minimum_avoid_heading
        ):

            return heading

        # =====================================================
        # SAME SIDE
        # =====================================================

        if (
            heading > 0.0
            and
            self.locked_avoid_side > 0
        ):

            self.last_avoid_heading = (
                heading
            )

            return heading

        if (
            heading < 0.0
            and
            self.locked_avoid_side < 0
        ):

            self.last_avoid_heading = (
                heading
            )

            return heading

        # =====================================================
        # OPPOSITE SIDE
        # =====================================================

        # Example:
        #
        # Locked LEFT (+)
        # New heading RIGHT (-)
        #
        # Reject the side change.

        side_name = (
            'LEFT'
            if self.locked_avoid_side > 0
            else 'RIGHT'
        )

        if self.last_avoid_heading != 0.0:

            self.get_logger().warning(
                f'Opposite-side heading '
                f'{heading:+.1f}° rejected | '
                f'locked={side_name} | '
                f'maintaining='
                f'{self.last_avoid_heading:+.1f}°'
            )

            return (
                self.last_avoid_heading
            )


        return (
            abs(heading)
            *
            self.locked_avoid_side
        )

    def clear_avoidance_side_lock(
        self
    ):

        if self.locked_avoid_side != 0:

            side_name = (
                'LEFT'
                if self.locked_avoid_side > 0
                else 'RIGHT'
            )

            self.get_logger().info(
                f'AVOIDANCE SIDE UNLOCKED -> '
                f'{side_name}'
            )

        self.locked_avoid_side = 0

        self.last_avoid_heading = 0.0

    # =========================================================
    # STATE TRANSITION
    # =========================================================

    def transition(
        self,
        new_state: FSMState
    ):

        if new_state == self.state:

            return

        old_state = self.state

        self.state = new_state

        # Reset verification counter on every state change.

        self.verify_counter = 0

        self.get_logger().info(
            f'FSM TRANSITION: '
            f'{old_state.name} -> '
            f'{new_state.name}'
        )

    # =========================================================
    # PUBLISH
    # =========================================================

    def publish_state(
        self,
        heading,
        front_clearance,
        path_blocked,
        direction_safe,
        confidence,
        ambiguous,
        stability_status
    ):

        output = Float32MultiArray()

        # =====================================================
        # OUTPUT LAYOUT
        # =====================================================
        #
        # [0] FSM state
        # [1] heading
        # [2] front_clearance
        # [3] path_blocked
        # [4] direction_safe
        # [5] confidence
        # [6] ambiguous
        # [7] stability_status
        #
        # FSM states:
        #
        # 0 = CRUISE
        # 1 = CAUTION
        # 2 = AVOID
        # 3 = VERIFY
        # 4 = RECOVERY
        # 5 = STOP
        #
        # =====================================================

        output.data = [
            float(
                self.state.value
            ),

            float(
                heading
            ),

            float(
                front_clearance
            ),

            float(
                path_blocked
            ),

            float(
                direction_safe
            ),

            float(
                confidence
            ),

            float(
                ambiguous
            ),

            float(
                stability_status
            )
        ]

        self.publisher.publish(
            output
        )

        self.get_logger().info(
            f'STATE={self.state.name} | '
            f'heading={heading:+.1f}° | '
            f'front={front_clearance:.2f} m | '
            f'blocked={path_blocked} | '
            f'safe={direction_safe} | '
            f'confidence={confidence:.3f} | '
            f'ambiguous={ambiguous}'
        )



def main(
    args=None
):

    rclpy.init(
        args=args
    )

    node = UGVFSM()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()