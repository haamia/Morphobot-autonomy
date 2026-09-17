#!/usr/bin/env python3

import time
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class FSMState(Enum):
    CRUISE = 0
    CAUTION = 1
    AVOID = 2
    VERIFY = 3
    RECOVERY = 4
    STOP = 5


class ManeuverPhase(Enum):
    IDLE = 0
    TURN = 1
    BYPASS = 2
    STRAIGHTEN = 3
    STRAIGHT_HOLD = 4


class AdaptiveController(Node):
    """
    Phase 4 UGV adaptive controller.

    The avoidance maneuver is deliberately latched:

        AVOID entry
            -> choose LEFT/RIGHT once
            -> TURN with a guaranteed non-zero angular command
            -> BYPASS while keeping the same side
            -> STRAIGHTEN once the forward corridor is clear
            -> STRAIGHT_HOLD until the FSM leaves AVOID

    This version intentionally does NOT use an internally integrated yaw
    estimate. That estimate caused the previous controller to enter BYPASS
    with angular velocity equal to zero while the robot was still facing the
    obstacle.

    ONLY CHANGE FROM THE PREVIOUS WORKING VERSION:
    BYPASS now has a hard maximum duration (maximum_bypass_time_sec). Its
    normal exit condition (front_clearance >= forward_clear_for_straight_m
    for forward_clear_cycles consecutive cycles) still works exactly as
    before and is checked first. But while skirting close to an obstacle,
    front_clearance can stay below that threshold for a long time, so that
    condition alone was letting BYPASS hold a constant nonzero angular
    velocity indefinitely -- which, sustained long enough, rotates the robot
    far enough to look like it turned back toward its start. The time cap
    guarantees BYPASS always ends and the robot always goes straight
    (angular = 0.0) after at most maximum_bypass_time_sec seconds, no matter
    what clearance reads.

    Everything else -- TURN, no counter-steering, STRAIGHT_HOLD staying at
    angular = 0.0 -- is unchanged.

    FSM input layout:
        [0] state
        [1] heading_deg
        [2] front_clearance_m
        [3] path_blocked
        [4] direction_safe
        [5] confidence
        [6] ambiguous
        [7] stability_status

    Output:
        /cmd_vel geometry_msgs/msg/Twist
    """

    def __init__(self):
        super().__init__('adaptive_controller')

        self.declare_parameter('input_topic', '/ugv_fsm_state')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('fsm_timeout_sec', 2.0)

        self.declare_parameter('cruise_speed_mps', 0.45)
        self.declare_parameter('caution_speed_mps', 0.20)
        self.declare_parameter('avoid_speed_mps', 0.16)
        self.declare_parameter('verify_speed_mps', 0.12)
        self.declare_parameter('straight_hold_speed_mps', 0.16)

        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('max_angular_speed_radps', 0.90)

        self.declare_parameter('minimum_avoid_heading_deg', 5.0)

        # Guaranteed turn: non-zero angular command for a fixed duration.
        self.declare_parameter('turn_angular_speed_radps', 0.90)
        self.declare_parameter('turn_duration_sec', 0.60)

        # Continue moving around the obstacle with the same side.
        self.declare_parameter('bypass_angular_speed_radps', 0.45)
        self.declare_parameter('minimum_bypass_time_sec', 0.60)
        # NEW: hard ceiling on BYPASS duration. This is the only behavioral
        # change in this file. Must be >= minimum_bypass_time_sec.
        self.declare_parameter('maximum_bypass_time_sec', 1.4)

        # Straighten with a short counter-steering command.
        self.declare_parameter('straighten_angular_speed_radps', 0.30)
        self.declare_parameter('straighten_duration_sec', 0.40)

        # Forward corridor must be clear before straightening begins.
        self.declare_parameter('forward_clear_for_straight_m', 2.40)
        self.declare_parameter('forward_clear_cycles', 4)


        self.declare_parameter('emergency_stop_distance_m', 0.40)
        self.declare_parameter('turn_only_distance_m', 0.80)
        self.declare_parameter('slow_down_distance_m', 2.00)

        self.declare_parameter('linear_acceleration_mps2', 0.60)
        self.declare_parameter('linear_deceleration_mps2', 1.20)
        self.declare_parameter('angular_acceleration_radps2', 4.0)
        self.declare_parameter('angular_deceleration_radps2', 5.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.fsm_timeout_sec = float(self.get_parameter('fsm_timeout_sec').value)

        self.cruise_speed = float(self.get_parameter('cruise_speed_mps').value)
        self.caution_speed = float(self.get_parameter('caution_speed_mps').value)
        self.avoid_speed = float(self.get_parameter('avoid_speed_mps').value)
        self.verify_speed = float(self.get_parameter('verify_speed_mps').value)
        self.straight_hold_speed = float(
            self.get_parameter('straight_hold_speed_mps').value
        )

        self.steering_gain = float(self.get_parameter('steering_gain').value)
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed_radps').value
        )

        self.minimum_avoid_heading = float(
            self.get_parameter('minimum_avoid_heading_deg').value
        )
        self.turn_angular_speed = float(
            self.get_parameter('turn_angular_speed_radps').value
        )
        self.turn_duration = float(
            self.get_parameter('turn_duration_sec').value
        )
        self.bypass_angular_speed = float(
            self.get_parameter('bypass_angular_speed_radps').value
        )
        self.minimum_bypass_time = float(
            self.get_parameter('minimum_bypass_time_sec').value
        )
        self.maximum_bypass_time = float(
            self.get_parameter('maximum_bypass_time_sec').value
        )
        self.straighten_angular_speed = float(
            self.get_parameter('straighten_angular_speed_radps').value
        )
        self.straighten_duration = float(
            self.get_parameter('straighten_duration_sec').value
        )
        self.forward_clear_for_straight = float(
            self.get_parameter('forward_clear_for_straight_m').value
        )
        self.forward_clear_cycles_required = int(
            self.get_parameter('forward_clear_cycles').value
        )

        self.emergency_stop_distance = float(
            self.get_parameter('emergency_stop_distance_m').value
        )
        self.turn_only_distance = float(
            self.get_parameter('turn_only_distance_m').value
        )
        self.slow_down_distance = float(
            self.get_parameter('slow_down_distance_m').value
        )

        self.linear_acceleration = float(
            self.get_parameter('linear_acceleration_mps2').value
        )
        self.linear_deceleration = float(
            self.get_parameter('linear_deceleration_mps2').value
        )
        self.angular_acceleration = float(
            self.get_parameter('angular_acceleration_radps2').value
        )
        self.angular_deceleration = float(
            self.get_parameter('angular_deceleration_radps2').value
        )

        self.validate_parameters()

        self.current_state = FSMState.STOP
        self.previous_state = FSMState.STOP

        self.heading_deg = 0.0
        self.front_clearance = 0.0
        self.path_blocked = True
        self.direction_safe = False
        self.confidence = 0.0
        self.ambiguous = True
        self.stability_status = 3
        self.last_fsm_time = 0.0


        self.maneuver_phase = ManeuverPhase.IDLE
        self.maneuver_side = 0       # +1 LEFT, -1 RIGHT
        self.phase_start_time = 0.0
        self.forward_clear_count = 0

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0

        self.subscriber = self.create_subscription(
            Float32MultiArray,
            self.input_topic,
            self.fsm_callback,
            10,
        )

        self.publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop,
        )

        self.get_logger().info('Adaptive controller started')
        self.get_logger().info(f'Input topic: {self.input_topic}')
        self.get_logger().info(f'Output topic: {self.cmd_vel_topic}')
        self.get_logger().info(
            f'TURN: {self.turn_angular_speed:.2f} rad/s '
            f'for {self.turn_duration:.2f} s'
        )
        self.get_logger().info(
            f'BYPASS: {self.bypass_angular_speed:.2f} rad/s | '
            f'min {self.minimum_bypass_time:.2f} s | '
            f'MAX {self.maximum_bypass_time:.2f} s (hard cap, NEW)'
        )
        self.get_logger().info(
            'Maneuver policy: TURN -> BYPASS (time-capped) -> STRAIGHT_HOLD '
            '(no counter-steer)'
        )

    def validate_parameters(self):
        if self.control_rate_hz <= 0.0:
            self.control_rate_hz = 20.0

        if self.fsm_timeout_sec <= 0.0:
            self.fsm_timeout_sec = 2.0

        self.turn_duration = max(0.20, self.turn_duration)
        self.minimum_bypass_time = max(0.20, self.minimum_bypass_time)
        # Hard ceiling must never be shorter than the minimum.
        self.maximum_bypass_time = max(
            self.minimum_bypass_time,
            self.maximum_bypass_time,
        )
        self.straighten_duration = max(0.10, self.straighten_duration)

        self.max_angular_speed = max(0.20, self.max_angular_speed)
        self.turn_angular_speed = min(
            self.max_angular_speed,
            max(0.40, self.turn_angular_speed),
        )
        self.bypass_angular_speed = min(
            self.max_angular_speed,
            max(0.10, self.bypass_angular_speed),
        )
        self.straighten_angular_speed = min(
            self.max_angular_speed,
            max(0.05, self.straighten_angular_speed),
        )

        self.emergency_stop_distance = max(
            0.20,
            self.emergency_stop_distance,
        )
        self.turn_only_distance = max(
            self.emergency_stop_distance + 0.05,
            self.turn_only_distance,
        )

        self.forward_clear_cycles_required = max(
            1,
            self.forward_clear_cycles_required,
        )

    # =============================================================
    # FSM CALLBACK
    # =============================================================
    def fsm_callback(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)

        if len(data) < 8:
            self.get_logger().warning(
                'UGV FSM state must contain at least 8 values.'
            )
            return

        try:
            new_state = FSMState(int(data[0]))
        except ValueError:
            self.get_logger().error(
                f'Unknown FSM state: {int(data[0])}'
            )
            self.force_stop()
            return

        self.previous_state = self.current_state
        self.current_state = new_state

        self.heading_deg = float(data[1])
        self.front_clearance = max(0.0, float(data[2]))
        self.path_blocked = bool(data[3])
        self.direction_safe = bool(data[4])
        self.confidence = max(0.0, min(1.0, float(data[5])))
        self.ambiguous = bool(data[6])
        self.stability_status = int(data[7])
        self.last_fsm_time = time.monotonic()

        # Only the AVOID transition creates a new maneuver.
        if (
            self.current_state == FSMState.AVOID
            and self.previous_state != FSMState.AVOID
        ):
            self.start_new_maneuver()

        # Leaving AVOID clears the maneuver memory.
        elif (
            self.current_state != FSMState.AVOID
            and self.previous_state == FSMState.AVOID
        ):
            self.finish_maneuver()

        self.compute_target_command()

    # =============================================================
    # START MANEUVER
    # =============================================================
    def start_new_maneuver(self):
        # Use FSM heading only once to select the side.
        if self.heading_deg > self.minimum_avoid_heading:
            self.maneuver_side = +1
        elif self.heading_deg < -self.minimum_avoid_heading:
            self.maneuver_side = -1
        elif self.maneuver_side != 0:
            self.maneuver_side = self.maneuver_side
        else:
            self.maneuver_side = +1

        self.maneuver_phase = ManeuverPhase.TURN
        self.phase_start_time = time.monotonic()
        self.forward_clear_count = 0

        side_name = 'LEFT' if self.maneuver_side > 0 else 'RIGHT'
        self.get_logger().info(
            f'NEW AVOID MANEUVER | side={side_name} | '
            f'fsm_heading={self.heading_deg:+.1f}° | '
            f'turn_w={self.maneuver_side * self.turn_angular_speed:+.2f}'
        )

    def finish_maneuver(self):
        old_phase = self.maneuver_phase
        self.maneuver_phase = ManeuverPhase.IDLE
        self.maneuver_side = 0
        self.phase_start_time = 0.0
        self.forward_clear_count = 0

        self.get_logger().info(
            f'AVOID COMPLETE | previous_phase={old_phase.name}'
        )

    def compute_target_command(self):
        linear = 0.0
        angular = 0.0

        # Global emergency condition.
        # Below this distance, stop completely.
        if self.front_clearance <= self.emergency_stop_distance:
            self.target_linear = 0.0
            self.target_angular = 0.0
            return

        if self.current_state == FSMState.CRUISE:
            linear = self.cruise_speed
            angular = self.calculate_normal_steering(self.heading_deg)
            linear *= self.calculate_clearance_speed_factor()

        elif self.current_state == FSMState.CAUTION:
            linear = self.caution_speed
            angular = self.calculate_normal_steering(self.heading_deg)
            linear *= self.calculate_clearance_speed_factor()

        elif self.current_state == FSMState.AVOID:
            linear, angular = self.compute_avoidance_command()

        elif self.current_state == FSMState.VERIFY:
            linear = self.verify_speed
            angular = self.calculate_normal_steering(self.heading_deg)
            linear *= self.calculate_clearance_speed_factor()

        elif self.current_state in (FSMState.RECOVERY, FSMState.STOP):
            linear = 0.0
            angular = 0.0

        self.target_linear = max(0.0, linear)
        self.target_angular = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular),
        )

    def compute_avoidance_command(self):
        if self.maneuver_side == 0:
            self.get_logger().warning(
                'AVOID without locked side; stopping.'
            )
            return 0.0, 0.0

        side = float(self.maneuver_side)
        elapsed = time.monotonic() - self.phase_start_time

        # ---------------------------------------------------------
        # TURN
        # ---------------------------------------------------------
        if self.maneuver_phase == ManeuverPhase.TURN:
            # GUARANTEED non-zero turn command.
            angular = side * self.turn_angular_speed

            # When too close, prioritize turning away over forward motion.
            if self.front_clearance <= self.turn_only_distance:
                linear = 0.0
            else:
                linear = min(
                    self.avoid_speed,
                    self.calculate_avoid_speed(),
                )

            if elapsed >= self.turn_duration:
                self.enter_phase(ManeuverPhase.BYPASS)

            return linear, angular

        # ---------------------------------------------------------
        # BYPASS
        # ---------------------------------------------------------
        if self.maneuver_phase == ManeuverPhase.BYPASS:
            # Keep the chosen side. Never search for a new direction here.
            angular = side * self.bypass_angular_speed

            if self.front_clearance <= self.turn_only_distance:
                linear = 0.0
            else:
                linear = min(
                    self.avoid_speed,
                    self.calculate_avoid_speed(),
                )

            if self.front_clearance >= self.forward_clear_for_straight:
                self.forward_clear_count += 1
            else:
                self.forward_clear_count = 0

            # Original exit condition: clearance confirmed clear.
            clearance_confirmed_clear = (
                elapsed >= self.minimum_bypass_time
                and self.forward_clear_count
                >= self.forward_clear_cycles_required
            )

         
            bypass_time_exceeded = elapsed >= self.maximum_bypass_time

            if clearance_confirmed_clear or bypass_time_exceeded:
                if bypass_time_exceeded and not clearance_confirmed_clear:
                    self.get_logger().info(
                        'BYPASS ended by time cap (clearance never '
                        f'confirmed clear) | elapsed={elapsed:.2f}s | '
                        f'front={self.front_clearance:.2f}m'
                    )
               
                self.enter_phase(ManeuverPhase.STRAIGHT_HOLD)

            return linear, angular

        if self.maneuver_phase == ManeuverPhase.STRAIGHTEN:
            # Kept only for backward compatibility with the enum.
            # NEVER counter-steer here.
            return self.straight_hold_speed, 0.0

        if self.maneuver_phase == ManeuverPhase.STRAIGHT_HOLD:
          
            if self.front_clearance <= self.turn_only_distance:
                self.get_logger().warning(
                    'STRAIGHT_HOLD re-encountered close obstacle '
                    f'(front={self.front_clearance:.2f} m) -> resuming BYPASS'
                )
                self.enter_phase(ManeuverPhase.BYPASS)
                return self.compute_avoidance_command()

            # Critical: do not restart avoidance while FSM is still AVOID.
            return self.straight_hold_speed, 0.0

        return 0.0, 0.0

    def enter_phase(self, new_phase):
        old_phase = self.maneuver_phase
        self.maneuver_phase = new_phase
        self.phase_start_time = time.monotonic()

        if new_phase == ManeuverPhase.BYPASS:
            self.forward_clear_count = 0

        if new_phase == ManeuverPhase.STRAIGHTEN:
            self.forward_clear_count = 0

        self.get_logger().info(
            f'MANEUVER PHASE | {old_phase.name} -> {new_phase.name}'
        )

    def calculate_avoid_speed(self):
        factor = self.calculate_clearance_speed_factor()

        # Keep a useful minimum speed when not in the turn-only region.
        factor = max(0.60, factor)
        return self.avoid_speed * factor

    def calculate_normal_steering(self, heading_deg):
        heading_rad = heading_deg * 3.141592653589793 / 180.0
        angular = self.steering_gain * heading_rad
        return max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular),
        )
    def calculate_clearance_speed_factor(self):
        distance = self.front_clearance

        if distance <= self.emergency_stop_distance:
            return 0.0

        if distance >= self.slow_down_distance:
            return 1.0

        denominator = self.slow_down_distance - self.emergency_stop_distance
        factor = (
            distance - self.emergency_stop_distance
        ) / denominator

        return max(0.0, min(1.0, factor))

    def control_loop(self):
        now = time.monotonic()
        dt = 1.0 / self.control_rate_hz

        timed_out = (
            self.last_fsm_time <= 0.0
            or (now - self.last_fsm_time) > self.fsm_timeout_sec
        )

        if timed_out:
            self.target_linear = 0.0
            self.target_angular = 0.0

            if self.last_fsm_time > 0.0:
                self.get_logger().warning(
                    'FSM command timeout - stopping robot'
                )

        self.current_linear = self.ramp_value(
            self.current_linear,
            self.target_linear,
            (
                self.linear_acceleration
                if self.target_linear >= self.current_linear
                else self.linear_deceleration
            ),
            dt,
        )

        self.current_angular = self.ramp_value(
            self.current_angular,
            self.target_angular,
            (
                self.angular_acceleration
                if self.target_angular >= self.current_angular
                else self.angular_deceleration
            ),
            dt,
        )

        cmd = Twist()
        cmd.linear.x = self.current_linear
        cmd.angular.z = self.current_angular
        self.publisher.publish(cmd)

        side_name = (
            'LEFT' if self.maneuver_side > 0
            else 'RIGHT' if self.maneuver_side < 0
            else 'NONE'
        )

        self.get_logger().info(
            f'STATE={self.current_state.name} | '
            f'PHASE={self.maneuver_phase.name} | '
            f'side={side_name} | '
            f'front={self.front_clearance:.2f} m | '
            f'target_v={self.target_linear:.3f} | '
            f'cmd_v={self.current_linear:.3f} | '
            f'target_w={self.target_angular:+.3f} | '
            f'cmd_w={self.current_angular:+.3f}'
        )
    def ramp_value(self, current, target, rate, dt):
        maximum_change = max(0.0, rate) * dt
        difference = target - current

        if abs(difference) <= maximum_change:
            return target

        if difference > 0.0:
            return current + maximum_change

        return current - maximum_change

    def force_stop(self):
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)



def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.force_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()