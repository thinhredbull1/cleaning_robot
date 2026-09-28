#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


class CmdVelMux:

    def __init__(self):

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/cmd_vel"
        )

        self.publish_rate = rospy.get_param(
            "~publish_rate",
            30.0
        )

        # -------------------------------------------------
        # Source configuration
        # -------------------------------------------------

        sources_config = rospy.get_param(
            "~sources",
            {}
        )

        self.sources = {}
        self.commands = {}

        for name, config in sources_config.items():

            topic = config["topic"]
            priority = int(config["priority"])
            timeout = float(
                config.get("timeout", 0.5)
            )

            self.sources[name] = {
                "topic": topic,
                "priority": priority,
                "timeout": timeout,
            }

            self.commands[name] = {
                "msg": None,
                "stamp": rospy.Time(0),
            }

            rospy.Subscriber(
                topic,
                Twist,
                self.cmd_callback,
                callback_args=name,
                queue_size=1
            )

            rospy.loginfo(
                "[cmd_vel_mux] "
                "source=%s topic=%s priority=%d timeout=%.2f",
                name,
                topic,
                priority,
                timeout
            )

        # -------------------------------------------------
        # Publisher
        # -------------------------------------------------

        self.cmd_vel_pub = rospy.Publisher(
            self.output_topic,
            Twist,
            queue_size=1
        )

        # -------------------------------------------------
        # Timer
        # -------------------------------------------------

        self.timer = rospy.Timer(
            rospy.Duration(
                1.0 / self.publish_rate
            ),
            self.timer_callback
        )

        rospy.loginfo(
            "[cmd_vel_mux] Started"
        )

        rospy.loginfo(
            "[cmd_vel_mux] Output topic: %s",
            self.output_topic
        )

    # -----------------------------------------------------
    # Subscriber callback
    # -----------------------------------------------------

    def cmd_callback(self, msg, source):

        self.commands[source]["msg"] = msg

        self.commands[source]["stamp"] = (
            rospy.Time.now()
        )

    # -----------------------------------------------------
    # Timer callback
    # -----------------------------------------------------

    def timer_callback(self, event):

        now = rospy.Time.now()

        selected_source = None

        for name, config in self.sources.items():

            command = self.commands[name]

            # Source chưa từng publish
            if command["msg"] is None:
                continue

            # ---------------------------------------------
            # Check timeout
            # ---------------------------------------------

            age = (
                now - command["stamp"]
            ).to_sec()

            if age > config["timeout"]:
                continue

            # ---------------------------------------------
            # Select source
            # ---------------------------------------------

            if selected_source is None:

                selected_source = name

                continue

            current_priority = config["priority"]

            selected_priority = (
                self.sources[selected_source]["priority"]
            )

            # Priority cao hơn
            if current_priority > selected_priority:

                selected_source = name

            # Cùng priority → command mới hơn thắng
            elif current_priority == selected_priority:

                if (
                    command["stamp"]
                    >
                    self.commands[selected_source]["stamp"]
                ):

                    selected_source = name

        # -------------------------------------------------
        # Publish result
        # -------------------------------------------------

        if selected_source is None:

            # Không có source nào active
            self.cmd_vel_pub.publish(
                Twist()
            )

            return

        selected_msg = (
            self.commands[selected_source]["msg"]
        )

        self.cmd_vel_pub.publish(
            selected_msg
        )

        rospy.logdebug(
            "[cmd_vel_mux] "
            "selected=%s priority=%d",
            selected_source,
            self.sources[selected_source]["priority"]
        )


def main():

    rospy.init_node(
        "cmd_vel_mux"
    )

    CmdVelMux()

    rospy.spin()


if __name__ == "__main__":
    main()