#!/usr/bin/env python3

import os
import subprocess

import rospy
import rospkg

from std_msgs.msg import String

from localize_manager.srv import (
    StartMapping,
    StartMappingResponse,
    SaveMap,
    SaveMapResponse,
    StartLocalization,
    StartLocalizationResponse,
    Stop,
    StopResponse,
)


class LocalizeManager:

    IDLE = "IDLE"
    MAPPING = "MAPPING"
    SAVING_MAP = "SAVING_MAP"
    LOCALIZING = "LOCALIZING"
    ERROR = "ERROR"

    def __init__(self):

        # -------------------------------------------------
        # Package paths
        # -------------------------------------------------

        rospack = rospkg.RosPack()

        self.package_path = rospack.get_path(
            "localize_manager"
        )

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------

        map_directory = rospy.get_param(
            "~map_directory",
            "maps"
        )

        if os.path.isabs(map_directory):

            self.map_directory = map_directory

        else:

            self.map_directory = os.path.join(
                self.package_path,
                map_directory
            )

        self.state_publish_rate = rospy.get_param(
            "~state_publish_rate",
            2.0
        )

        # -------------------------------------------------
        # Launch files
        # -------------------------------------------------

        self.mapping_launch = os.path.join(
            self.package_path,
            "launch",
            "mapping.launch"
        )

        self.localization_launch = os.path.join(
            self.package_path,
            "launch",
            "localization.launch"
        )

        # -------------------------------------------------
        # Runtime state
        # -------------------------------------------------

        self.state = self.IDLE

        self.current_map = ""

        self.mapping_process = None
        self.localization_process = None

        # -------------------------------------------------
        # State publisher
        # -------------------------------------------------

        self.state_pub = rospy.Publisher(
            "/localize_manager/state",
            String,
            queue_size=1,
            latch=True
        )

        # -------------------------------------------------
        # Services
        # -------------------------------------------------

        self.start_mapping_srv = rospy.Service(
            "/localize_manager/start_mapping",
            StartMapping,
            self.handle_start_mapping
        )

        self.save_map_srv = rospy.Service(
            "/localize_manager/save_map",
            SaveMap,
            self.handle_save_map
        )

        self.start_localization_srv = rospy.Service(
            "/localize_manager/start_localization",
            StartLocalization,
            self.handle_start_localization
        )

        self.stop_srv = rospy.Service(
            "/localize_manager/stop",
            Stop,
            self.handle_stop
        )

        # -------------------------------------------------
        # Timer
        # -------------------------------------------------

        self.timer = rospy.Timer(
            rospy.Duration(
                1.0 / self.state_publish_rate
            ),
            self.timer_callback
        )

        self.publish_state()

        rospy.loginfo(
            "[localize_manager] Started"
        )

        rospy.loginfo(
            "[localize_manager] Map directory: %s",
            self.map_directory
        )

    # =====================================================
    # State
    # =====================================================

    def set_state(self, state):

        self.state = state

        self.publish_state()

        rospy.loginfo(
            "[localize_manager] STATE -> %s",
            state
        )

    def publish_state(self):

        self.state_pub.publish(
            String(data=self.state)
        )

    def timer_callback(self, event):

        self.check_processes()

    # =====================================================
    # Process management
    # =====================================================

    def check_processes(self):

        # Mapping process unexpectedly died
        if self.mapping_process is not None:

            return_code = self.mapping_process.poll()

            if return_code is not None:

                rospy.logerr(
                    "[localize_manager] "
                    "gmapping process exited: %d",
                    return_code
                )

                self.mapping_process = None

                if self.state == self.MAPPING:
                    self.set_state(self.ERROR)

        # Localization process unexpectedly died
        if self.localization_process is not None:

            return_code = self.localization_process.poll()

            if return_code is not None:

                rospy.logerr(
                    "[localize_manager] "
                    "localization process exited: %d",
                    return_code
                )

                self.localization_process = None

                if self.state == self.LOCALIZING:
                    self.set_state(self.ERROR)

    # =====================================================
    # Start mapping
    # =====================================================

    def handle_start_mapping(self, request):

        if self.state == self.MAPPING:

            return StartMappingResponse(
                False,
                "Already mapping"
            )

        if self.state == self.LOCALIZING:

            self.stop_localization()

        elif self.state == self.ERROR:

            self.stop_all_processes()

        try:

            rospy.loginfo(
                "[localize_manager] Starting mapping"
            )

            self.mapping_process = subprocess.Popen([
                "roslaunch",
                "localize_manager",
                "mapping.launch"
            ])

            rospy.sleep(1.0)

            if self.mapping_process.poll() is not None:

                self.mapping_process = None

                self.set_state(self.ERROR)

                return StartMappingResponse(
                    False,
                    "Failed to start gmapping"
                )

            self.current_map = ""

            self.set_state(self.MAPPING)

            return StartMappingResponse(
                True,
                "Mapping started"
            )

        except Exception as e:

            rospy.logerr(
                "[localize_manager] "
                "Failed to start mapping: %s",
                str(e)
            )

            self.set_state(self.ERROR)

            return StartMappingResponse(
                False,
                str(e)
            )

    # =====================================================
    # Save map
    # =====================================================

    def handle_save_map(self, request):

        # -------------------------------------------------
        # Must be mapping
        # -------------------------------------------------

        if self.state != self.MAPPING:

            return SaveMapResponse(
                False,
                "Robot is not in MAPPING state"
            )

        # -------------------------------------------------
        # Validate map name
        # -------------------------------------------------

        map_name = request.map_name.strip()

        if not map_name:

            return SaveMapResponse(
                False,
                "map_name is empty"
            )

        # Prevent path traversal
        if "/" in map_name or "\\" in map_name:

            return SaveMapResponse(
                False,
                "Invalid map_name"
            )

        # -------------------------------------------------
        # Prepare path
        # -------------------------------------------------

        map_path = os.path.join(
            self.map_directory,
            map_name,
            map_name
        )

        map_folder = os.path.dirname(
            map_path
        )

        try:

            os.makedirs(
                map_folder,
                exist_ok=True
            )

            rospy.loginfo(
                "[localize_manager] "
                "Saving map to %s",
                map_path
            )

            # -------------------------------------------------
            # Save current map
            # -------------------------------------------------

            result = subprocess.run(
                [
                    "rosrun",
                    "map_server",
                    "map_saver",
                    "-f",
                    map_path
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            if result.returncode != 0:

                rospy.logerr(
                    "[localize_manager] "
                    "map_saver failed:\n%s",
                    result.stderr
                )

                return SaveMapResponse(
                    False,
                    "map_saver failed"
                )

            # -------------------------------------------------
            # Verify output
            # -------------------------------------------------

            yaml_file = map_path + ".yaml"
            pgm_file = map_path + ".pgm"

            if (
                not os.path.exists(yaml_file)
                or
                not os.path.exists(pgm_file)
            ):

                return SaveMapResponse(
                    False,
                    "Map files were not created"
                )

            rospy.loginfo(
                "[localize_manager] "
                "Map saved successfully: %s",
                map_name
            )

            # -------------------------------------------------
            # IMPORTANT:
            # Do NOT stop gmapping.
            # Do NOT change state.
            # -------------------------------------------------

            return SaveMapResponse(
                True,
                "Map saved successfully: {}".format(
                    map_name
                )
            )

        except Exception as e:

            rospy.logerr(
                "[localize_manager] "
                "Save map exception: %s",
                str(e)
            )

            return SaveMapResponse(
                False,
                str(e)
            )

    # =====================================================
    # Start localization
    # =====================================================

    def handle_start_localization(self, request):

        map_name = request.map_name.strip()

        if not map_name:

            return StartLocalizationResponse(
                False,
                "map_name is empty"
            )

        if "/" in map_name or "\\" in map_name:

            return StartLocalizationResponse(
                False,
                "Invalid map_name"
            )

        map_yaml = os.path.join(
            self.map_directory,
            map_name,
            map_name + ".yaml"
        )

        if not os.path.exists(map_yaml):

            return StartLocalizationResponse(
                False,
                "Map does not exist: {}".format(
                    map_yaml
                )
            )

        try:

            # Stop mapping if necessary
            if self.state == self.MAPPING:

                self.stop_mapping()

            elif self.state == self.LOCALIZING:

                self.stop_localization()

            elif self.state == self.ERROR:

                self.stop_all_processes()

            rospy.loginfo(
                "[localize_manager] "
                "Starting localization with map: %s",
                map_name
            )

            self.localization_process = subprocess.Popen([
                "roslaunch",
                "localize_manager",
                "localization.launch",
                "map_file:={}".format(map_yaml)
            ])

            rospy.sleep(1.0)

            if self.localization_process.poll() is not None:

                self.localization_process = None

                self.set_state(self.ERROR)

                return StartLocalizationResponse(
                    False,
                    "Failed to start localization"
                )

            self.current_map = map_name

            self.set_state(self.LOCALIZING)

            return StartLocalizationResponse(
                True,
                "Localization started"
            )

        except Exception as e:

            rospy.logerr(
                "[localize_manager] "
                "Failed to start localization: %s",
                str(e)
            )

            self.set_state(self.ERROR)

            return StartLocalizationResponse(
                False,
                str(e)
            )

    # =====================================================
    # Stop
    # =====================================================

    def handle_stop(self, request):

        self.stop_all_processes()

        self.current_map = ""

        self.set_state(self.IDLE)

        return StopResponse(
            True,
            "Localization manager stopped"
        )

    # =====================================================
    # Stop mapping
    # =====================================================

    def stop_mapping(self):

        if self.mapping_process is None:
            return

        rospy.loginfo(
            "[localize_manager] Stopping gmapping"
        )

        self.mapping_process.terminate()

        try:

            self.mapping_process.wait(
                timeout=5.0
            )

        except subprocess.TimeoutExpired:

            rospy.logwarn(
                "[localize_manager] "
                "gmapping did not terminate, killing"
            )

            self.mapping_process.kill()

            self.mapping_process.wait()

        self.mapping_process = None

    # =====================================================
    # Stop localization
    # =====================================================

    def stop_localization(self):

        if self.localization_process is None:
            return

        rospy.loginfo(
            "[localize_manager] "
            "Stopping localization"
        )

        self.localization_process.terminate()

        try:

            self.localization_process.wait(
                timeout=5.0
            )

        except subprocess.TimeoutExpired:

            rospy.logwarn(
                "[localize_manager] "
                "localization did not terminate, killing"
            )

            self.localization_process.kill()

            self.localization_process.wait()

        self.localization_process = None

    # =====================================================
    # Stop everything
    # =====================================================

    def stop_all_processes(self):

        self.stop_mapping()

        self.stop_localization()

    # =====================================================
    # Shutdown
    # =====================================================

    def shutdown(self):

        rospy.loginfo(
            "[localize_manager] Shutting down"
        )

        self.stop_all_processes()


def main():

    rospy.init_node(
        "localize_manager"
    )

    manager = LocalizeManager()

    rospy.on_shutdown(
        manager.shutdown
    )

    rospy.spin()


if __name__ == "__main__":
    main()