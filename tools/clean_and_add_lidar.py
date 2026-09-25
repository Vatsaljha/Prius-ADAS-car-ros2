#!/usr/bin/env python3

from pathlib import Path
import re
import shutil


URDF = (
    Path.home()
    / "prius_ws/src/osrf_car_demo/prius_description/urdf/prius.urdf"
)

BACKUP = URDF.with_suffix(".urdf.before_clean_lidar_script")


def remove_named_element(text, tag, name):
    pattern = re.compile(
        rf'\s*<{tag}\s+name="{re.escape(name)}"[^>]*>'
        rf'.*?</{tag}>',
        re.DOTALL,
    )

    text, count = pattern.subn("", text)

    print(
        f"Removed {count} {tag}: {name}"
    )

    return text


def remove_gazebo_reference(text, reference):
    pattern = re.compile(
        rf'\s*<gazebo\s+reference="{re.escape(reference)}"[^>]*>'
        rf'.*?</gazebo>',
        re.DOTALL,
    )

    text, count = pattern.subn("", text)

    print(
        f"Removed {count} Gazebo block(s): {reference}"
    )

    return text


def main():

    if not URDF.exists():
        print("ERROR: URDF does not exist:")
        print(URDF)
        return

    print()
    print("==============================================")
    print("CLEANING ALL OLD LIDAR SENSORS")
    print("==============================================")
    print()

    if not BACKUP.exists():
        shutil.copy2(URDF, BACKUP)
        print("Backup:")
        print(BACKUP)
        print()

    text = URDF.read_text()

    # -------------------------------------------------------
    # Remove LEFT LiDAR
    # -------------------------------------------------------

    text = remove_named_element(
        text,
        "link",
        "front_left_laser_link",
    )

    text = remove_named_element(
        text,
        "joint",
        "front_left_laser_joint",
    )

    text = remove_gazebo_reference(
        text,
        "front_left_laser_link",
    )

    # -------------------------------------------------------
    # Remove RIGHT LiDAR
    # -------------------------------------------------------

    text = remove_named_element(
        text,
        "link",
        "front_right_laser_link",
    )

    text = remove_named_element(
        text,
        "joint",
        "front_right_laser_joint",
    )

    text = remove_gazebo_reference(
        text,
        "front_right_laser_link",
    )

    # -------------------------------------------------------
    # Remove EVERY existing center LiDAR
    #
    # This fixes the duplicate center_laser_link problem.
    # -------------------------------------------------------

    text = remove_named_element(
        text,
        "link",
        "center_laser_link",
    )

    text = remove_named_element(
        text,
        "joint",
        "center_laser_joint",
    )

    text = remove_gazebo_reference(
        text,
        "center_laser_link",
    )

    # -------------------------------------------------------
    # NEW CENTER LiDAR
    # -------------------------------------------------------

    lidar = r"""

  <!-- ================================================== -->
  <!-- CENTER FORWARD 2D LiDAR                           -->
  <!-- ================================================== -->

  <link name="center_laser_link">

    <inertial>

      <mass value="0.05"/>

      <origin
        xyz="0 0 0"
        rpy="0 0 0"/>

      <inertia
        ixx="0.00001"
        ixy="0"
        ixz="0"
        iyy="0.00001"
        iyz="0"
        izz="0.00001"/>

    </inertial>

    <visual>

      <origin
        xyz="0 0 0"
        rpy="0 0 0"/>

      <geometry>

        <cylinder
          radius="0.08"
          length="0.08"/>

      </geometry>

      <material name="CenterLidarBlack">

        <color
          rgba="0.05 0.05 0.05 1.0"/>

      </material>

    </visual>

    <collision>

      <origin
        xyz="0 0 0"
        rpy="0 0 0"/>

      <geometry>

        <cylinder
          radius="0.08"
          length="0.08"/>

      </geometry>

    </collision>

  </link>


  <!-- ================================================== -->
  <!-- CENTER LiDAR JOINT                                -->
  <!-- ================================================== -->

  <joint
    name="center_laser_joint"
    type="fixed">

    <parent link="chassis"/>

    <!--
      Centered on vehicle.
      Forward of chassis.
      Elevated above body.
    -->

    <origin
      xyz="1.0 0.0 0.70"
      rpy="0 0 -1.5707963"/>

    <child link="center_laser_link"/>

  </joint>


  <!-- ================================================== -->
  <!-- GAZEBO LiDAR SENSOR                              -->
  <!-- ================================================== -->

  <gazebo reference="center_laser_link">

    <sensor
      name="center_2d_lidar"
      type="ray">

      <always_on>true</always_on>

      <visualize>true</visualize>

      <update_rate>15.0</update_rate>


      <ray>

        <scan>

          <horizontal>

            <samples>360</samples>

            <resolution>1</resolution>

            <min_angle>-3.14159265359</min_angle>

            <max_angle>3.14159265359</max_angle>

          </horizontal>

        </scan>


        <range>

          <min>0.20</min>

          <max>30.0</max>

          <resolution>0.01</resolution>

        </range>

      </ray>


      <plugin
        name="center_lidar_plugin"
        filename="libgazebo_ros_ray_sensor.so">

        <ros>

          <namespace>/prius</namespace>

          <remapping>
            ~/out:=scan
          </remapping>

        </ros>

        <output_type>
          sensor_msgs/LaserScan
        </output_type>

        <frame_name>
          center_laser_link
        </frame_name>

      </plugin>

    </sensor>

  </gazebo>

"""

    # -------------------------------------------------------
    # Insert exactly once
    # -------------------------------------------------------

    end = text.rfind("</robot>")

    if end == -1:
        print("ERROR: </robot> was not found")
        return

    text = (
        text[:end]
        + lidar
        + text[end:]
    )

    URDF.write_text(text)

    print()
    print("==============================================")
    print("DONE")
    print("==============================================")
    print()
    print("Removed:")
    print("  front_left_laser")
    print("  front_right_laser")
    print("  previous center_laser")
    print()
    print("Added:")
    print("  ONE center_laser_link")
    print()
    print("Topic:")
    print("  /prius/scan")
    print()
    print("Frame:")
    print("  center_laser_link")
    print()


if __name__ == "__main__":
    main()
