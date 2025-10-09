"""Basic IK

Simplest Inverse Kinematics Example using PyRoki.
"""

import time

import numpy as np
import pyroki as pk
import viser
from robot_descriptions.loaders.yourdfpy import load_robot_description
from viser.extras import ViserUrdf

import pyroki_snippets as pks

from viser.extras import ViserUrdf
import yourdfpy

def main():
    """Main function for basic IK."""

    model_path = "/home/gonnayaswanth/pyroki/robot_models/urdf"
    robot_name = "/robot.urdf"

    model = model_path + robot_name

    urdf = yourdfpy.URDF.load(model)
    target_link_name = "tool"
    robot = pk.Robot.from_urdf(urdf)

    # Set up visualizer.
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")

    # Create interactive controller with initial position.
    ik_target = server.scene.add_transform_controls(
        "/ik_target", scale=0.2, position=(-0.0905339, 0.362746, 0.552664), wxyz=(0.000375478, 0.120817, -0.99267, -0.00301725)
    )
    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    for i in range(1):
        # Solve IK.
        start_time = time.time()
        solution = pks.solve_ik(
            robot=robot,
            target_link_name=target_link_name,
            target_position=np.array(ik_target.position),
            target_wxyz=np.array(ik_target.wxyz),
        )

        print(solution)

        # Update timing handle.
        elapsed_time = time.time() - start_time
        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)

        # Update visualizer.
        urdf_vis.update_cfg(solution)


if __name__ == "__main__":
    main()
