"""Online Planning

Run online planning in collision aware environments.
"""

import time

import numpy as np
import pyroki as pk
import viser
from pyroki.collision import HalfSpace, RobotCollision, Sphere
from robot_descriptions.loaders.yourdfpy import load_robot_description
from viser.extras import ViserUrdf
import yourdfpy
from pathlib import Path
import os

import pyroki_snippets as pks


def main():
    """Main function for online planning with collision."""
    base_dir = Path(__file__).resolve().parent.parent
    os.chdir(base_dir / "robot_models")
    
    robot_name = "syncro_5.urdf"
    urdf_path = base_dir / "robot_models" / "urdf" 

    model = urdf_path / robot_name

    urdf = yourdfpy.URDF.load(model)
    target_link_name = "end-effector"
    robot = pk.Robot.from_urdf(urdf)

    robot_coll = RobotCollision.from_urdf(urdf)
    plane_coll = HalfSpace.from_point_and_normal(
        np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])
    )
    sphere_coll = Sphere.from_center_and_radius(
        np.array([0.0, 0.0, 0.0]), np.array([0.05])
    )

    # Define the online planning parameters.
    len_traj, dt = 3, 0.1

    # Set up visualizer.
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2, cell_size=0.1)
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/robot")

    # Create interactive controller for IK target.
    ik_target_handle = server.scene.add_transform_controls(
        "/ik_target", scale=0.2, position=(0.0, 0.36813, 0.5535), wxyz=(0, 0, 1, 0)
    )

    # Create interactive controller and mesh for the sphere obstacle.
    sphere_handle = server.scene.add_transform_controls(
        "/obstacle", scale=0.2, position=(0.4, 0.3, 0.4)
    )
    server.scene.add_mesh_trimesh("/obstacle/mesh", mesh=sphere_coll.to_trimesh())
    target_frame_handle = server.scene.add_batched_axes(
        "target_frame",
        axes_length=0.05,
        axes_radius=0.005,
        batched_positions=np.zeros((25, 3)),
        batched_wxyzs=np.array([[1.0, 0.0, 0.0, 0.0]] * 25),
    )

    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    sol_pos, sol_wxyz = None, None
    sol_traj = np.array(
        robot.joint_var_cls.default_factory()[None].repeat(len_traj, axis=0)
    )
    print(f"")
    while True:
        start_time = time.time()

        sphere_coll_world_current = sphere_coll.transform_from_wxyz_position(
            wxyz=np.array(sphere_handle.wxyz),
            position=np.array(sphere_handle.position),
        )

        world_coll_list = [plane_coll, sphere_coll_world_current]
        sol_traj, sol_pos, sol_wxyz = pks.solve_online_planning(
            robot=robot,
            robot_coll=robot_coll,
            world_coll=world_coll_list,
            target_link_name=target_link_name,
            target_position=np.array(ik_target_handle.position),
            target_wxyz=np.array(ik_target_handle.wxyz),
            timesteps=len_traj,
            dt=dt,
            start_cfg=sol_traj[0],
            prev_sols=sol_traj,
        )

        print(f"time taken {time.time() - start_time}")

        # Update timing handle.
        timing_handle.value = (
            0.99 * timing_handle.value + 0.01 * (time.time() - start_time) * 1000
        )

        # Update visualizer.
        urdf_vis.update_cfg(
            sol_traj[0]
        )  # The first step of the online trajectory solution.

        print(sol_pos)
        print(sol_wxyz)


        # Update the planned trajectory visualization.
        if hasattr(target_frame_handle, "batched_positions"):
            target_frame_handle.batched_positions = np.array(sol_pos)  # type: ignore[attr-defined]
            target_frame_handle.batched_wxyzs = np.array(sol_wxyz)  # type: ignore[attr-defined]
        else:
            # This is an older version of Viser.
            target_frame_handle.positions_batched = np.array(sol_pos)  # type: ignore[attr-defined]
            target_frame_handle.wxyzs_batched = np.array(sol_wxyz)  # type: ignore[attr-defined]
    

if __name__ == "__main__":
    main()
