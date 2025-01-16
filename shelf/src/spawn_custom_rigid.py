# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates how to create a rigid object and interact with it.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p source/standalone/tutorials/01_assets/run_rigid_object.py

"""

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on spawning and interacting with a rigid object.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.lab.sim as sim_utils
import omni.isaac.lab.utils.math as math_utils
from omni.isaac.lab.assets import RigidObject, RigidObjectCfg
from omni.isaac.lab.sim import SimulationContext


def design_scene():
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
    cfg.func("/World/Light", cfg)

    # Create a single group called "Origin0"
    origin = [10.0, 0.0, 0.0]  # Single origin at the center
    prim_utils.create_prim("/World/Shelf", "Xform", translation=origin)

    # Rigid Object
    shelf_cfg = RigidObjectCfg(
        init_state=RigidObjectCfg.InitialStateCfg(pos =(10.0, 0.0, 0.0), rot =(1.0, 0.0, 0.0, 0.0)),
        prim_path="/World/Shelf/Shelf",
        spawn=sim_utils.UsdFileCfg(
            usd_path= "/home/ryz/IsaacLab/shelf/env/shelf_origin.usd",
            scale=(0.3, 0.3, 0.3),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                articulation_enabled=False
            )
        )
    )
    shlef_object = RigidObject(cfg=shelf_cfg)

    # return the scene information
    scene_entities = {"shelf": shlef_object}
    return scene_entities, origin


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, RigidObject], origins: torch.Tensor):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability. In general, it is better to access the entities directly from
    #   the dictionary. This dictionary is replaced by the InteractiveScene class in the next tutorial.
    # cone_object = entities["cone"]
    shelf_object = entities["shelf"]

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    # Simulate physics
    while simulation_app.is_running():
        # # reset
        # if count % 250 == 0:
        #     # reset counters
        #     sim_time = 0.0
        #     count = 0
        #     # reset root state
        #     root_state = cone_object.data.default_root_state.clone()
        #     # sample a random position on a cylinder around the origins
        #     root_state[:, :3] += origins
        #     root_state[:, :3] += math_utils.sample_cylinder(
        #         radius=0.1, h_range=(0.25, 0.5), size=cone_object.num_instances, device=cone_object.device
        #     )
        #     # write root state to simulation
        #     cone_object.write_root_link_pose_to_sim(root_state[:, :7])
        #     cone_object.write_root_com_velocity_to_sim(root_state[:, 7:])
        #     # reset buffers
        #     cone_object.reset()
        #     print("----------------------------------------")
        #     print("[INFO]: Resetting object state...")
        # apply sim data
        # shelf_object.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        # shelf_object.update(sim_dt)
        # print the root position
        # if count % 50 == 0:
        #     print(f"Root position (in world): {shelf_object.data.root_link_state_w[:, :3]}")


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view(eye=[1.5, 0.0, 1.0], target=[0.0, 0.0, 0.0])
    # Design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
