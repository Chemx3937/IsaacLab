# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates how to create a rigid object and interact with it.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p shelf/src/shelf_env.py

"""

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on using the differential IK controller.")
parser.add_argument("--robot", type=str, default="rb10", help="Name of the robot.")
parser.add_argument("--num_envs", type=int, default=15, help="Number of environments to spawn.")
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
from omni.isaac.lab.assets import AssetBaseCfg
from omni.isaac.lab.assets import RigidObject, RigidObjectCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.markers import VisualizationMarkers
from omni.isaac.lab.markers.config import FRAME_MARKER_CFG
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg

# IK를 위한 모듈
from omni.isaac.lab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from omni.isaac.lab.utils.math import subtract_frame_transforms


# RB10 Articulation 만든 거 사용을 위한 모듈 설정
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from cfg.rb10_cfg import RB_CFG, RB_Gripper_CFG, UR_Gripper_CFG

@configclass
class ShelfPickingSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )


    # mount
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/Stand/stand_instanceable.usd", scale=(1.0, 1.0, 2.0)
        ),
    )

    # Robot Define (articulation) <- Use rb.usd
    if args_cli.robot == "rb10":
        # robot = RB_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        robot = RB_Gripper_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # robot = UR_Gripper_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    else:
        raise ValueError(f"Robot {args_cli.robot} is not supported. Valid: rb10")
    


    # Shelf Define
    # 사용하는 usd에 root_joint 등록되어 있으면 rb10이랑 붙어서 spawn되는 문제 발생
    shelf_cfg = RigidObjectCfg(
        init_state=RigidObjectCfg.InitialStateCfg(
            pos =(
                table.init_state.pos[0] + 0.0,
                table.init_state.pos[1] - 1.5,
                ground.init_state.pos[2] + 0.0,
            ),
            # pos=(0.0, -1.0, -1.05),
            # rot =(1.0, 0.0, 0.0, 180.0)
        ),
        prim_path="{ENV_REGEX_NS}/Shelf",
        spawn=sim_utils.UsdFileCfg(
            usd_path= "/home/ryz/IsaacLab/shelf/env/shelf2.usd",
            scale=(1.0, 1.0, 1.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                # rigid_body_enabled=True,
                disable_gravity=False,
            ),
            # articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            #     articulation_enabled=True
            # )
        )
    )

    bin = RigidObjectCfg(
        init_state=RigidObjectCfg.InitialStateCfg(
            pos =(
                table.init_state.pos[0] + 0.0,
                table.init_state.pos[1] - 0.7,
                ground.init_state.pos[2] + 0.0,
            ),
        ),
        prim_path="{ENV_REGEX_NS}/Bin",
        spawn=sim_utils.UsdFileCfg(
            usd_path= "/home/ryz/IsaacLab/shelf/env/bin.usd",
            scale=(9.0, 4.0, 5.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                disable_gravity=False,
            ),
        )
    )

    ######### Spawn Object ######### 

    # x: +-0.25, y: 
    # bottle = RigidObjectCfg(
    #     init_state=RigidObjectCfg.InitialStateCfg(
    #         pos =(
    #             shelf_cfg.init_state.pos[0] + 0.3,
    #             shelf_cfg.init_state.pos[1] - 0.15,
    #             shelf_cfg.init_state.pos[2] + 1.3,
    #         ),
    #     ),
    #     prim_path="{ENV_REGEX_NS}/Bottle",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path= "/home/ryz/IsaacLab/shelf/env/bottle.usd",
    #         scale=(1.0, 1.0, 1.0),
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #             rigid_body_enabled=True,
    #             disable_gravity=False,
    #         ),
    #     )
    # )

    # candle = RigidObjectCfg(
    #     init_state=RigidObjectCfg.InitialStateCfg(
    #         pos =(
    #             shelf_cfg.init_state.pos[0] + 0.2,
    #             shelf_cfg.init_state.pos[1] - 0.05,
    #             shelf_cfg.init_state.pos[2] + 1.3,
    #         ),
    #     ),
    #     prim_path="{ENV_REGEX_NS}/Candle",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path= "/home/ryz/IsaacLab/shelf/env/candle.usd",
    #         scale=(1.0, 1.0, 1.0),
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #             rigid_body_enabled=True,
    #             disable_gravity=False,
    #         ),
    #     )
    # )

    coke_can = RigidObjectCfg(
        init_state=RigidObjectCfg.InitialStateCfg(
            pos =(
                shelf_cfg.init_state.pos[0] + 0.0,
                shelf_cfg.init_state.pos[1] + 0.0,
                shelf_cfg.init_state.pos[2] + 1.3,
            ),
        ),
        prim_path="{ENV_REGEX_NS}/Coke",
        spawn=sim_utils.UsdFileCfg(
            usd_path= "/home/ryz/IsaacLab/shelf/env/coke_can.usd",
            scale=(1.0, 1.0, 1.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                disable_gravity=False,
            ),
        )
    )

    milk = RigidObjectCfg(
        init_state=RigidObjectCfg.InitialStateCfg(
            pos =(
                shelf_cfg.init_state.pos[0] - 0.15,
                shelf_cfg.init_state.pos[1] + 0.1,
                shelf_cfg.init_state.pos[2] + 1.3,
            ),
        ),
        prim_path="{ENV_REGEX_NS}/Milk",
        spawn=sim_utils.UsdFileCfg(
            usd_path= "/home/ryz/IsaacLab/shelf/env/milk.usd",
            scale=(1.0, 1.0, 1.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                disable_gravity=False,
            ),
        )
    )
    

    tomato_sauce = RigidObjectCfg(
        init_state=RigidObjectCfg.InitialStateCfg(
            pos =(
                shelf_cfg.init_state.pos[0] - 0.3,
                shelf_cfg.init_state.pos[1] - 0.15,
                shelf_cfg.init_state.pos[2] + 1.3,
            ),
        ),
        prim_path="{ENV_REGEX_NS}/Tomato_sauce",
        spawn=sim_utils.UsdFileCfg(
            usd_path= "/home/ryz/IsaacLab/shelf/env/tomato_sauce.usd",
            scale=(1.0, 1.0, 1.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                disable_gravity=False,
            ),
        )
    )

    cup = RigidObjectCfg(
        init_state=RigidObjectCfg.InitialStateCfg(
            pos =(
                shelf_cfg.init_state.pos[0] + 0.0,
                shelf_cfg.init_state.pos[1] + 0.2,
                shelf_cfg.init_state.pos[2] + 1.3,
            ),
        ),
        prim_path="{ENV_REGEX_NS}/Cup",
        spawn=sim_utils.UsdFileCfg(
            usd_path= "/home/ryz/IsaacLab/shelf/env/cup.usd",
            scale=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                disable_gravity=False,
            ),
        )
    )


    # red_book = RigidObjectCfg(
    #     init_state=RigidObjectCfg.InitialStateCfg(
    #         pos =(
    #             shelf_cfg.init_state.pos[0] - 0.25,
    #             shelf_cfg.init_state.pos[1] - 0.1,
    #             shelf_cfg.init_state.pos[2] + 1.3,
    #         ),
    #     ),
    #     prim_path="{ENV_REGEX_NS}/Red_book",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path= "/home/ryz/IsaacLab/shelf/env/red_book.usd",
    #         scale=(1.0, 1.0, 1.0),
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #             rigid_body_enabled=True,
    #             disable_gravity=False,
    #         ),
    #     )
    # )

    # green_book = RigidObjectCfg(
    #     init_state=RigidObjectCfg.InitialStateCfg(
    #         pos =(
    #             shelf_cfg.init_state.pos[0] - 0.3,
    #             shelf_cfg.init_state.pos[1] + 0.15,
    #             shelf_cfg.init_state.pos[2] + 1.3,
    #         ),
    #         # 쿼터니안 값으로 rot 입력해야 됨
    #         rot=(0.707, 0.0, 0.707, 0.0)    # y축으로 90도 회전
    #     ),
    #     prim_path="{ENV_REGEX_NS}/Green_book",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path= "/home/ryz/IsaacLab/shelf/env/green_book.usd",
    #         scale=(1.0, 1.0, 1.0),
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #             rigid_body_enabled=True,
    #             disable_gravity=False,
    #         ),
    #     )
    # )

    # blue_book = RigidObjectCfg(
    #     init_state=RigidObjectCfg.InitialStateCfg(
    #         pos =(
    #             shelf_cfg.init_state.pos[0] + 0.3,
    #             shelf_cfg.init_state.pos[1] + 0.1,
    #             shelf_cfg.init_state.pos[2] + 1.3,
    #         ),
    #         # 쿼터니안 값으로 rot 입력해야 됨
    #         rot=(0.5, 0.5, 0.5, -0.5)    # y축으로 90도, z축으로 90도 회전
    #     ),
    #     prim_path="{ENV_REGEX_NS}/Blue_book",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path= "/home/ryz/IsaacLab/shelf/env/blue_book.usd",
    #         scale=(1.0, 1.0, 1.0),
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #             rigid_body_enabled=True,
    #             disable_gravity=False,
    #         ),
    #     )
    # )
    ######### Spawn Object ######### 


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability.
    robot = scene["robot"]

    # Create controller
    diff_ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls")
    diff_ik_controller = DifferentialIKController(diff_ik_cfg, num_envs=scene.num_envs, device=sim.device)

    # Markers
    frame_marker_cfg = FRAME_MARKER_CFG.copy()
    frame_marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    tcp_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/tcp_current"))
    goal_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/tcp_goal"))

    # Define goals for the arm
    tcp_goals = [
        [0.0, -0.5, 0.5, 1.0, 0.0, 0.0, 0.0],
        [0.0, -0.9, 0.425, 1.0, 0.0, 0.0, 0.0],
        [0.5, 0.5, 0.7, 0.707, 0, 0.707, 0],
        [0.5, -0.4, 0.6, 0.707, 0.707, 0.0, 0.0],
        [0.5, 0, 0.5, 0.0, 1.0, 0.0, 0.0],
    ]
    tcp_goals = torch.tensor(tcp_goals, device=sim.device)
    # Track the given command
    current_goal_idx = 0
    # Create buffers to store actions
    ik_commands = torch.zeros(scene.num_envs, diff_ik_controller.action_dim, device=robot.device)
    ik_commands[:] = tcp_goals[current_goal_idx]

    # Specify robot-specific parameters
    if args_cli.robot == "rb10":
        robot_entity_cfg = SceneEntityCfg("robot", joint_names=[".*"], body_names=["tcp"])
    else:
        raise ValueError(f"Robot {args_cli.robot} is not supported. Valid: rb10")
    
    # Resolving the scene entities
    robot_entity_cfg.resolve(scene)
    # Obtain the frame index of the end-effector
    # For a fixed base robot, the frame index is one less than the body index. This is because
    # the root body is not included in the returned Jacobians.
    if robot.is_fixed_base:
        tcp_jacobi_idx = robot_entity_cfg.body_ids[0] - 1
    else:
        tcp_jacobi_idx = robot_entity_cfg.body_ids[0]


    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    # Simulation loop
    while simulation_app.is_running():
        # reset
        if count % 150 == 0:
            # reset time
            count = 0
            # reset joint state
            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()
            # reset actions
            ik_commands[:] = tcp_goals[current_goal_idx]
            joint_pos_des = joint_pos[:, robot_entity_cfg.joint_ids].clone()
            # reset controller
            diff_ik_controller.reset()
            diff_ik_controller.set_command(ik_commands)
            # change goal
            current_goal_idx = (current_goal_idx + 1) % len(tcp_goals)
        else:
            # obtain quantities from simulation
            jacobian = robot.root_physx_view.get_jacobians()[:, tcp_jacobi_idx, :, robot_entity_cfg.joint_ids]
            tcp_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
            root_pose_w = robot.data.root_state_w[:, 0:7]
            joint_pos = robot.data.joint_pos[:, robot_entity_cfg.joint_ids]
            # compute frame in root frame
            tcp_pos_b, tcp_quat_b = subtract_frame_transforms(
                root_pose_w[:, 0:3], root_pose_w[:, 3:7], tcp_pose_w[:, 0:3], tcp_pose_w[:, 3:7]
            )
            # compute the joint commands
            joint_pos_des = diff_ik_controller.compute(tcp_pos_b, tcp_quat_b, jacobian, joint_pos)

        # apply actions
        robot.set_joint_position_target(joint_pos_des, joint_ids=robot_entity_cfg.joint_ids)
        scene.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        count += 1
        # update buffers
        scene.update(sim_dt)

        # obtain quantities from simulation
        tcp_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
        # update marker positions
        tcp_marker.visualize(tcp_pose_w[:, 0:3], tcp_pose_w[:, 3:7])
        goal_marker.visualize(ik_commands[:, 0:3] + scene.env_origins, ik_commands[:, 3:7])


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])
    # Design scene
    scene_cfg = ShelfPickingSceneCfg(num_envs=args_cli.num_envs, env_spacing=5.0)
    scene = InteractiveScene(scene_cfg)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()