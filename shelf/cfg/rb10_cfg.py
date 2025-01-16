# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Rainbow Robotics.

The following configuration parameters are available:

* :obj:`RB10_CFG`: The RB10 arm without a gripper.

Reference: https://github.com/RainbowRobotics/rbpodo_ros2
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##


import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

"""Configuration of RB-10 arm using implicit actuator models."""

"""Ver. Joint Drive Parameter Cali Need(stiffness & damping)"""
RB_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/ryz/IsaacLab/shelf/env/rb.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={
            "base": 0.0,
            "shoulder": 0.0,
            "elbow": 0.0,
            "wrist1": 0.0,
            "wrist2": 0.0,
            "wrist3": 0.0,
        },
    ),
    actuators={
        "base": ImplicitActuatorCfg(
            joint_names_expr=["base"],
            effort_limit=330.0,
            velocity_limit=None,
            stiffness=8000,
            damping=500,
        ),
        "shoulder": ImplicitActuatorCfg(
            joint_names_expr=["shoulder"],
            effort_limit=2500.0,
            velocity_limit=None,
            stiffness=8000,
            damping=500,
        ),
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=["elbow"],
            effort_limit=150.0,
            velocity_limit=None,
            stiffness=8000,
            damping=500,
        ),                
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=["wrist.*"],
            effort_limit=10.0,
            velocity_limit=None,
            stiffness=8000,
            damping=500,
        ),
    },
)

"""Ver. Joint Drive Parameter Cali Fin(stiffness & damping)"""
RB10_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/ryz2/IsaacLab/shelf/env/Rb10.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "base": 0.0,
            "shoulder": 0.0,
            "elbow": 0.0,
            "wrist1": 0.0,
            "wrist2": 0.0,
            "wrist3": 0.0,
        },
    ),
    actuators={
        "base": ImplicitActuatorCfg(
            joint_names_expr=["base"],
            effort_limit=330.0,
            velocity_limit=None,
            stiffness=800,
            damping=40,
        ),
        "shoulder": ImplicitActuatorCfg(
            joint_names_expr=["shoulder"],
            effort_limit=2500.0,
            velocity_limit=None,
            stiffness=800,
            damping=40,
        ),
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=["elbow"],
            effort_limit=150.0,
            velocity_limit=None,
            stiffness=800,
            damping=40,
        ),                
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=["wrist.*"],
            effort_limit=10.0,
            velocity_limit=None,
            stiffness=800,
            damping=40,
        ),
    },
)
