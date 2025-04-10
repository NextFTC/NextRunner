package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.profiles.ProfileParams
import com.acmerobotics.roadrunner.trajectories.TrajectoryBuilderParams

val TEST_PROFILE_PARAMS = ProfileParams(
    0.25,
    0.1,
    1e-4,
)

val TEST_TRAJECTORY_BUILDER_PARAMS = TrajectoryBuilderParams(
    1e-6,
    TEST_PROFILE_PARAMS
)
