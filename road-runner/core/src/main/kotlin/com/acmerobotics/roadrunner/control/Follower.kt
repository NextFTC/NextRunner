package com.acmerobotics.roadrunner.control

import com.acmerobotics.roadrunner.paths.AccelConstraint
import com.acmerobotics.roadrunner.paths.ProfileParams
import com.acmerobotics.roadrunner.paths.VelConstraint

data class FollowerParams(
    @JvmField
    val profileParams: ProfileParams,
    @JvmField
    val velConstraint: VelConstraint,
    @JvmField
    val accelConstraint: AccelConstraint
)