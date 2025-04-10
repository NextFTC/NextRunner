package com.acmerobotics.roadrunner.control

import com.acmerobotics.roadrunner.profiles.AccelConstraint
import com.acmerobotics.roadrunner.profiles.ProfileParams
import com.acmerobotics.roadrunner.profiles.VelConstraint

data class FollowerParams(
    @JvmField
    val profileParams: ProfileParams,
    @JvmField
    val velConstraint: VelConstraint,
    @JvmField
    val accelConstraint: AccelConstraint
)