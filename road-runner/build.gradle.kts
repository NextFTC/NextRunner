val libVersion by extra { "1.1.0-SNAP5" }
val dashVersion by extra { "0.4.10" }

plugins {
    id("com.android.application") version "8.0.0" apply false
    id("com.android.library") version "8.0.0" apply false
    kotlin("android") version "1.9.0" apply false
    kotlin("jvm") version "1.9.0" apply false
    kotlin("kapt") version "1.9.0" apply false
    kotlin("plugin.serialization") version "1.9.0" apply false
}

buildscript {
    repositories {
        google()
        mavenCentral()
    }
    dependencies {
        classpath("com.android.tools.build:gradle:8.0.0")
        classpath("org.jetbrains.kotlin:kotlin-gradle-plugin:1.9.0")
        // classpath("androidx.navigation:navigation-safe-args-gradle-plugin:2.7.0") // Example
    }
}

allprojects {
    repositories {
        google()
        mavenCentral()
    }
}