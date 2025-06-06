// Version properties are now defined in gradle.properties

plugins {
    id("com.android.application") version "8.0.2" apply false
    id("com.android.library") version "8.0.2" apply false
    id("org.jetbrains.dokka") version "2.0.0" apply false

    kotlin("android") version "2.0.0" apply false
    kotlin("jvm") version "2.0.0" apply false
    kotlin("kapt") version "2.0.0" apply false
    kotlin("plugin.serialization") version "2.0.0" apply false
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
