import java.net.URI

val libVersion: String by rootProject.extra
val dashVersion: String by rootProject.extra

plugins {
    kotlin("jvm")

    `java-library`

    id("org.jetbrains.dokka")

    id("org.jlleitschuh.gradle.ktlint") version "10.2.1"
    id("org.jlleitschuh.gradle.ktlint-idea") version "10.2.1"

    `maven-publish`
}

repositories {
    mavenCentral()
    maven { url = URI("https://maven.brott.dev/") }
}

dependencies {
    api(project(":core"))

    api("com.acmerobotics.dashboard:core:$dashVersion")

    testImplementation("org.jetbrains.kotlin:kotlin-test")

    testImplementation(testFixtures(project(":core")))
}

kotlin {
    compilerOptions {
        freeCompilerArgs.set(listOf("-Xjvm-default=all"))
    }
}

java {
    withSourcesJar()
}

tasks.named<Test>("test") {
    useJUnitPlatform()
}

publishing {
    publications {
        create<MavenPublication>("maven") {
            groupId = "dev.nextftc.nextrunner"
            artifactId = "actions"
            version = libVersion

            from(components["java"])
        }
    }

    repositories {
        maven {
            name = "zharelReleases"
            url = File(project.property("zharelReleasesLocation")!!.toString()).toURI()
        }
        maven {
            name = "zharelSnapshots"
            url = File(project.property("zharelSnapshotsLocation")!!.toString()).toURI()
        }
    }
}