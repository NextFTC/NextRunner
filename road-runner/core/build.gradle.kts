val libVersion: String by rootProject.extra

plugins {
    kotlin("jvm")

    `java-library`
    `java-test-fixtures`

    id("org.jetbrains.dokka") version "1.9.10"

    id("org.jlleitschuh.gradle.ktlint") version "10.2.1"
    id("org.jlleitschuh.gradle.ktlint-idea") version "10.2.1"

    `maven-publish`
}

repositories {
    mavenCentral()
}

dependencies {
    api(kotlin("stdlib-jdk8"))

    testImplementation("org.jetbrains.kotlin:kotlin-test")

    testImplementation("org.knowm.xchart:xchart:3.8.1")

    dokkaHtmlPlugin("org.jetbrains.dokka:mathjax-plugin:1.9.10")
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
            artifactId = "core"
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
