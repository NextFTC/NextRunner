val libVersion: String by rootProject.extra
val rrUVersion = "1.5.0-RC-1"

plugins {
    id("org.jetbrains.kotlin.jvm") version "1.9.0"

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
            groupId = "com.acmerobotics.roadrunner"
            artifactId = "core_unofficial"
            version = rrUVersion

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
