val libVersion = project.property("libVersion").toString()
val kotestVersion = project.property("kotestVersion").toString()

plugins {
    kotlin("jvm")

    `java-library`
    `java-test-fixtures`

    id("org.jetbrains.dokka")

    id("org.jlleitschuh.gradle.ktlint") version "10.2.1"
    id("org.jlleitschuh.gradle.ktlint-idea") version "10.2.1"

    `maven-publish`
}

repositories {
    mavenCentral()
}

dependencies {
    testImplementation("org.jetbrains.kotlin:kotlin-test")
    testImplementation("io.kotest:kotest-runner-junit5:$kotestVersion")

    testFixturesApi("io.kotest:kotest-assertions-core:$kotestVersion")
    testFixturesApi("io.kotest:kotest-property:$kotestVersion")

    testImplementation("org.knowm.xchart:xchart:3.8.8")

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
