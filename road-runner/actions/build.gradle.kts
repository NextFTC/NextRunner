import java.net.URI

val libVersion = project.property("libVersion").toString()
val dashVersion = project.property("dashVersion").toString()

val releasesDir: URI = File(project.property("zharelReleasesLocation").toString()).toURI()
val snapshotsDir: URI = File(project.property("zharelSnapshotsLocation").toString()).toURI()


plugins {
    kotlin("jvm")

    `java-library`

    id("org.jetbrains.dokka")

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
            url = releasesDir
        }
        maven {
            name = "zharelSnapshots"
            url = snapshotsDir
        }
    }
}
