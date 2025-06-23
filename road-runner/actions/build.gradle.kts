import java.net.URI

val releasesDir: URI = File(project.property("zharelReleasesLocation").toString()).toURI()
val snapshotsDir: URI = File(project.property("zharelSnapshotsLocation").toString()).toURI()


plugins {
    kotlin("jvm")

    `java-library`
    `java-test-fixtures`

    id("org.jetbrains.dokka")

    `maven-publish`
    id("io.deepmedia.tools.deployer")
}

repositories {
    mavenCentral()
    maven { url = URI("https://maven.brott.dev/") }
}

dependencies {
    api(project(":core"))

    api(libs.dashboard.core)

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
    withJavadocJar()
}

tasks.named<Test>("test") {
    useJUnitPlatform()
}

publishing {
    publications {
        create<MavenPublication>("maven") {
            groupId = "dev.nextftc.nextrunner"
            artifactId = "actions"
            version = libs.versions.lib.get()

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
