val libVersion: String by rootProject.extra
val dashVersion: String by rootProject.extra
val sdkVersion: String = "10.0.0"
val nodeVersion: String = "18.12.1"
val webDir: File = file("${project.projectDir.parent}/web")

plugins {
    id("com.android.library")
    kotlin("android")

    `maven-publish`

    id("com.github.node-gradle.node") version "2.2.4"
}

android {
    namespace = "com.acmerobotics.roadrunner.ftc"
    compileSdk = 35

    defaultConfig {
    minSdk = 24

      testInstrumentationRunner = "android.support.test.runner.AndroidJUnitRunner"
      consumerProguardFiles("consumer-rules.pro")
    }

    buildTypes {
       release {
           isMinifyEnabled = false
           proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
       }
    }
}

node {
    version = nodeVersion
    download = true
    nodeModulesDir = webDir
}

val yarnBuild by tasks.registering(Exec::class) {
    environment("VITE_APP_VERSION" to libVersion)
}

val yarnInstall by tasks.named("yarn_install") // Assuming yarn_install task exists

yarnBuild.configure {
    dependsOn(yarnInstall)
}

val cleanWebAssets by tasks.registering(Delete::class) {
    delete(android.sourceSets["main"]?.assets?.srcDirs?.firstOrNull()?.let { file("$it/web") } ?: project.buildDir.resolve("web_assets_deletion_fallback"))
    // Added a fallback in case assets sourceDirs is null or empty
}

tasks.named("clean").configure {
    dependsOn(cleanWebAssets)
}

val copyWebAssets by tasks.registering(Copy::class) {
    from(file("${project.projectDir.parent}/web/dist"))
    into(android.sourceSets["main"]?.assets?.srcDirs?.firstOrNull()?.let { file("$it/web") } ?: project.buildDir.resolve("web_assets_copy_fallback"))
    // Added a fallback in case assets sourceDirs is null or empty
    dependsOn(cleanWebAssets, yarnBuild)
}

android {
    libraryVariants.all {
        preBuildProvider.get().dependsOn(copyWebAssets)
    }
}

repositories {
    mavenCentral()
    maven("https://maven.brott.dev/")
}

dependencies {
    api(kotlin("stdlib-jdk8"))

    api(project(":core"))
    api(project(":actions"))

    api("com.acmerobotics.dashboard:core:$dashVersion")
    api("com.acmerobotics.dashboard:dashboard:$dashVersion")

    testImplementation(kotlin("test"))

    implementation("org.firstinspires.ftc:RobotCore:$sdkVersion")
    implementation("org.firstinspires.ftc:Hardware:$sdkVersion")

    implementation("com.fasterxml.jackson.core:jackson-databind:2.12.7")

    testImplementation(testFixtures(project(":core")))
    testImplementation(testFixtures(project(":actions")))
}

publishing {
    publications {
        create<MavenPublication>("maven") {
            groupId = "dev.nextftc.nextrunner"
            artifactId = "ftc"
            version = libVersion

            afterEvaluate {
                from(components["release"])
            }
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