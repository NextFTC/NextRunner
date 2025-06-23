import com.moowork.gradle.node.yarn.YarnTask
import java.net.URI
import kotlin.collections.mapOf

val nodeVersion: String = "18.12.1"


val webDir: File = file("${project.projectDir.parent}/web")

val releasesDir: URI = File(project.property("zharelReleasesLocation").toString()).toURI()
val snapshotsDir: URI = File(project.property("zharelSnapshotsLocation").toString()).toURI()


plugins {
    id("com.android.library")

    kotlin("android")
    kotlin("plugin.serialization")

    id("org.jetbrains.dokka")

    `maven-publish`
    id("io.deepmedia.tools.deployer")

    id("com.github.node-gradle.node") version "2.2.4"
}

android {
    namespace = "com.acmerobotics.roadrunner.ftc"
    //noinspection GradleDependency
    compileSdk = 33

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

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    kotlinOptions {
        jvmTarget = "1.8"
        freeCompilerArgs += ("-Xjvm-default=all")
    }

    testOptions {
        unitTests {
            isReturnDefaultValues = true
        }
    }
}

node {
    version = nodeVersion
    download = true
    nodeModulesDir = webDir
}

val yarnInstall = tasks.named("yarn_install")

tasks.named<YarnTask>("yarn_build") {
    setEnvironment(
        mapOf("VITE_APP_VERSION" to libs.versions.lib.get())
    )

    dependsOn(yarnInstall)
}

val yarnBuild = tasks.named("yarn_build")

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

    publishing {
        singleVariant("release") {
            withSourcesJar()
            withJavadocJar()
        }
    }
}

repositories {
    mavenCentral()
    maven("https://maven.brott.dev/")
}

dependencies {
    api(project(":core"))
    api(project(":actions"))

    implementation(libs.bundles.dashboard)

    implementation(libs.bundles.ftcsdk)

    implementation("com.fasterxml.jackson.core:jackson-databind:2.12.7")
    implementation("org.jetbrains.kotlinx:kotlinx-serialization-json:1.8.0")

    testImplementation(kotlin("test"))
    testImplementation(libs.mockk)
    testImplementation(libs.bundles.kotest)

    testImplementation(testFixtures(project(":core")))
}

publishing {
    publications {
        create<MavenPublication>("maven") {
            groupId = "dev.nextftc.nextrunner"
            artifactId = "ftc"
            version = libs.versions.lib.get()

            afterEvaluate {
                from(components["release"])
            }
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
