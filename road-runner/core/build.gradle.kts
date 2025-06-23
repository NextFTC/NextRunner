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
}

dependencies {
    testImplementation(libs.kotlin.test)

    testFixturesApi(libs.bundles.kotest)

    testImplementation("org.knowm.xchart:xchart:3.8.8")

    dokkaHtmlPlugin("org.jetbrains.dokka:mathjax-plugin:1.9.10")
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
            artifactId = "core"
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

val dokkaJar = tasks.register<Jar>("dokkaJar") {
    dependsOn(tasks.named("dokkaGenerate"))
    from(dokka.basePublicationsDirectory.dir("html"))
    archiveClassifier.set("html-docs")
}

deployer {
    projectInfo {
        groupId.set("dev.nextftc.nextrunner")
        artifactId.set("core")

        name.set("NextRunner")
        description.set("A modern fork of RoadRunner.")
        url.set("https://github.com/NextFTC/NextRunner")
        scm {
            fromGithub("NextFTC", "NextRunner")
        }
        license("GNU General Public License, version 3", "https://www.gnu.org/licenses/gpl-3.0.html")

        developer("Zachary Harel", "ftc@zharel.me", url = "https://github.com/zachwaffle4")
        developer("Ryan Brott", "ftc@zharel.me", url = "https://github.com/zachwaffle4")
    }

    signing {
        key.set(secret("MVN_GPG_KEY"))
        password.set(secret("MVN_GPG_PASSWORD"))
    }

    content {
        kotlinComponents {
            kotlinSources()
            docs(dokkaJar)
        }
    }

    localSpec {
        release.version.set("$version")
    }

    nexusSpec("snapshot") {
        release.version.set("$version")
        repositoryUrl.set("https://central.sonatype.com/repository/maven-snapshots/")
        auth {
            user.set(secret("SONATYPE_USERNAME"))
            password.set(secret("SONATYPE_PASSWORD"))
        }
    }

    centralPortalSpec {
        auth {
            user.set(secret("SONATYPE_USERNAME"))
            password.set(secret("SONATYPE_PASSWORD"))
        }
        allowMavenCentralSync.set((property("automaticMavenCentralSync") as String).toBoolean())
    }
}