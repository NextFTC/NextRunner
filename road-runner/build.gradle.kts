import org.jetbrains.dokka.gradle.DokkaTaskPartial

plugins {
    id("com.android.application") version "8.0.2" apply false
    id("com.android.library") version "8.0.2" apply false
    id("org.jetbrains.dokka") version "2.0.0"
    id("io.deepmedia.tools.deployer") version "0.18.0"

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

dependencies {
    dokka(project(":core"))
    dokka(project(":actions"))
    dokka(project(":ftc"))
}

val dokkaJar = tasks.register<Jar>("dokkaJar") {
    dependsOn(tasks.named("dokkaGenerate"))
    from(dokka.basePublicationsDirectory.dir("html"))
    archiveClassifier.set("html-docs")
}

dokka {
    moduleName.set("NextRunner")

    dokkaSourceSets.configureEach {
        sourceLink {
            localDirectory.set(projectDir.resolve("src/main/kotlin"))
            remoteUrl("https://github.com/NextFTC/NextRunner/blob/main/src/main/kotlin")
            remoteLineSuffix.set("#L")
        }

        externalDocumentationLinks.register("dashboard-docs") {
            url("https://acmerobotics.github.io/ftc-dashboard/javadoc/") // ENSURE THERE IS A TRAILING SLASH
            packageListUrl.set(rootDir.resolve("docs/dashboard.package.list").toURI())
        }
    }
}

deployer {
    projectInfo {
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
        androidComponents("release") {
            kotlinSources()
            docs(dokkaJar)
        }
    }

    localSpec {
        release.version.set("$version")
    }

    nexusSpec("snapshot") {
        release.version.set("$version-SNAPSHOT")
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