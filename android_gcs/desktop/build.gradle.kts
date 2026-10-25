// ESP32 Rover GCS for Windows (any desktop with Java 17): the same core as the
// Android app, with a Swing window. `gradle :desktop:fatJar` makes one runnable jar.
plugins {
    id("org.jetbrains.kotlin.jvm")
}

java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

tasks.withType<org.jetbrains.kotlin.gradle.tasks.KotlinCompile>().configureEach {
    kotlinOptions.jvmTarget = "17"
}

dependencies {
    implementation(project(":core"))
}

val mainClassName = "org.ardupilot.esp32gcs.desktop.MainKt"

tasks.register<Jar>("fatJar") {
    group = "build"
    description = "One runnable jar with Kotlin and the core included"
    archiveBaseName.set("Esp32RoverGcs")
    archiveVersion.set("1.4")
    manifest { attributes["Main-Class"] = mainClassName }
    duplicatesStrategy = DuplicatesStrategy.EXCLUDE
    from(sourceSets.main.get().output)
    dependsOn(configurations.runtimeClasspath)
    from({ configurations.runtimeClasspath.get().filter { it.name.endsWith(".jar") }.map { zipTree(it) } })
    exclude("META-INF/*.SF", "META-INF/*.DSA", "META-INF/*.RSA", "META-INF/versions/**")
}
