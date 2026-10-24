// The app's Android-free core: MAVLink, the vehicle, links, NTRIP, fences,
// zig-zag, patrol and the LiDAR behaviours. Used by the Android app and the
// Windows (desktop) app alike.
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
