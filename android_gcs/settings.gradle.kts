pluginManagement {
    repositories { google(); mavenCentral(); gradlePluginPortal() }
}
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories { google(); mavenCentral() }
}
rootProject.name = "Esp32Gcs"
include(":core", ":desktop")
// GCS_DESKTOP_ONLY=1 builds the Windows app without the Android one (no Android SDK needed),
// as the Windows exe workflow does.
if (System.getenv("GCS_DESKTOP_ONLY") != "1") include(":app")
