#!/usr/bin/env python3
"""Writes Esp32RoverGcs.xcodeproj: one SwiftUI target for iPhone, iPad and Mac.
Run from apple_gcs/: python3 tools/gen_xcodeproj.py (again after adding .swift files)."""
import hashlib, os

NAME = "Esp32RoverGcs"
BUNDLE = "com.quangvu16.Esp32RoverGcs"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, NAME)

def oid(*k):
    return hashlib.md5("/".join(k).encode()).hexdigest()[:24].upper()

files = []
for d, _, fs in os.walk(SRC):
    for f in sorted(fs):
        if f.endswith(".swift"):
            files.append(os.path.relpath(os.path.join(d, f), SRC))
files.sort()

# groups: top-level dir of the source folder
groups = {}
for f in files:
    g = os.path.dirname(f)
    groups.setdefault(g, []).append(f)

o = []
def add(s): o.append(s)

add("// !$*UTF8*$!\n{\n\tarchiveVersion = 1;\n\tclasses = {\n\t};\n\tobjectVersion = 56;\n\tobjects = {\n")
add("\n/* Begin PBXBuildFile section */\n")
for f in files:
    add(f"\t\t{oid('bf',f)} /* {os.path.basename(f)} in Sources */ = {{isa = PBXBuildFile; fileRef = {oid('fr',f)} /* {os.path.basename(f)} */; }};\n")
add("/* End PBXBuildFile section */\n\n/* Begin PBXFileReference section */\n")
add(f"\t\t{oid('product')} /* {NAME}.app */ = {{isa = PBXFileReference; explicitFileType = wrapper.application; includeInIndex = 0; path = {NAME}.app; sourceTree = BUILT_PRODUCTS_DIR; }};\n")
for f in files:
    add(f"\t\t{oid('fr',f)} /* {os.path.basename(f)} */ = {{isa = PBXFileReference; lastKnownFileType = sourcecode.swift; path = {os.path.basename(f)}; sourceTree = \"<group>\"; }};\n")
add("/* End PBXFileReference section */\n\n/* Begin PBXFrameworksBuildPhase section */\n")
add(f"\t\t{oid('fw')} /* Frameworks */ = {{isa = PBXFrameworksBuildPhase; buildActionMask = 2147483647; files = (\n\t\t\t); runOnlyForDeploymentPostprocessing = 0; }};\n")
add("/* End PBXFrameworksBuildPhase section */\n\n/* Begin PBXGroup section */\n")
add(f"\t\t{oid('maingroup')} = {{isa = PBXGroup; children = (\n\t\t\t\t{oid('srcgroup')} /* {NAME} */,\n\t\t\t\t{oid('products')} /* Products */,\n\t\t\t); sourceTree = \"<group>\"; }};\n")
add(f"\t\t{oid('products')} /* Products */ = {{isa = PBXGroup; children = (\n\t\t\t\t{oid('product')} /* {NAME}.app */,\n\t\t\t); name = Products; sourceTree = \"<group>\"; }};\n")
top = [oid('fr',f) + f" /* {f} */" for f in groups.get("", [])]
sub = [oid('grp',g) + f" /* {g} */" for g in sorted(groups) if g]
kids = "".join(f"\t\t\t\t{x},\n" for x in sub + top)
add(f"\t\t{oid('srcgroup')} /* {NAME} */ = {{isa = PBXGroup; children = (\n{kids}\t\t\t); path = {NAME}; sourceTree = \"<group>\"; }};\n")
for g in sorted(groups):
    if not g: continue
    kids = "".join(f"\t\t\t\t{oid('fr',f)} /* {os.path.basename(f)} */,\n" for f in groups[g])
    add(f"\t\t{oid('grp',g)} /* {g} */ = {{isa = PBXGroup; children = (\n{kids}\t\t\t); path = {g}; sourceTree = \"<group>\"; }};\n")
add("/* End PBXGroup section */\n\n/* Begin PBXNativeTarget section */\n")
add(f"""\t\t{oid('target')} /* {NAME} */ = {{
\t\t\tisa = PBXNativeTarget;
\t\t\tbuildConfigurationList = {oid('tcl')} /* Build configuration list for PBXNativeTarget "{NAME}" */;
\t\t\tbuildPhases = (
\t\t\t\t{oid('sources')} /* Sources */,
\t\t\t\t{oid('fw')} /* Frameworks */,
\t\t\t\t{oid('res')} /* Resources */,
\t\t\t);
\t\t\tbuildRules = (
\t\t\t);
\t\t\tdependencies = (
\t\t\t);
\t\t\tname = {NAME};
\t\t\tproductName = {NAME};
\t\t\tproductReference = {oid('product')} /* {NAME}.app */;
\t\t\tproductType = "com.apple.product-type.application";
\t\t}};
""")
add("/* End PBXNativeTarget section */\n\n/* Begin PBXProject section */\n")
add(f"""\t\t{oid('project')} /* Project object */ = {{
\t\t\tisa = PBXProject;
\t\t\tattributes = {{
\t\t\t\tBuildIndependentTargetsInParallel = 1;
\t\t\t\tLastSwiftUpdateCheck = 1500;
\t\t\t\tLastUpgradeCheck = 1500;
\t\t\t\tTargetAttributes = {{
\t\t\t\t\t{oid('target')} = {{
\t\t\t\t\t\tCreatedOnToolsVersion = 15.0;
\t\t\t\t\t}};
\t\t\t\t}};
\t\t\t}};
\t\t\tbuildConfigurationList = {oid('pcl')} /* Build configuration list for PBXProject "{NAME}" */;
\t\t\tcompatibilityVersion = "Xcode 14.0";
\t\t\tdevelopmentRegion = en;
\t\t\thasScannedForEncodings = 0;
\t\t\tknownRegions = (
\t\t\t\ten,
\t\t\t\tBase,
\t\t\t);
\t\t\tmainGroup = {oid('maingroup')};
\t\t\tproductRefGroup = {oid('products')} /* Products */;
\t\t\tprojectDirPath = "";
\t\t\tprojectRoot = "";
\t\t\ttargets = (
\t\t\t\t{oid('target')} /* {NAME} */,
\t\t\t);
\t\t}};
""")
add("/* End PBXProject section */\n\n/* Begin PBXResourcesBuildPhase section */\n")
add(f"\t\t{oid('res')} /* Resources */ = {{isa = PBXResourcesBuildPhase; buildActionMask = 2147483647; files = (\n\t\t\t); runOnlyForDeploymentPostprocessing = 0; }};\n")
add("/* End PBXResourcesBuildPhase section */\n\n/* Begin PBXSourcesBuildPhase section */\n")
srcs = "".join(f"\t\t\t\t{oid('bf',f)} /* {os.path.basename(f)} in Sources */,\n" for f in files)
add(f"\t\t{oid('sources')} /* Sources */ = {{isa = PBXSourcesBuildPhase; buildActionMask = 2147483647; files = (\n{srcs}\t\t\t); runOnlyForDeploymentPostprocessing = 0; }};\n")
add("/* End PBXSourcesBuildPhase section */\n\n/* Begin XCBuildConfiguration section */\n")

def settings(d):
    return "".join(f"\t\t\t\t{k} = {v};\n" for k, v in d.items())

proj_common = {
    "ALWAYS_SEARCH_USER_PATHS": "NO",
    "CLANG_ENABLE_MODULES": "YES",
    "CLANG_ENABLE_OBJC_ARC": "YES",
    "ENABLE_STRICT_OBJC_MSGSEND": "YES",
    "GCC_NO_COMMON_BLOCKS": "YES",
    "IPHONEOS_DEPLOYMENT_TARGET": "17.0",
    "MACOSX_DEPLOYMENT_TARGET": "14.0",
    "SWIFT_VERSION": "5.0",
}
# Debug is optimised too (-O): the indoor navigation maths is ~20x slower at -Onone
proj_debug = dict(proj_common, **{"DEBUG_INFORMATION_FORMAT": "dwarf", "ENABLE_TESTABILITY": "YES", "ONLY_ACTIVE_ARCH": "YES",
    "GCC_OPTIMIZATION_LEVEL": "0", "SWIFT_OPTIMIZATION_LEVEL": '"-O"', "SWIFT_ACTIVE_COMPILATION_CONDITIONS": '"DEBUG $(inherited)"',
    "GCC_PREPROCESSOR_DEFINITIONS": '(\n\t\t\t\t\t"DEBUG=1",\n\t\t\t\t\t"$(inherited)",\n\t\t\t\t)'})
proj_release = dict(proj_common, **{"DEBUG_INFORMATION_FORMAT": '"dwarf-with-dsym"', "SWIFT_COMPILATION_MODE": "wholemodule",
    "SWIFT_OPTIMIZATION_LEVEL": '"-O"'})
target = {
    "PRODUCT_NAME": '"$(TARGET_NAME)"',
    "PRODUCT_BUNDLE_IDENTIFIER": BUNDLE,
    "MARKETING_VERSION": "1.1",
    "CURRENT_PROJECT_VERSION": "2",
    "SDKROOT": "auto",
    "SUPPORTED_PLATFORMS": '"iphoneos iphonesimulator macosx"',
    "SUPPORTS_MACCATALYST": "NO",
    "SUPPORTS_MAC_DESIGNED_FOR_IPHONE_IPAD": "NO",
    "TARGETED_DEVICE_FAMILY": '"1,2"',
    "CODE_SIGN_STYLE": "Automatic",
    '"CODE_SIGN_IDENTITY[sdk=macosx*]"': '"-"',
    "DEVELOPMENT_TEAM": '""',
    "ENABLE_PREVIEWS": "YES",
    "ENABLE_APP_SANDBOX": "NO",
    "ENABLE_HARDENED_RUNTIME": "NO",
    "GENERATE_INFOPLIST_FILE": "YES",
    "INFOPLIST_KEY_CFBundleDisplayName": '"Rover GCS"',
    "INFOPLIST_KEY_LSApplicationCategoryType": '"public.app-category.utilities"',
    "INFOPLIST_KEY_NSLocalNetworkUsageDescription": '"The app talks MAVLink to the rover\'s ESP32 over its WiFi network (192.168.4.1)."',
    '"INFOPLIST_KEY_UIApplicationSceneManifest_Generation[sdk=iphone*]"': "YES",
    '"INFOPLIST_KEY_UILaunchScreen_Generation[sdk=iphone*]"': "YES",
    '"INFOPLIST_KEY_UIApplicationSupportsIndirectInputEvents[sdk=iphone*]"': "YES",
    '"INFOPLIST_KEY_UISupportedInterfaceOrientations_iPhone[sdk=iphone*]"': '"UIInterfaceOrientationPortrait UIInterfaceOrientationLandscapeLeft UIInterfaceOrientationLandscapeRight"',
    '"INFOPLIST_KEY_UISupportedInterfaceOrientations_iPad[sdk=iphone*]"': '"UIInterfaceOrientationPortrait UIInterfaceOrientationPortraitUpsideDown UIInterfaceOrientationLandscapeLeft UIInterfaceOrientationLandscapeRight"',
    "LD_RUNPATH_SEARCH_PATHS": '"$(inherited) @executable_path/Frameworks"',
    '"LD_RUNPATH_SEARCH_PATHS[sdk=macosx*]"': '"$(inherited) @executable_path/../Frameworks"',
    "SWIFT_EMIT_LOC_STRINGS": "YES",
    "SWIFT_VERSION": "5.0",
}
for cfg, base in (("Debug", proj_debug), ("Release", proj_release)):
    add(f"\t\t{oid('pc',cfg)} /* {cfg} */ = {{\n\t\t\tisa = XCBuildConfiguration;\n\t\t\tbuildSettings = {{\n{settings(base)}\t\t\t}};\n\t\t\tname = {cfg};\n\t\t}};\n")
    add(f"\t\t{oid('tc',cfg)} /* {cfg} */ = {{\n\t\t\tisa = XCBuildConfiguration;\n\t\t\tbuildSettings = {{\n{settings(target)}\t\t\t}};\n\t\t\tname = {cfg};\n\t\t}};\n")
add("/* End XCBuildConfiguration section */\n\n/* Begin XCConfigurationList section */\n")
add(f"\t\t{oid('pcl')} /* Build configuration list for PBXProject \"{NAME}\" */ = {{isa = XCConfigurationList; buildConfigurations = (\n\t\t\t\t{oid('pc','Debug')} /* Debug */,\n\t\t\t\t{oid('pc','Release')} /* Release */,\n\t\t\t); defaultConfigurationIsVisible = 0; defaultConfigurationName = Release; }};\n")
add(f"\t\t{oid('tcl')} /* Build configuration list for PBXNativeTarget \"{NAME}\" */ = {{isa = XCConfigurationList; buildConfigurations = (\n\t\t\t\t{oid('tc','Debug')} /* Debug */,\n\t\t\t\t{oid('tc','Release')} /* Release */,\n\t\t\t); defaultConfigurationIsVisible = 0; defaultConfigurationName = Release; }};\n")
add("/* End XCConfigurationList section */\n\t};\n\trootObject = " + oid('project') + " /* Project object */;\n}\n")

proj = os.path.join(ROOT, NAME + ".xcodeproj")
os.makedirs(os.path.join(proj, "xcshareddata", "xcschemes"), exist_ok=True)
open(os.path.join(proj, "project.pbxproj"), "w").write("".join(o))

ref = f"""<BuildableReference BuildableIdentifier = "primary" BlueprintIdentifier = "{oid('target')}" BuildableName = "{NAME}.app" BlueprintName = "{NAME}" ReferencedContainer = "container:{NAME}.xcodeproj"></BuildableReference>"""
scheme = f"""<?xml version="1.0" encoding="UTF-8"?>
<Scheme LastUpgradeVersion = "1500" version = "1.7">
   <BuildAction parallelizeBuildables = "YES" buildImplicitDependencies = "YES">
      <BuildActionEntries>
         <BuildActionEntry buildForTesting = "YES" buildForRunning = "YES" buildForProfiling = "YES" buildForArchiving = "YES" buildForAnalyzing = "YES">
            {ref}
         </BuildActionEntry>
      </BuildActionEntries>
   </BuildAction>
   <TestAction buildConfiguration = "Debug" selectedDebuggerIdentifier = "Xcode.DebuggerFoundation.Debugger.LLDB" selectedLauncherIdentifier = "Xcode.DebuggerFoundation.Launcher.LLDB" shouldUseLaunchSchemeArgsEnv = "YES">
   </TestAction>
   <LaunchAction buildConfiguration = "Debug" selectedDebuggerIdentifier = "Xcode.DebuggerFoundation.Debugger.LLDB" selectedLauncherIdentifier = "Xcode.DebuggerFoundation.Launcher.LLDB" launchStyle = "0" useCustomWorkingDirectory = "NO" ignoresPersistentStateOnLaunch = "NO" debugDocumentVersioning = "YES" debugServiceExtension = "internal" allowLocationSimulation = "YES">
      <BuildableProductRunnable runnableDebuggingMode = "0">
         {ref}
      </BuildableProductRunnable>
   </LaunchAction>
   <ProfileAction buildConfiguration = "Release" shouldUseLaunchSchemeArgsEnv = "YES" savedToolIdentifier = "" useCustomWorkingDirectory = "NO" debugDocumentVersioning = "YES">
      <BuildableProductRunnable runnableDebuggingMode = "0">
         {ref}
      </BuildableProductRunnable>
   </ProfileAction>
   <AnalyzeAction buildConfiguration = "Debug"></AnalyzeAction>
   <ArchiveAction buildConfiguration = "Release" revealArchiveInOrganizer = "YES"></ArchiveAction>
</Scheme>
"""
open(os.path.join(proj, "xcshareddata", "xcschemes", NAME + ".xcscheme"), "w").write(scheme)
print(f"{len(files)} Swift files -> {proj}")
