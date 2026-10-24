#!/bin/bash
# Builds the app for the Mac and for the iPhone simulator and writes the results to
# apple_gcs/build.log (errors and warnings first). Run from anywhere:
#   bash apple_gcs/tools/build.sh            build once
#   bash apple_gcs/tools/build.sh --watch    build again whenever a .swift file or tools/rebuild changes
cd "$(dirname "$0")/.." || exit 1

build_once() {
    {
        echo "=== build started $(date '+%H:%M:%S')"
        for dest in "platform=macOS" "generic/platform=iOS Simulator"; do
            echo "=== $dest"
            xcodebuild -project Esp32RoverGcs.xcodeproj -scheme Esp32RoverGcs -configuration Debug \
                -destination "$dest" -derivedDataPath build/DerivedData \
                CODE_SIGNING_ALLOWED=NO build > "build/last-$(echo "$dest" | tr -c 'a-zA-Z' '_').log" 2>&1
            rc=$?
            grep -E "error:|warning:|\*\* BUILD" "build/last-$(echo "$dest" | tr -c 'a-zA-Z' '_').log" | grep -v "^ld: warning" | sort -u
            echo "=== exit $rc"
        done
        echo "=== build finished $(date '+%H:%M:%S')"
    } > build/build.log.tmp 2>&1
    mv build/build.log.tmp build.log
}

mkdir -p build
if [ "$1" = "--watch" ]; then
    last=""
    echo "Watching for changes. Press Ctrl+C to stop."
    while true; do
        now=$( (find Esp32RoverGcs Esp32RoverGcs.xcodeproj -type f \( -name '*.swift' -o -name 'project.pbxproj' \) -exec stat -f '%m %N' {} + 2>/dev/null; cat tools/rebuild 2>/dev/null) | shasum)
        if [ "$now" != "$last" ]; then
            last=$now
            echo "$(date '+%H:%M:%S') building…"
            build_once
            tail -1 build.log
        fi
        sleep 3
    done
else
    build_once
    cat build.log
fi
