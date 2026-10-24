import SwiftUI
#if os(macOS)
import AppKit
#endif

@main
struct Esp32RoverGcsApp: App {
    @StateObject private var model = AppModel()
    #if os(macOS)
    @NSApplicationDelegateAdaptor(AppDelegate.self) private var delegate
    #endif

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(model)
                #if os(macOS)
                .frame(minWidth: 900, minHeight: 640)
                #endif
        }
        #if os(macOS)
        .commands {
            CommandGroup(replacing: .newItem) {}
        }
        #endif
    }
}

#if os(macOS)
/// Closing the last window quits, and stops anything driving the car first.
final class AppDelegate: NSObject, NSApplicationDelegate {
    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }
    func applicationWillTerminate(_ notification: Notification) {
        AppModel.current?.stopDriving()
        AppModel.current?.vehicle.close()
    }
}
#endif
