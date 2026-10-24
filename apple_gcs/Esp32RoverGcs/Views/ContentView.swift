import SwiftUI
#if os(macOS)
import AppKit
#endif

/// Every message from the autopilot (STATUSTEXT), newest first.
struct LogView: View {
    @EnvironmentObject var m: AppModel
    var body: some View {
        _ = m.tick
        return List(m.vehicle.messages.reversed()) { l in
            HStack(alignment: .top) {
                Text(l.time, format: .dateTime.hour().minute().second()).font(.caption.monospacedDigit()).foregroundStyle(.secondary)
                Text(Vehicle.SEVERITY[min(l.severity, 7)]).font(.caption.bold())
                    .foregroundStyle(l.severity <= 3 ? Color.bad : l.severity == 4 ? Color.warnC : Color.secondary)
                    .frame(width: 54, alignment: .leading)
                Text(l.text).font(.callout)
            }
        }
        .listStyle(.plain)
        .overlay { if m.vehicle.messages.isEmpty { Text("No messages yet").foregroundStyle(.secondary) } }
    }
}

struct ContentView: View {
    @EnvironmentObject var m: AppModel
    @Environment(\.scenePhase) private var phase

    var body: some View {
        VStack(spacing: 0) {
            StatusBar()
            Divider()
            TabView {
                DataView().tabItem { Label("Data", systemImage: "gauge.with.dots.needle.33percent") }
                PlanView().tabItem { Label("Plan", systemImage: "map") }
                #if os(iOS)
                // An iPhone shows four tabs and "More": keep the ones used while driving in front.
                LidarView().tabItem { Label("LiDAR", systemImage: "dot.radiowaves.left.and.right") }
                SetupView().tabItem { Label("Setup", systemImage: "wrench.and.screwdriver") }
                ParamsView().tabItem { Label("Params", systemImage: "slider.horizontal.3") }
                #else
                ParamsView().tabItem { Label("Params", systemImage: "slider.horizontal.3") }
                SetupView().tabItem { Label("Setup", systemImage: "wrench.and.screwdriver") }
                LidarView().tabItem { Label("LiDAR", systemImage: "dot.radiowaves.left.and.right") }
                #endif
                LogView().tabItem { Label("Log", systemImage: "text.alignleft") }
            }
        }
        .overlay(alignment: .bottom) { BannerView() }
        .animation(.easeInOut(duration: 0.2), value: m.banner)
        .onChange(of: phase) { _, p in
            // Leaving the app stops anything that drives (the LiDAR behaviours, the stick).
            if p != .active { m.stopDriving() }
        }
        #if os(macOS)
        .background(KeyDriver())
        #endif
    }
}

#if os(macOS)
/// Arrow keys / WASD drive the car while the window has the focus, like the Windows app.
/// Held keys are forgotten when the window loses the focus, so a key let go elsewhere
/// can't leave the car driving.
struct KeyDriver: NSViewRepresentable {
    @EnvironmentObject var m: AppModel

    func makeNSView(context: Context) -> NSView {
        let v = NSView()
        context.coordinator.install(m)
        return v
    }
    func updateNSView(_ nsView: NSView, context: Context) {}
    func makeCoordinator() -> Coordinator { Coordinator() }

    final class Coordinator {
        private var monitor: Any?
        private var resign: Any?
        private var keys = Set<UInt16>()
        private weak var model: AppModel?

        func install(_ m: AppModel) {
            model = m
            monitor = NSEvent.addLocalMonitorForEvents(matching: [.keyDown, .keyUp]) { [weak self] e in
                guard let self = self, let m = self.model else { return e }
                // Leave typing in text fields alone.
                if let r = NSApp.keyWindow?.firstResponder, r is NSTextView { return e }
                let drive: Set<UInt16> = [123, 124, 125, 126, 0, 1, 2, 13]   // ← → ↓ ↑ A S D W
                guard drive.contains(e.keyCode) else { return e }
                if e.type == .keyDown { self.keys.insert(e.keyCode) } else { self.keys.remove(e.keyCode) }
                self.apply(m)
                return nil
            }
            resign = NotificationCenter.default.addObserver(forName: NSWindow.didResignKeyNotification, object: nil, queue: .main) { [weak self] _ in
                guard let self = self, let m = self.model else { return }
                self.keys.removeAll(); self.apply(m)
            }
        }

        private func apply(_ m: AppModel) {
            let fwd: Float = (keys.contains(126) || keys.contains(13) ? 1 : 0) - (keys.contains(125) || keys.contains(1) ? 1 : 0)
            let right: Float = (keys.contains(124) || keys.contains(2) ? 1 : 0) - (keys.contains(123) || keys.contains(0) ? 1 : 0)
            m.stickX = right; m.stickY = fwd
            m.stickHeld = !keys.isEmpty
        }

        deinit {
            if let mo = monitor { NSEvent.removeMonitor(mo) }
            if let r = resign { NotificationCenter.default.removeObserver(r) }
        }
    }
}
#endif
