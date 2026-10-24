import SwiftUI
import UniformTypeIdentifiers

extension Color {
    static let good = Color(red: 0.20, green: 0.70, blue: 0.35)
    static let warnC = Color(red: 0.95, green: 0.65, blue: 0.10)
    static let bad = Color(red: 0.88, green: 0.25, blue: 0.22)
    static let muted = Color.secondary
}

/// A small coloured status pill.
struct Pill: View {
    let text: String
    let color: Color
    var body: some View {
        Text(text)
            .font(.caption.weight(.semibold))
            .lineLimit(1)
            .padding(.horizontal, 8).padding(.vertical, 4)
            .background(color.opacity(0.18), in: Capsule())
            .overlay(Capsule().stroke(color.opacity(0.6), lineWidth: 1))
            .foregroundStyle(color)
    }
}

/// The bar at the top of every tab: link, mode, armed, GPS, battery, lap.
struct StatusBar: View {
    @EnvironmentObject var m: AppModel
    var body: some View {
        let v = m.vehicle
        _ = m.tick
        return ScrollView(.horizontal, showsIndicators: false) {
            HStack(spacing: 6) {
                Pill(text: v.connected ? v.linkText : v.linkText, color: v.connected ? .good : .bad)
                Pill(text: v.modeName, color: .accentColor)
                Pill(text: v.armed ? "ARMED" : "Disarmed", color: v.armed ? .bad : .muted)
                Pill(text: "\(Vehicle.GPS_FIX[min(v.gpsFix, Vehicle.GPS_FIX.count - 1)]) · \(v.gpsSats) sats",
                     color: v.gpsFix >= 6 ? .good : v.gpsFix >= 3 ? .warnC : .bad)
                if v.voltage > 0 {
                    Pill(text: String(format: "%.1f V", v.voltage) + (v.batteryPct >= 0 ? " \(v.batteryPct)%" : ""), color: .muted)
                }
                if v.customMode == 10 && Patrol.laps(m.plan) != 1 {
                    Pill(text: "Lap \(m.lapCounter.lap)" + (Patrol.laps(m.plan) > 0 ? "/\(Patrol.laps(m.plan))" : ""), color: .accentColor)
                }
                if !v.missionProgress.isEmpty { Pill(text: v.missionProgress, color: .warnC) }
            }
            .padding(.horizontal, 10).padding(.vertical, 6)
        }
    }
}

/// The message banner at the bottom of the window.
struct BannerView: View {
    @EnvironmentObject var m: AppModel
    var body: some View {
        if let b = m.banner {
            Text(b)
                .font(.callout)
                .padding(.horizontal, 14).padding(.vertical, 10)
                .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 12))
                .shadow(radius: 4)
                .padding(.bottom, 60)
                .transition(.opacity)
                .onTapGesture { m.banner = nil }
        }
    }
}

/// A plain-text document for the file importer/exporter (.parm and .waypoints files).
struct TextFile: FileDocument {
    static var readableContentTypes: [UTType] { [.plainText, .data] }
    var text: String
    init(_ text: String = "") { self.text = text }
    init(configuration: ReadConfiguration) throws {
        text = configuration.file.regularFileContents.map { String(decoding: $0, as: UTF8.self) } ?? ""
    }
    func fileWrapper(configuration: WriteConfiguration) throws -> FileWrapper {
        FileWrapper(regularFileWithContents: Data(text.utf8))
    }
}

/// Reads a file the user picked (security-scoped on iOS and in a sandbox).
func readPickedFile(_ url: URL) -> String? {
    let scoped = url.startAccessingSecurityScopedResource()
    defer { if scoped { url.stopAccessingSecurityScopedResource() } }
    return try? String(contentsOf: url, encoding: .utf8)
}

/// A labelled numeric field bound to a Float.
struct NumberField: View {
    let label: String
    @Binding var value: Float
    var unit = ""
    var body: some View {
        HStack {
            Text(label)
            Spacer()
            TextField(label, value: $value, format: .number)
                .multilineTextAlignment(.trailing)
                .frame(maxWidth: 90)
                #if os(iOS)
                .keyboardType(.decimalPad)
                #endif
            if !unit.isEmpty { Text(unit).foregroundStyle(.secondary) }
        }
    }
}

/// A labelled slider for a Float setting.
struct SliderRow: View {
    let label: String
    @Binding var value: Float
    let range: ClosedRange<Float>
    var step: Float = 0.05
    var format = "%.2f"
    var body: some View {
        VStack(alignment: .leading, spacing: 2) {
            HStack { Text(label); Spacer(); Text(String(format: format, value)).monospacedDigit().foregroundStyle(.secondary) }
            Slider(value: $value, in: range, step: step)
        }
    }
}

func modeButtons() -> [(Int, String)] {
    [(0, "Manual"), (1, "Acro"), (4, "Hold"), (10, "Auto"), (15, "Guided"), (11, "RTL"), (12, "SmartRTL"), (3, "Steering"), (5, "Loiter")]
}
