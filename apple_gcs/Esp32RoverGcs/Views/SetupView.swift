import SwiftUI

/// Initial setup: connection, accelerometer and compass calibration, radio,
/// proximity, RTK over NTRIP, and vehicle actions.
struct SetupView: View {
    @EnvironmentObject var m: AppModel
    @State private var confirmClear = false

    var body: some View {
        _ = m.tick
        let v = m.vehicle
        return Form {
            Section("Connection") {
                Picker("Protocol", selection: $m.transport) {
                    ForEach(AppModel.Transport.allCases) { Text($0.rawValue).tag($0) }
                }
                .pickerStyle(.segmented)
                TextField("Address (blank = listen, UDP)", text: $m.host)
                    .autocorrectionDisabled()
                    #if os(iOS)
                    .textInputAutocapitalization(.never)
                    .keyboardType(.numbersAndPunctuation)
                    #endif
                    .onSubmit { m.connect() }
                TextField("Port", text: $m.port)
                    .onSubmit { m.connect() }
                HStack {
                    Button("Connect") { m.connect() }.buttonStyle(.borderedProminent)
                    Button("Disconnect") { m.disconnect() }.buttonStyle(.bordered)
                }
                Text("TCP 192.168.4.1:5760 is the board's default. Join its WiFi `ardupilot123` first. For a UDP build choose UDP, leave the address empty and use port 14550.")
                    .font(.caption).foregroundStyle(.secondary)
                Text("\(v.linkText) · \(v.rxPackets) packets").font(.caption).foregroundStyle(.secondary)
            }

            Section("Accelerometer") {
                Text(m.accelText)
                HStack {
                    Button("6 positions") { if m.requireLink() { m.accelPos = 0; v.startAccelCal() } }
                    Button("Level only") { if m.requireLink() { v.accelCalLevelOnly() } }
                    Button("Done, next position") { if (1...6).contains(m.accelPos) { v.accelCalPosition(m.accelPos) } }
                        .buttonStyle(.borderedProminent)
                        .disabled(!(1...6).contains(m.accelPos))
                }
                .buttonStyle(.bordered)
            }

            Section("Compass") {
                Text(m.magText)
                ForEach(0..<3, id: \.self) { i in
                    if v.magCalPct[i] > 0 || v.magCalStatus[i] > 0 {
                        ProgressView(value: Double(v.magCalPct[i]), total: 100) { Text("Compass \(i + 1)").font(.caption) }
                    }
                }
                HStack {
                    Button("Start") { if m.requireLink() { v.startMagCal(); m.magText = "Rotate the car in every direction until all bars are full." } }
                    Button("Accept") { v.acceptMagCal() }
                    Button("Cancel") { v.cancelMagCal() }
                }
                .buttonStyle(.bordered)
            }

            Section("Radio") {
                ForEach(0..<8, id: \.self) { i in
                    HStack {
                        Text("CH\(i + 1)").font(.caption.monospaced()).frame(width: 40, alignment: .leading)
                        ProgressView(value: Double(max(800, min(2200, v.rc[i])) - 800), total: 1400)
                        Text("\(v.rc[i])").font(.caption.monospacedDigit()).frame(width: 44, alignment: .trailing)
                    }
                }
                Button(m.rcCal ? "Save calibration" : "Calibrate radio") { m.radioCalToggle() }
                    .buttonStyle(.bordered)
                Text("Channel 1 steering, 3 throttle, 8 mode switch.").font(.caption).foregroundStyle(.secondary)
            }

            Section("LiDAR proximity") {
                let now = clockMs()
                ForEach(0..<8, id: \.self) { i in
                    HStack {
                        Text(["Front", "Front right", "Right", "Back right", "Back", "Back left", "Left", "Front left"][i])
                        Spacer()
                        let fresh = now - v.proximityTime[i] < 2000
                        Text(!fresh ? "—" : v.proximity[i] == 0 ? "clear" : String(format: "%.2f m", v.proximity[i]))
                            .monospacedDigit()
                            .foregroundStyle(fresh && v.proximity[i] > 0 && v.proximity[i] < 1 ? Color.bad : Color.secondary)
                    }
                }
                Text("Put an object in front of the car: it must show at Front, or set PRX1_YAW_CORR.").font(.caption).foregroundStyle(.secondary)
            }

            Section("RTK / NTRIP") {
                TextField("Caster", text: $m.ntripHost).autocorrectionDisabled()
                    #if os(iOS)
                    .textInputAutocapitalization(.never)
                    #endif
                TextField("Port", text: $m.ntripPort)
                TextField("Mountpoint", text: $m.ntripMount).autocorrectionDisabled()
                    #if os(iOS)
                    .textInputAutocapitalization(.never)
                    #endif
                TextField("User", text: $m.ntripUser).autocorrectionDisabled()
                    #if os(iOS)
                    .textInputAutocapitalization(.never)
                    #endif
                SecureField("Password", text: $m.ntripPass)
                #if os(iOS)
                Toggle("Over mobile data (the board's WiFi has no internet)", isOn: $m.ntripCellular)
                #endif
                HStack {
                    Button(m.ntrip == nil ? "Start NTRIP" : "Restart") { m.startNtrip() }.buttonStyle(.borderedProminent)
                    Button("Stop") { m.stopNtrip() }.buttonStyle(.bordered).disabled(m.ntrip == nil)
                }
                if let n = m.ntrip {
                    Text(n.status).font(.callout)
                    Text("\(n.frames) frames · \(n.bytes / 1024) KB · sent \(v.rtcmBytes / 1024) KB").font(.caption).foregroundStyle(.secondary)
                    if !n.types.isEmpty {
                        Text(n.types.keys.sorted().map { "\($0)×\(n.types[$0]!)" }.joined(separator: " ")).font(.caption.monospaced())
                    }
                }
            }

            Section("Vehicle") {
                Button("Set home here (car's position)") { if m.requireLink() { v.setHomeHere() } }
                Button("Reboot the board") { if m.requireLink() { v.reboot() } }
                Button("Clear the mission on the vehicle", role: .destructive) { confirmClear = true }
            }
        }
        .formStyle(.grouped)
        .confirmationDialog("Clear the vehicle's mission?", isPresented: $confirmClear, titleVisibility: .visible) {
            Button("Clear", role: .destructive) { if m.requireLink() { v.clearMission(); m.show("Mission cleared") } }
        }
    }
}
