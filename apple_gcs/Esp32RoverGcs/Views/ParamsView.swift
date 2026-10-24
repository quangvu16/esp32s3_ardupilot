import SwiftUI

/// The full parameter list: search, edit (write now or stage), .parm load/save, reboot.
struct ParamsView: View {
    @EnvironmentObject var m: AppModel
    @State private var search = ""
    @State private var editing: Param?
    @State private var importing = false
    @State private var exporting = false
    @State private var confirmReboot = false

    var body: some View {
        _ = m.tick
        let v = m.vehicle
        let all = v.sortedParams
        let q = search.uppercased()
        let shown = q.isEmpty ? all : all.filter { $0.name.contains(q) }
        return VStack(spacing: 0) {
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 6) {
                    Button("Refresh") { if m.requireLink() { v.fetchParams() } }.buttonStyle(.borderedProminent)
                    Button("Write staged (\(m.staged.count))") { m.writeStaged() }.disabled(m.staged.isEmpty)
                    Button("Discard staged") { m.staged.removeAll() }.disabled(m.staged.isEmpty)
                    Button("Load file") { importing = true }.disabled(all.isEmpty)
                    Button("Save file") { exporting = true }.disabled(all.isEmpty)
                    Button("Reboot") { confirmReboot = true }
                }
                .buttonStyle(.bordered)
                .padding(.horizontal, 10).padding(.vertical, 6)
            }
            Text(v.paramsFetching ? "Reading \(all.count) of \(v.paramCount)…" :
                    all.isEmpty ? "Press Refresh to read the parameters" : "\(all.count) parameters" +
                    (v.paramWritesPending > 0 ? " · writing \(v.paramWritesPending)" : ""))
                .font(.caption).foregroundStyle(.secondary)
            TextField("Search, e.g. PRX or FS_", text: $search)
                .textFieldStyle(.roundedBorder)
                .autocorrectionDisabled()
                #if os(iOS)
                .textInputAutocapitalization(.characters)
                #endif
                .padding(.horizontal, 10).padding(.vertical, 4)
            List(shown) { p in
                Button { editing = p } label: {
                    HStack {
                        Text(p.name).font(.body.monospaced())
                        Spacer()
                        if let s = m.staged[p.name] {
                            Text("\(Patrol.fmt(p.value)) → \(Patrol.fmt(s))").foregroundStyle(Color.warnC).monospacedDigit()
                        } else {
                            Text(Patrol.fmt(p.value)).foregroundStyle(.secondary).monospacedDigit()
                        }
                    }
                }
                .buttonStyle(.plain)
            }
            .listStyle(.plain)
        }
        .sheet(item: $editing) { p in ParamEditor(param: p) }
        .fileImporter(isPresented: $importing, allowedContentTypes: [.plainText, .data]) { r in
            if case .success(let url) = r, let s = readPickedFile(url) { m.loadParmFile(s) }
        }
        .fileExporter(isPresented: $exporting, document: TextFile(m.parmFileText()), contentType: .plainText,
                      defaultFilename: "rover.parm") { _ in }
        .confirmationDialog("Reboot the board?", isPresented: $confirmReboot, titleVisibility: .visible) {
            Button("Reboot", role: .destructive) { if m.requireLink() { v.reboot() } }
        }
    }
}

struct ParamEditor: View {
    @EnvironmentObject var m: AppModel
    @Environment(\.dismiss) private var dismiss
    let param: Param
    @State private var text = ""

    var body: some View {
        NavigationStack {
            Form {
                Text(param.name).font(.headline.monospaced())
                HStack {
                    Text("Now")
                    Spacer()
                    Text(Patrol.fmt(param.value)).foregroundStyle(.secondary)
                }
                TextField("New value", text: $text)
                    #if os(iOS)
                    .keyboardType(.numbersAndPunctuation)
                    #endif
                Section {
                    Button("Write now") {
                        guard let v = Float(text) else { m.show("Not a number"); return }
                        if m.requireLink() {
                            m.paramWriteOk = 0; m.paramWriteFail = []
                            m.vehicle.setParam(param.name, v)
                        }
                        dismiss()
                    }
                    Button("Stage") {
                        guard let v = Float(text) else { m.show("Not a number"); return }
                        m.staged[param.name] = v
                        dismiss()
                    }
                    if m.staged[param.name] != nil {
                        Button("Unstage", role: .destructive) { m.staged.removeValue(forKey: param.name); dismiss() }
                    }
                }
            }
            .formStyle(.grouped)
            .navigationTitle("Parameter")
            .toolbar { ToolbarItem(placement: .cancellationAction) { Button("Cancel") { dismiss() } } }
            .onAppear { text = Patrol.fmt(m.staged[param.name] ?? param.value) }
        }
        .frame(minWidth: 320, minHeight: 300)
    }
}
