import SwiftUI
import MapKit

extension Coord {
    var cl: CLLocationCoordinate2D { CLLocationCoordinate2D(latitude: lat, longitude: lon) }
}

/// The map shared by the Data and Plan tabs: the car (with its heading), its trail,
/// home, the mission, the fence, the zig-zag points and the Guided target.
/// Taps are reported as coordinates.
struct MapPanel: View {
    @EnvironmentObject var m: AppModel
    var editing: Bool = false               // Plan tab: number the waypoints, highlight the fence zone
    var showFence: Bool = true
    var onTap: (Coord) -> Void = { _ in }

    @State private var camera: MapCameraPosition = .automatic
    @State private var centred = false
    @AppStorage("map_satellite") private var satellite = true

    var body: some View {
        _ = m.tick
        let v = m.vehicle
        let mission = m.plan.filter { $0.hasPosition }
        return MapReader { proxy in
            Map(position: $camera, interactionModes: [.pan, .zoom]) {
                // trail
                if v.trail.count > 1 {
                    MapPolyline(coordinates: v.trail.map { CLLocationCoordinate2D(latitude: $0.0, longitude: $0.1) })
                        .stroke(.orange.opacity(0.8), lineWidth: 2)
                }
                // mission
                if mission.count > 1 {
                    MapPolyline(coordinates: mission.map { CLLocationCoordinate2D(latitude: $0.lat, longitude: $0.lon) })
                        .stroke(.yellow, lineWidth: 3)
                }
                ForEach(Array(mission.enumerated()), id: \.element.id) { i, it in
                    Annotation("", coordinate: CLLocationCoordinate2D(latitude: it.lat, longitude: it.lon)) {
                        Text("\(i + 1)")
                            .font(.caption2.bold())
                            .frame(width: 22, height: 22)
                            .background(Circle().fill(v.missionSeq == i + 1 && v.customMode == 10 ? Color.good : Color.yellow))
                            .foregroundStyle(.black)
                    }
                }
                // fence
                if showFence {
                    ForEach(Array(m.fence.enumerated()), id: \.element.id) { zi, z in
                        let col: Color = z.kind.inclusion ? .green : .red
                        let w: CGFloat = (editing && m.fenceZone == zi) ? 4 : 2
                        if z.kind.polygon && z.points.count >= 3 {
                            MapPolygon(coordinates: z.points.map { $0.cl })
                                .foregroundStyle(col.opacity(0.15))
                                .stroke(col, lineWidth: w)
                        } else if z.kind.polygon && z.points.count == 2 {
                            MapPolyline(coordinates: z.points.map { $0.cl }).stroke(col, lineWidth: w)
                        } else if z.kind.circle, let c = z.points.first {
                            MapCircle(center: c.cl, radius: CLLocationDistance(z.radius))
                                .foregroundStyle(col.opacity(0.15))
                                .stroke(col, lineWidth: w)
                        } else if z.kind == .returnPoint, let c = z.points.first {
                            Annotation("Return", coordinate: c.cl) {
                                Image(systemName: "arrow.uturn.backward.circle.fill").foregroundStyle(.blue)
                            }
                        }
                    }
                }
                // zig-zag A and B
                if let a = m.zigzag.a {
                    Annotation("A", coordinate: a.cl) { Image(systemName: "a.circle.fill").font(.title3).foregroundStyle(.purple) }
                }
                if let b = m.zigzag.b {
                    Annotation("B", coordinate: b.cl) { Image(systemName: "b.circle.fill").font(.title3).foregroundStyle(.purple) }
                }
                // home
                if v.hasHome {
                    Annotation("Home", coordinate: CLLocationCoordinate2D(latitude: v.homeLat, longitude: v.homeLon)) {
                        Image(systemName: "house.circle.fill").font(.title2).foregroundStyle(.white, .green)
                    }
                }
                // guided target
                if let g = m.guidedTarget, v.customMode == 15 {
                    Annotation("Target", coordinate: g.cl) {
                        Image(systemName: "scope").font(.title2).foregroundStyle(.pink)
                    }
                }
                // the car
                if v.hasPosition {
                    Annotation("", coordinate: CLLocationCoordinate2D(latitude: v.lat, longitude: v.lon)) {
                        Image(systemName: "location.north.fill")
                            .font(.title)
                            .foregroundStyle(v.armed ? Color.bad : Color.accentColor)
                            .shadow(color: .white, radius: 2)
                            .rotationEffect(.radians(Double(v.yaw)))
                    }
                }
            }
            .mapStyle(satellite ? .hybrid(elevation: .flat) : .standard(elevation: .flat))
            .onTapGesture(coordinateSpace: .local) { pt in
                if let c = proxy.convert(pt, from: .local) { onTap(Coord(lat: c.latitude, lon: c.longitude)) }
            }
            .overlay(alignment: .topTrailing) {
                VStack(spacing: 8) {
                    Button { centre() } label: { Image(systemName: "location.fill").frame(width: 34, height: 34) }
                    Button { satellite.toggle() } label: { Image(systemName: satellite ? "map" : "globe.americas.fill").frame(width: 34, height: 34) }
                }
                .buttonStyle(.plain)
                .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 8))
                .padding(8)
            }
            .onChange(of: v.hasPosition) { _, has in if has && !centred { centre() } }
            .onAppear { if v.hasPosition && !centred { centre() } }
        }
    }

    private func centre() {
        let v = m.vehicle
        let c: CLLocationCoordinate2D
        if v.hasPosition { c = CLLocationCoordinate2D(latitude: v.lat, longitude: v.lon) }
        else if let f = m.plan.first(where: { $0.hasPosition }) { c = CLLocationCoordinate2D(latitude: f.lat, longitude: f.lon) }
        else { return }
        centred = true
        withAnimation { camera = .region(MKCoordinateRegion(center: c, latitudinalMeters: 80, longitudinalMeters: 80)) }
    }
}
