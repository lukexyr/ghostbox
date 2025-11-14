//
//  ContentView.swift
//  ghostbox
//
//  Modified to include Accelerometer data
//

import SwiftUI
import CoreBluetooth
import CoreMotion
import Combine

// MARK: - BLE Manager
class BLEManager: NSObject, CBPeripheralManagerDelegate, ObservableObject {
    @Published var isAdvertising = false
    @Published var connectedClients = 0
    @Published var currentAccel = (x: 0.0, y: 0.0, z: 0.0)
    @Published var currentMag = (x: 0.0, y: 0.0, z: 0.0)
    
    private var peripheralManager: CBPeripheralManager?
    private var motionDataCharacteristic: CBMutableCharacteristic?
    private var motionManager: CMMotionManager?
    private var displayLink: CADisplayLink?
    
    // BLE UUIDs
    let serviceUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdef0")
    let characteristicUUID = CBUUID(string: "87654321-4321-8765-4321-fedcba987654")
    
    override init() {
        super.init()
        peripheralManager = CBPeripheralManager(delegate: self, queue: .main)
        motionManager = CMMotionManager()
    }
    
    // MARK: - Setup BLE Service
    func setupBLEService() {
        motionDataCharacteristic = CBMutableCharacteristic(
            type: characteristicUUID,
            properties: [.notify],
            value: nil,
            permissions: [.readable]
        )
        
        let service = CBMutableService(type: serviceUUID, primary: true)
        service.characteristics = [motionDataCharacteristic!]
        
        peripheralManager?.add(service)
    }
    
    // MARK: - Start Advertising
    func startAdvertising() {
        let advertisementData: [String: Any] = [
            CBAdvertisementDataServiceUUIDsKey: [serviceUUID],
            CBAdvertisementDataLocalNameKey: "iPhoneMotion"
        ]
        
        peripheralManager?.startAdvertising(advertisementData)
    }
    
    // MARK: - Sensor Data Loop
    func startSensorUpdates() {
        guard let motionManager = motionManager else { return }
        
        if !motionManager.isDeviceMotionAvailable {
            print("Device Motion nicht verfügbar")
            return
        }
        
        motionManager.deviceMotionUpdateInterval = 1.0 / 60.0 // 60 Hz
        motionManager.startDeviceMotionUpdates()
        
        displayLink = CADisplayLink(
            target: self,
            selector: #selector(updateSensorData)
        )
        displayLink?.preferredFramesPerSecond = 60
        displayLink?.add(to: .main, forMode: .common)
    }
    
    @objc private func updateSensorData() {
        guard let motion = motionManager?.deviceMotion else { return }
        
        // Quaternion aus Attitude
        let quat = motionToQuaternion(motion.attitude)
        
        // Accelerometer-Daten (in G, Earth's gravity)
        let accel = motion.userAcceleration
        currentAccel = (accel.x, accel.y, accel.z)
        
        let mag = motion.magneticField
        currentMag = (mag.x, mag.y, mag.z)
        
        // Datenformat: 4x Float32 (Quaternion) + 3x Float32 (Accel) + 3x Float32 (Magno) = 40 bytes
        var data = Data()
        
        // Quaternion (16 bytes)
        data.append(contentsOf: withUnsafeBytes(of: quat.x) { Data($0) })
        data.append(contentsOf: withUnsafeBytes(of: quat.y) { Data($0) })
        data.append(contentsOf: withUnsafeBytes(of: quat.z) { Data($0) })
        data.append(contentsOf: withUnsafeBytes(of: quat.w) { Data($0) })
        
        // Accelerometer (12 bytes)
        let ax = Float(accel.x)
        let ay = Float(accel.y)
        let az = Float(accel.z)
        data.append(contentsOf: withUnsafeBytes(of: ax) { Data($0) })
        data.append(contentsOf: withUnsafeBytes(of: ay) { Data($0) })
        data.append(contentsOf: withUnsafeBytes(of: az) { Data($0) })
        
        // Magnetometer (12 bytes)
        let mx = Float(mag.x)
        let my = Float(mag.y)
        let mz = Float(mag.z)
        
        data.append(contentsOf: withUnsafeBytes(of: mx) { Data($0) })
        data.append(contentsOf: withUnsafeBytes(of: my) { Data($0) })
        data.append(contentsOf: withUnsafeBytes(of: mz) { Data($0) })
        
        
        
        // An alle verbundenen Clients senden
        if connectedClients > 0 {
            peripheralManager?.updateValue(
                data,
                for: motionDataCharacteristic!,
                onSubscribedCentrals: nil
            )
        }
    }
    
    // MARK: - CMAttitude zu Quaternion konvertieren
    private func motionToQuaternion(_ attitude: CMAttitude) -> (x: Float, y: Float, z: Float, w: Float) {
        let r = attitude.rotationMatrix
        
        let trace = r.m11 + r.m22 + r.m33
        var qx: Double, qy: Double, qz: Double, qw: Double
        
        if trace > 0 {
            let S = sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (r.m32 - r.m23) / S
            qy = (r.m13 - r.m31) / S
            qz = (r.m21 - r.m12) / S
        } else if r.m11 > r.m22 && r.m11 > r.m33 {
            let S = sqrt(1.0 + r.m11 - r.m22 - r.m33) * 2
            qw = (r.m32 - r.m23) / S
            qx = 0.25 * S
            qy = (r.m12 + r.m21) / S
            qz = (r.m13 + r.m31) / S
        } else if r.m22 > r.m33 {
            let S = sqrt(1.0 + r.m22 - r.m11 - r.m33) * 2
            qw = (r.m13 - r.m31) / S
            qx = (r.m12 + r.m21) / S
            qy = 0.25 * S
            qz = (r.m23 + r.m32) / S
        } else {
            let S = sqrt(1.0 + r.m33 - r.m11 - r.m22) * 2
            qw = (r.m21 - r.m12) / S
            qx = (r.m13 + r.m31) / S
            qy = (r.m23 + r.m32) / S
            qz = 0.25 * S
        }
        
        return (Float(qx), Float(qy), Float(qz), Float(qw))
    }
    
    // MARK: - CBPeripheralManagerDelegate
    func peripheralManagerDidUpdateState(_ peripheral: CBPeripheralManager) {
        switch peripheral.state {
        case .poweredOn:
            print("Bluetooth: Powered On")
            setupBLEService()
            startAdvertising()
            startSensorUpdates()
            isAdvertising = true
        case .poweredOff:
            print("Bluetooth: Powered Off")
            isAdvertising = false
        default:
            print("Bluetooth: State = \(peripheral.state)")
        }
    }
    
    func peripheralManager(
        _ peripheral: CBPeripheralManager,
        central: CBCentral,
        didSubscribeTo characteristic: CBCharacteristic
    ) {
        print("Central subscribed: \(central.identifier)")
        connectedClients += 1
    }
    
    func peripheralManager(
        _ peripheral: CBPeripheralManager,
        central: CBCentral,
        didUnsubscribeFrom characteristic: CBCharacteristic
    ) {
        print("Central unsubscribed: \(central.identifier)")
        connectedClients = max(0, connectedClients - 1)
    }
}

// MARK: - SwiftUI View
struct ContentView: View {
    @StateObject private var bleManager = BLEManager()
    
    var body: some View {
        VStack(spacing: 20) {
            Text("iPhone Motion + Accel")
                .font(.title)
            
            if bleManager.isAdvertising {
                HStack {
                    Circle()
                        .fill(Color.green)
                        .frame(width: 10, height: 10)
                    Text("Broadcasting via BLE")
                        .foregroundColor(.green)
                }
            } else {
                HStack {
                    Circle()
                        .fill(Color.red)
                        .frame(width: 10, height: 10)
                    Text("BLE nicht aktiv")
                        .foregroundColor(.red)
                }
            }
            
            Text("Connected Clients: \(bleManager.connectedClients)")
                .font(.headline)
            
            // Live Accelerometer Display
            VStack(alignment: .leading, spacing: 8) {
                Text("Live Accelerometer (G):")
                    .font(.headline)
                HStack {
                    Text("X:")
                    Text(String(format: "%.3f", bleManager.currentAccel.x))
                        .monospaced()
                        .foregroundColor(.red)
                }
                HStack {
                    Text("Y:")
                    Text(String(format: "%.3f", bleManager.currentAccel.y))
                        .monospaced()
                        .foregroundColor(.green)
                }
                HStack {
                    Text("Z:")
                    Text(String(format: "%.3f", bleManager.currentAccel.z))
                        .monospaced()
                        .foregroundColor(.blue)
                }
            }
            .padding()
            .background(Color.gray.opacity(0.1))
            .cornerRadius(8)
            
            VStack(alignment: .leading) {
                Text("Data Format:")
                    .font(.caption)
                    .foregroundColor(.gray)
                Text("28 bytes: Quat(x,y,z,w) + Accel(x,y,z)")
                    .font(.caption)
                    .monospaced()
                
                Text("Service UUID:")
                    .font(.caption)
                    .foregroundColor(.gray)
                Text("12345678-1234-5678-1234-56789abcdef0")
                    .font(.caption)
                    .monospaced()
            }
            .padding()
            .background(Color.gray.opacity(0.1))
            .cornerRadius(8)
            
            Spacer()
            
            Text("Bewege dein iPhone um Daten zu senden")
                .font(.caption)
                .foregroundColor(.gray)
        }
        .padding()
    }
}

#Preview {
    ContentView()
}
