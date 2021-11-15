# Meshtastic-ZPS
Zero-GPS Positioning System for Meshtastic

This is experimental draft work. It may not break your device, but **it will try**. Use at own risk.

## ZPS protocol outline
A mesh network device (the client) scans the network environment and generates a compact packet from the strongest signals. It then attaches a timestamp (which may be wildly incorrect!) and broadcasts it to the network on the ZPS port.

Upon receiving the packet, another mesh node that has the necessary capability (by querying a local positioning service like FIND3), resolves the network scan to a lat/lon position and returns the solution as a direct message to the sender’s ZPS port.

The client device uses the received data to set its Position fields accordingly.

## Roles in the ZPS system
### Sensor (surveyor)
Continuously scans the network environment and supplies the resolver(s) with georeferenced datasets comprising network address and signal strength of Wifi/BLE devices in range, for learning purposes.
Multiple surveyors can operate in parallel on the same network.
System requirements: free (unused) WiFi and BLE capability, a reasonably accurate GPS.

### Client
Is a GPS-less (or GPS-denied) device. Scans the network environment periodically and sends out LOCATE broadcasts, for the purpose of determining its own location with the assistance of a resolver node.
Multiple clients can operate in parallel on the same network.
System requirements: free (unused) WiFi and BLE capability.

### Server (resolver)
Listens for ZPS broadcasts (SURVEY and LOCATE) and relays them to the local positioning service (LPS). If the LPS returns a positive result to a LOCATE query, the resolver relays it to the client as a RESULT message.
Multiple resolvers can operate in parallel on the network (but why?)
Requirements: Internet or Intranet connection to an LPS service.

### Local Positioning Service (LPS)
An Internet/Intranet/Onion/etc service that can estimate a device’s location using a snapshot of the device’s network environment (in our case Wifi and BLE signals). 
The LPS can be privately operated (like FIND3) or a public cloud-based service (like Mozilla MLS). In any case, the LPS is technically not part of the mesh network and, for the most part, will be treated as a black box with a query interface to which our resolver node connects.

## Message types

| type       | from   | to     | pos | scan | size |
|------------|--------|--------|-----|------|------|
| **SURVEY** | sensor | all    | YES | YES  | 2+n  |
| **LOCATE** | client | all    | NO  | YES  | 1+n  |
| **RESULT** | server | client | YES | NO   | 2    |

### SURVEY
Information message sent by sensor device to broadcast address, contains a network scan and the associated lat/lon location.

### LOCATE
Query message sent by client device to broadcast address, contains a network scan but no lat/lon location.

### RESULT
Location message sent by server to client address in response to a LOCATE packet, contains a resolved lat/lon location only (no network scan data).

## Message format details
All ZPS packets are structured as arrays of int64 fields. 

The first int64 is the header, which encodes information about the packet content, and a timestamp
An optional second int64 encodes a lat/lon tuple, as 32-bit signed integers
The rest of the int64’s until the end of packet represent networks/devices detected **in a single scan**.

### Header (mandatory)
```
|--------|--------|--------|--------|--------|--------|--------|--------|
| RSVD   | FLAGS  | DOP    | RSVD   | TIMESTAMP(uint32)                 |
|--------|--------|--------|--------|--------|--------|--------|--------|
63       55       47       39       31                                  0
```
### Position field (optional)
```
|--------|--------|--------|--------|--------|--------|--------|--------|
| LATITUDE(sint32)                  | LONGITUDE(sint32)                 |
|--------|--------|--------|--------|--------|--------|--------|--------|
63                                  31                                  0
```
### Network field (0-n)
```
|--------|--------|--------|--------|--------|--------|--------|--------|
| RSSI   | CHAN   | BSSID or BLE_ADDR                                   |
|--------|--------|--------|--------|--------|--------|--------|--------|
63       55       47                                                    0
```

## Notes, issues and reflections
### Exclusive access to ISM radio
Currently, the ZPS plugin expects to have exclusive access to the onboard ISM (WiFi/Bluetooth) radio. This makes it incompatible with network connected or phone-paired devices. 

### Physical limits
We need to be honest with ourselves about the outer limits of what can be achieved.

With an ESP32, scanning for BSS and for BLE can’t happen in parallel (same radio is used).
- a BSS scan takes 4 seconds.
- a BLE scan takes 3 seconds.

The time resolution of this method is rather poor to start with. 
At a walking speed of 5.5km/h, a person moves ~1.5 meters in a second - that’s **6 meters** in the time it takes to resolve a WiFi position. A hybrid scan, which should have the best positional accuracy, has a **10+ meters** uncertainty due to the time factor alone. This is not simply a time lag; the actual **sharpness** of the positioning process is degraded, due to a smearing effect similar to the “motion blur” effect in imaging. 
