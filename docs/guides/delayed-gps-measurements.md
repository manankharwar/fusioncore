# Handling delayed GPS measurements in ROS 2 sensor fusion

GPS fixes typically arrive 50-200 ms after the moment they were computed. By the time the NavSatFix message reaches your filter, the robot has moved. If your filter doesn't account for this delay, it fuses a position that corresponds to a different point in the robot's trajectory and the estimate accumulates error proportional to speed times delay.

This page explains how delay causes errors, how to diagnose it in your ROS 2 setup, and what both robot_localization and FusionCore provide to handle it.

---

## Why GPS arrives late

The delay from physical observation to ROS timestamp has several sources:

- **Receiver processing:** the receiver takes 10-50 ms to compute the fix from satellite signals
- **Serial/USB latency:** UART at 9600 baud introduces up to 10 ms per byte; a full NMEA sentence at that rate takes ~50 ms
- **Driver buffering:** the ROS driver parses the sentence and publishes, adding 5-20 ms
- **Network latency:** if the receiver publishes over TCP or a serial bridge, add network jitter

Total: 50-200 ms is typical. At 1 m/s, 100 ms delay = 10 cm position error per fix. At 3 m/s (typical outdoor robot), it's 30 cm per fix, accumulating into meters over a long run.

---

## How to measure your GPS delay

Record a rosbag with both IMU and GPS, then compare timestamps vs receive times:

```bash
ros2 bag play your_bag.bag --pause
ros2 bag info your_bag.bag
```

For a rough estimate, check the lag between when the GPS timestamp says the fix happened and when it was received:

```python
import rclpy
from sensor_msgs.msg import NavSatFix
import time

def cb(msg):
    ros_time = rclpy.clock.Clock().now().nanoseconds / 1e9
    msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    print(f"GPS delay: {(ros_time - msg_time)*1000:.1f} ms")
```

If your driver publishes with `stamp.sec = 0`, it is not timestamping the fix at observation time. The message arrives with zero timestamp and the filter applies it "now", which is wrong by the full processing delay.

---

## Diagnosing delay-induced error

Delay-induced error looks different from GPS noise:

- Position error is correlated with speed: faster runs have worse trajectory error
- The error is consistent in direction: straight segments show the estimate behind the true path
- Plotting GPS fix positions vs filter output shows the filter output consistently ahead of the GPS fixes

---

## robot_localization: smooth_lagged_data

robot_localization provides `smooth_lagged_data: true` with `history_length` to handle late-arriving sensor messages. When enabled, rl buffers recent filter state and rewinds to re-process a late measurement at its correct timestamp.

```yaml
smooth_lagged_data: true
history_length: 0.5    # seconds of history to keep
```

The limitation: rewinding is computationally expensive and the re-linearization at the old state is approximate in EKF mode. For delays up to 100-200 ms at typical robot speeds, it works reasonably well. For longer delays or higher speeds the approximation error grows.

Also: if your GPS driver publishes with zero-stamped headers (`stamp.sec = 0`), rl cannot tell what time the fix corresponds to and smooth_lagged_data doesn't help. Fix the driver or use a timestamp correction node first.

---

## FusionCore: IMU ring buffer retrodiction

FusionCore keeps a 1-second ring buffer of IMU measurements. When a GPS fix arrives with a timestamp earlier than the current filter time, FusionCore:

1. Identifies the filter state at the GPS timestamp from the ring buffer
2. Applies the GPS update to that historical state
3. Replays all buffered IMU steps forward to reconstruct the current state exactly

This is exact replay, not approximation. The same sigma points are re-propagated forward from the GPS-updated historical state. No linearization artifact.

```yaml
# No configuration needed: IMU ring buffer is always on
# Handles any delay up to 1 second automatically
```

The buffer is always active and **there is nothing to configure from ROS.**

The size is a COUNT of IMU messages, not a duration: `imu_buffer_size`, default 100,
and it is a `FusionCoreConfig` field rather than a ROS parameter. That means the
delay it covers depends on your IMU rate:

```
delay covered  =  imu_buffer_size / IMU rate

  100 messages at 400 Hz  ->  0.25 s
  100 messages at 100 Hz  ->  1.0 s
  100 messages at  20 Hz  ->  5.0 s
```

So "one second" is only true at 100 Hz. Work out your own number before relying on
it. Changing it requires linking `fusioncore_core` directly; it cannot be set from a
params file.

Zero-stamped GPS is handled **automatically**, with no parameter. When a driver
publishes a fix with a zero stamp, FusionCore falls back to the receive time rather
than treating it as an ancient measurement.

---

## Comparing the two approaches

| Aspect | robot_localization | FusionCore |
|---|---|---|
| Mechanism | State rewind + re-processing | IMU ring buffer exact replay |
| Max delay handled | Configurable via `history_length` | 0.5 s default, `max_measurement_delay` |
| Accuracy | Approximate (re-linearization at old state) | Exact (sigma point replay) |
| Zero-stamped GPS | Not handled | Not handled either: a zero stamp is treated as a real time |
| CPU cost | Proportional to delay and sensor rate | Constant (O(N) buffer lookup) |
| Configuration | `smooth_lagged_data: true` + `history_length` | Always on |

---

## Practical recommendations

**For robot_localization users:**

1. Fix the GPS driver to stamp messages at observation time if possible
2. Set `smooth_lagged_data: true` and `history_length: 0.5` as a minimum
3. Measure your actual delay and set `history_length` at least 2x that value
4. If using a u-blox ZED-F9P: enable the `timemark` feature to get hardware-timestamped fixes

**For FusionCore users:**

Delay compensation is automatic, and there is no parameter to turn it on. If you
see delay-correlated errors, the causes worth checking are:

- **A GPS driver that stamps on arrival rather than at observation.** This is the
  common one and FusionCore cannot detect it: the stamp looks perfectly valid, it is
  just late by a constant amount. Fix it in the driver.
- **Delay beyond the gate.** A fix older than `max_measurement_delay` (0.5 s by
  default, declared without a `gnss.` prefix) is rejected outright rather than
  replayed. Raise it if your link is genuinely slower than that, but understand you
  are then fusing half-second-old position.
- **A zero or unset stamp.** FusionCore treats it as a real time, which places the
  measurement at the epoch and it is then rejected as impossibly old. There is no
  wall-clock fallback. Fix the driver.

The IMU ring buffer holds `imu_buffer_size` samples, 100 by default, which is one
second at 100 Hz. That is a `FusionCoreConfig` field rather than a ROS parameter, so
it cannot be changed from a YAML.

---

## Related

- [Configuration reference: GNSS section](../configuration.md)
- [FusionCore vs robot_localization](../vs-robot-localization.md)
- [GPS position jumping near buildings](gps-multipath-position-jumping.md)
