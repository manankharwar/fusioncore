#!/usr/bin/env python3
"""Score candidate GPS-track-heading turn gates against a real recorded bag.

Answers the only question that matters for issue #144: how far does the bearing
baseline actually get before the gate throws it away? Fusion needs 25 m.

Walks the run sample by sample, accumulating distance and rotation, latching when
the candidate gate trips, and reports the longest baseline reached.

Uses the GYRO for rotation, deliberately. Wheel encoders over-report rotation
under slip: across three runs on 2026-09-25 the encoder integral was 129 degrees
out on one of them while the gyro tracked the quaternion to within 2%. Scoring
with the encoder makes slip look like turning and gives a wrong answer.

Measured on those three bags:

    CURRENT rate > 0.3 rad/s     6.1 to  9.1 m     never reaches 25
    angle > 5 deg               26.7 to 27.2 m     clears, on all three

Usage:
    python3 tools/score_turn_gate.py <bag> [<bag> ...]

Needs only /imu/data and /odom/wheels, both present in any record.sh bag.
"""
import rosbag2_py, sys, math, bisect
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for bag in sys.argv[1:]:
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:t.type for t in r.get_all_topics_and_types()}
    enc=[]; gyr=[]
    while r.has_next():
        topic,data,t=r.read_next()
        if topic=='/odom/wheels':
            m=deserialize_message(data,get_message(types[topic]))
            enc.append((t/1e9, abs(m.twist.twist.linear.x)))
        elif topic=='/imu/data':
            m=deserialize_message(data,get_message(types[topic]))
            gyr.append((t/1e9, m.angular_velocity.z))
    et=[e[0] for e in enc]
    def speed_at(t):
        i=bisect.bisect_left(et,t)
        if i>=len(enc): i=len(enc)-1
        return enc[i][1]
    def sim(gate,param):
        best=0.0;dist=0.0;ang=0.0;lat=0;prev=gyr[0][0]
        for (t,wz) in gyr[1:]:
            dt=t-prev; prev=t
            if dt<=0 or dt>1.0: continue
            dist += speed_at(t)*dt
            ang  += wz*dt
            trip=(abs(wz)>param) if gate=='rate' else (abs(math.degrees(ang))>param)
            if trip:
                best=max(best,dist); dist=0.0; ang=0.0; lat+=1
        return max(best,dist),lat
    print(f"  {bag.split('/')[-1]}  (GYRO-based, {len(gyr)} samples)")
    b,l=sim('rate',0.3)
    print(f"    CURRENT rate > 0.30 rad/s     longest baseline {b:6.1f} m   latched {l:5d}x")
    for d in (5.0,10.0,15.0,20.0,30.0):
        b,l=sim('angle',d)
        mk="  <- clears 25 m" if b>=25.0 else ""
        print(f"    angle > {d:4.0f} deg                 longest baseline {b:6.1f} m   latched {l:5d}x{mk}")
    print()
