#!/usr/bin/env python2
"""Validate one LiDAR config; create a separate host-clock config on first use."""
from __future__ import print_function
import os
import sys
import yaml


def validate(data):
    if not isinstance(data, dict) or len(data.get('lidar', [])) != 1:
        raise ValueError('This startup helper expects exactly one configured LiDAR')
    sensor = data['lidar'][0]
    driver = sensor['driver']
    address = str(driver['host_address'])
    parts = address.split('.')
    if len(parts) != 4 or any(not p.isdigit() or not 0 <= int(p) <= 255 for p in parts):
        raise ValueError('host_address must be the Pi receiving IPv4 address')
    if parts[0] in ('0', '127') or int(parts[0]) >= 224 or parts[-1] in ('0', '255'):
        raise ValueError('host_address must be a unicast receiving address in a /24 subnet')
    if int(data['common']['msg_source']) != 1 or not data['common']['send_point_cloud_ros']:
        raise ValueError('Live LiDAR input and point-cloud publishing must be enabled')
    for key in ('msop_port', 'difop_port'):
        if not 1 <= int(driver[key]) <= 65535:
            raise ValueError('Invalid UDP port: '+key)
    topic = sensor['ros']['ros_send_point_cloud_topic']
    model = str(driver['lidar_type'])
    if not isinstance(topic, str) or not topic.startswith('/') or any(c.isspace() for c in topic):
        raise ValueError('Invalid point-cloud topic')
    if any(c.isspace() for c in model):
        raise ValueError('Invalid LiDAR model')
    return address, topic, model


def prepare(source, destination):
    path = destination if os.path.exists(destination) else source
    with open(path) as handle:
        data = yaml.safe_load(handle)
    result = validate(data)
    if not os.path.exists(destination):
        data['lidar'][0]['driver']['use_lidar_clock'] = False
        encoded = yaml.safe_dump(data, default_flow_style=False).encode('utf-8')
        descriptor = os.open(destination, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        with os.fdopen(descriptor, 'wb') as handle:
            handle.write(encoded)
    elif data['lidar'][0]['driver'].get('use_lidar_clock', True):
        raise ValueError('LiDAR-clock config needs verified external clock synchronization; use controller clock for initial setup')
    return result


if __name__ == '__main__':
    try:
        print(' '.join(prepare(sys.argv[1], sys.argv[2])))
    except Exception as error:
        print('LiDAR configuration error: '+str(error), file=sys.stderr)
        sys.exit(1)
