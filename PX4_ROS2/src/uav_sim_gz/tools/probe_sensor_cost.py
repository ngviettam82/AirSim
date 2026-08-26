#!/usr/bin/env python3
"""Attribute RTF cost to individual render sensors, isolated from PX4/ROS/drone."""

import argparse
import os
import shutil
import statistics
import subprocess
import tempfile

CAMERA_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='{name}'>
    <static>true</static>
    <link name="{name}/base_link">
      <visual name="{name}/visual">
        <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
      </visual>
      <sensor name="{name}_sensor" type="camera">
        <topic>probe/{name}/image</topic>
        <update_rate>{rate}</update_rate>
        <always_on>1</always_on>
        <visualize>false</visualize>
        <camera>
          <horizontal_fov>1.2217305</horizontal_fov>
          <image>
            <width>{width}</width>
            <height>{height}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.1</near><far>100</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""

DEPTH_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='{name}'>
    <static>true</static>
    <link name="{name}/base_link">
      <visual name="{name}/visual">
        <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
      </visual>
      <sensor name="{name}_sensor" type="depth_camera">
        <topic>probe/{name}/depth</topic>
        <update_rate>{rate}</update_rate>
        <always_on>1</always_on>
        <visualize>false</visualize>
        <camera>
          <horizontal_fov>1.274</horizontal_fov>
          <image>
            <width>{width}</width>
            <height>{height}</height>
            <format>R_FLOAT32</format>
          </image>
          <clip><near>0.2</near><far>19.1</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""

# Named after what they represent, not their numbers.
PROBES = [
    ('baseline', []),
    ('oakd_rgb_1080p', [('camera', 1920, 1080, 30)]),
    ('rgb_720p', [('camera', 1280, 720, 30)]),
    ('rgb_480p', [('camera', 640, 480, 30)]),
    ('rgb_1080p_at_10hz', [('camera', 1920, 1080, 10)]),
    ('oakd_depth_480p', [('depth', 640, 480, 30)]),
    ('oakd_depth_at_10hz', [('depth', 640, 480, 10)]),
    ('rgb_480p_plus_depth_480p', [('camera', 640, 480, 30), ('depth', 640, 480, 30)]),
    ('oakd_as_shipped', [('camera', 1920, 1080, 30), ('depth', 640, 480, 30)]),
]


def write_probe_model(models_dir, name, kind, width, height, rate):
    path = os.path.join(models_dir, name)
    os.makedirs(path, exist_ok=True)
    template = CAMERA_TEMPLATE if kind == 'camera' else DEPTH_TEMPLATE
    with open(os.path.join(path, 'model.sdf'), 'w', newline='\n') as handle:
        handle.write(template.format(name=name, width=width, height=height, rate=rate))
    with open(os.path.join(path, 'model.config'), 'w', newline='\n') as handle:
        handle.write(
            '<?xml version="1.0"?><model><name>%s</name>'
            '<sdf version="1.9">model.sdf</sdf></model>\n' % name)


def build_world(base_world, out_path, includes):
    with open(base_world, encoding='utf-8') as handle:
        text = handle.read()
    block = ''
    for index, name in enumerate(includes):
        block += (
            "    <include><uri>model://%s</uri>"
            "<pose>%d 0 1.0 0 0 0</pose></include>\n" % (name, index * 2))
    text = text.replace('</world>', block + '</world>')
    with open(out_path, 'w', newline='\n', encoding='utf-8') as handle:
        handle.write(text)


def attach_subscribers(topics, env):
    """Hold a subscription so the sensor renders; lesson doc 2.5h."""
    handles = []
    for topic in topics:
        handles.append(subprocess.Popen(
            ['gz', 'topic', '-e', '-t', topic],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    return handles


def sensor_topics(label, sensors):
    topics = []
    for index, (kind, _, _, _) in enumerate(sensors):
        name = '%s_%d' % (label, index)
        if kind == 'camera':
            topics.append('/probe/%s/image' % name)
        else:
            topics.append('/probe/%s/depth' % name)
            # Cloud is a separate connection that may drive extra work.
            topics.append('/probe/%s/depth/points' % name)
    return topics


def sample_rtf(world_name, samples, settle_sec):
    """Median instantaneous RTF, after letting the first steps settle."""
    subprocess.run(['sleep', str(settle_sec)], check=False)
    try:
        output = subprocess.run(
            ['gz', 'topic', '-e', '-t', '/world/%s/stats' % world_name,
             '-n', str(samples)],
            capture_output=True, text=True, timeout=180).stdout
    except subprocess.TimeoutExpired:
        return None
    values = [float(line.split()[1]) for line in output.splitlines()
              if 'real_time_factor' in line]
    if not values:
        return None
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default=os.path.expanduser(
        '~/PX4_ROS2/install/uav_sim_gz/share/uav_sim_gz/worlds/uav_arena.sdf'))
    parser.add_argument('--samples', type=int, default=60)
    parser.add_argument('--settle', type=int, default=12)
    parser.add_argument(
        '--subscribe', action='store_true',
        help='Hold a subscription on each sensor topic. Without this a render '
             'sensor may never render, and the probe measures nothing.')
    args = parser.parse_args()
    print('subscribers attached: %s' % ('yes' if args.subscribe else 'NO - '
          'render sensors may be idle, treat results as a lower bound'))

    scratch = tempfile.mkdtemp(prefix='rtf_probe_')
    models_dir = os.path.join(scratch, 'models')
    os.makedirs(models_dir)

    env = dict(os.environ)
    env['GZ_SIM_RESOURCE_PATH'] = models_dir + ':' + env.get('GZ_SIM_RESOURCE_PATH', '')
    env['MESA_D3D12_DEFAULT_ADAPTER_NAME'] = 'NVIDIA'
    env.pop('LIBGL_ALWAYS_SOFTWARE', None)

    # Mean, not median: RTF is bimodal, see README section 7.
    print('%-26s %7s %7s %7s %7s  %s'
          % ('probe', 'mean', 'p50', 'min', 'max', 'sensors'))
    print('-' * 86)

    results = {}
    for label, sensors in PROBES:
        includes = []
        for index, (kind, width, height, rate) in enumerate(sensors):
            name = '%s_%d' % (label, index)
            write_probe_model(models_dir, name, kind, width, height, rate)
            includes.append(name)

        world_path = os.path.join(scratch, '%s.sdf' % label)
        build_world(args.world, world_path, includes)

        server = subprocess.Popen(
            ['gz', 'sim', '-s', '-r', '-v', '1', world_path],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        listeners = []
        try:
            if args.subscribe and sensors:
                subprocess.run(['sleep', '6'], check=False)
                listeners = attach_subscribers(sensor_topics(label, sensors), env)
            values = sample_rtf('uav_arena', args.samples, args.settle)
        finally:
            for listener in listeners:
                listener.terminate()
            server.terminate()
            try:
                server.wait(timeout=20)
            except subprocess.TimeoutExpired:
                server.kill()
            subprocess.run(['sleep', '3'], check=False)

        if not values:
            print('%-26s %7s' % (label, 'NO DATA'))
            continue
        values.sort()
        mean = statistics.fmean(values)
        results[label] = mean
        described = ', '.join(
            '%s %dx%d@%dHz' % (k, w, h, r) for k, w, h, r in sensors) or 'none'
        print('%-26s %7.3f %7.3f %7.3f %7.3f  %s'
              % (label, mean, values[len(values) // 2], values[0], values[-1],
                 described))

    shutil.rmtree(scratch, ignore_errors=True)

    print()
    if 'baseline' in results:
        print('cost relative to an empty world (mean RTF drop):')
        for label, mean in results.items():
            if label != 'baseline':
                print('  %-26s %+.3f' % (label, mean - results['baseline']))


if __name__ == '__main__':
    main()
