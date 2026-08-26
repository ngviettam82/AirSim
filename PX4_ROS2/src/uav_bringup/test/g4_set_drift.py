#!/usr/bin/env python3
"""Write G4's stimulus into localization_params.yaml before the node starts.

It cannot be a `ros2 param set`: gps_adapter_node calls drift_.configure() once
in its constructor, so a runtime set is accepted and does NOTHING -- a stimulus
that never happened, which would then be read as a product that failed to detect
it. The stimulus has to exist before the node does.

Edits only the `drift:` block under `gps_adapter_node:`; vio_adapter_node has a
`degrade.drift` block with the same key names, and touching it would change the
thing being measured against.

Usage: g4_set_drift.py <params.yaml> <true|false> <seed>
"""
import sys


def main():
    if len(sys.argv) != 4:
        print('usage: g4_set_drift.py <params.yaml> <true|false> <seed>')
        return 2
    path, enabled, seed = sys.argv[1], sys.argv[2], sys.argv[3]
    if enabled not in ('true', 'false'):
        print('FATAL: enabled must be true or false, got %r' % enabled)
        return 2

    lines = open(path, encoding='utf-8').read().split('\n')
    in_gps = False
    in_drift = False
    touched = []
    for index, line in enumerate(lines):
        stripped = line.strip()
        if stripped.endswith(':') and line[:1] not in (' ', '', '#'):
            in_gps = stripped == 'gps_adapter_node:'
            in_drift = False
            continue
        if in_gps and stripped == 'drift:':
            in_drift = True
            continue
        if in_drift:
            indent = len(line) - len(line.lstrip())
            if stripped and indent <= 4:
                in_drift = False
                continue
            if stripped.startswith('enabled:'):
                lines[index] = ' ' * indent + 'enabled: ' + enabled
                touched.append('enabled')
            elif stripped.startswith('seed:'):
                lines[index] = ' ' * indent + 'seed: ' + seed
                touched.append('seed')

    # R27-1: refuse silently doing nothing. A stimulus that was not written is
    # not a weaker stimulus, it is no experiment at all.
    if sorted(touched) != ['enabled', 'seed']:
        print('FATAL: expected to set enabled+seed under gps_adapter_node.drift, set %r'
              % touched)
        return 2

    open(path, 'w', encoding='utf-8').write('\n'.join(lines))
    print('  stimulus written: gps_adapter_node.drift.enabled=%s seed=%s' % (enabled, seed))
    return 0


if __name__ == '__main__':
    sys.exit(main())
