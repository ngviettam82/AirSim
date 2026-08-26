#!/usr/bin/env python3
"""P10.9b D0: reduces preflight_baseline_capture.py's raw JSONL (one line
per OBSERVATION) into a candidate waiver table -- one row per DISTINCT
(source, child), with every level/action/message combination seen, first/
last-seen wall time, and observation count. This table (not the raw JSONL)
is what a human reviews before signing any row into
config/preflight_waivers.yaml -- printed as both a human-readable table and
a ready-to-paste YAML block (still requires a human to pick `when` and
write `waiver_reason`; this script never invents either).

Usage: preflight_baseline_report.py --jsonl FILE1.jsonl [FILE2.jsonl ...]
"""
import argparse
import json
from collections import OrderedDict


def load(path):
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            records.append(json.loads(line))
    return records


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--jsonl', nargs='+', required=True)
    args = parser.parse_args()

    groups = OrderedDict()   # (source, child) -> dict
    for path in args.jsonl:
        for record in load(path):
            key = (record['source'], record['child'])
            if key not in groups:
                groups[key] = {
                    'levels': set(), 'actions': set(), 'messages': set(),
                    'count': 0, 'first_t_wall': record['t_wall'], 'last_t_wall': record['t_wall'],
                }
            g = groups[key]
            g['levels'].add(record['level'])
            g['actions'].add(record.get('action') or '')
            g['messages'].add(record.get('message') or '')
            g['count'] += 1
            g['first_t_wall'] = min(g['first_t_wall'], record['t_wall'])
            g['last_t_wall'] = max(g['last_t_wall'], record['t_wall'])

    if not groups:
        print('NO NON-OK Sub-B CHILDREN OBSERVED -- nothing to review (parked stack was fully '
              'clean in every input file).')
        return

    print('=' * 100)
    print('CANDIDATE WAIVER TABLE -- for project owner review, NOT auto-signed')
    print('=' * 100)
    for (source, child), g in groups.items():
        span_sec = g['last_t_wall'] - g['first_t_wall']
        print('-' * 100)
        print('source=%r  child=%r' % (source, child))
        print('  levels seen:   %s' % sorted(g['levels']))
        print('  actions seen:  %s' % sorted(g['actions']))
        print('  messages seen: %s' % sorted(g['messages']))
        print('  observations:  %d over %.1fs (first=%.3f last=%.3f)' %
              (g['count'], span_sec, g['first_t_wall'], g['last_t_wall']))

    print()
    print('=' * 100)
    print('YAML SKELETON (paste rows you decide are TRUE+HARMLESS into '
          'config/preflight_waivers.yaml -- waiver_reason left blank on purpose, '
          'a human must write it)')
    print('=' * 100)
    print('    waiver_source: [')
    for (source, _child), _g in groups.items():
        print('      "%s",' % source)
    print('    ]')
    print('    waiver_child: [')
    for (_source, child), _g in groups.items():
        print('      "%s",' % child)
    print('    ]')
    print('    waiver_when: [')
    for _key in groups:
        print('      "preflight",  # OR "perception_off" -- REVIEW, this is a guess')
    print('    ]')
    print('    waiver_reason: [')
    for _key in groups:
        print('      "TODO: project owner must write why this is TRUE and HARMLESS here",')
    print('    ]')


if __name__ == '__main__':
    main()
