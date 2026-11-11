#!/usr/bin/env python3
"""Compare issued marker draws with received marker IDs over their common span."""
import argparse
import csv
import json
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--server',type=Path,required=True);p.add_argument('--client',type=Path,required=True)
a=p.parse_args();draws=list(csv.DictReader(a.server.open()));client=json.loads(a.client.read_text());events=client['events']
if len(events)<2:raise SystemExit('Need at least two valid received marker IDs')
server={int(row['frame_id']):float(row['monotonic_seconds']) for row in draws}
first,last=events[0],events[-1]
if first['id'] not in server or last['id'] not in server:raise SystemExit('Client IDs do not match this server capture')
if any(b['id']<=a['id'] for a,b in zip(events,events[1:])):raise SystemExit('Non-monotonic IDs; repeat with an unobstructed marker')
span=last['id']-first['id'];elapsed_server=server[last['id']]-server[first['id']];elapsed_client=(last['wall_ms']-first['wall_ms'])/1000
print(json.dumps({'issued_marker_draws_per_second':span/elapsed_server,'received_marker_updates_per_second':(len(events)-1)/elapsed_client,
 'unobserved_marker_ids':span-(len(events)-1),'invalid_client_samples':client['invalid_samples'],
 'scope':'Marker delivery only; not complete simulator frames or latency.'},indent=2))
