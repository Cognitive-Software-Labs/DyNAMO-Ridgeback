#!/usr/bin/env python3
"""Replay received-field flags from passive candump timing; never access ROS/CAN.

This is a scheduling model, not execution of the full driver or proof of field
age/physical synchronization. Diagnostic phase and timer phase are swept.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import statistics

ROOT=Path(__file__).resolve().parents[2]
DEVICES=(5,4,2,3)
FIELDS={0x1400:0,0x1480:1,0x1500:2,0x1540:3,0x0480:4}
FIELD_BYTES=(2,2,4,4,4)


def receive_field_update(line):
    """Return a recognized RX field and whether its speed-mode payload is valid."""
    verbose=re.match(r'\s*\(([0-9.]+)\)\s+vcan0\s+(RX|TX)\s+[-A-Z ]+\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s*(.*)$',line)
    if verbose:
        if verbose[2]!='RX':return None
        stamp,ident=float(verbose[1]),int(verbose[3],16)
        dlc=int(verbose[4]);payload=verbose[5].split()
        payload_bytes=dlc
        well_formed=len(payload)==dlc and all(re.fullmatch(r'[0-9A-Fa-f]{2}',byte) for byte in payload)
    else:
        compact=re.match(r'\s*\(([0-9.]+)\)\s+vcan0\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)\s+([RT])\s*$',line)
        if not compact or compact[4]!='R':return None
        stamp,ident=float(compact[1]),int(compact[2],16)
        payload_bytes=len(compact[3])//2
        well_formed=len(compact[3])%2==0
    device=ident&0x3f;api=ident&0xffc0
    field=FIELDS.get(api)
    return (stamp,device,field,well_formed and payload_bytes==FIELD_BYTES[field]) if device in DEVICES and field is not None else None


def receive_field(line):
    """Return only valid RX fields for the historical flag-model comparison."""
    update=receive_field_update(line)
    return update[:3] if update is not None and update[3] else None


def field_receive_gaps(events):
    """Report per-wheel RX gaps; missing fields remain explicit."""
    rows=[]
    for device in DEVICES:
        for field in range(len(FIELD_BYTES)):
            stamps=[stamp for stamp,wheel,index in events if wheel==device and index==field]
            gaps=[b-a for a,b in zip(stamps,stamps[1:])]
            rows.append({'wheel':device,'field':field,'frames':len(stamps),
                         'max_gap_ms':max(gaps)*1000 if gaps else None,
                         'median_gap_ms':statistics.median(gaps)*1000 if gaps else None})
    return rows


def replay(events,phase,strict):
    first,last=events[0][0],events[-1][0]
    flags={d:[False]*5 for d in DEVICES};updated={d:[None]*5 for d in DEVICES}
    index=0;tick=first+phase;diagnostic=first+1.;published=[];ages=[];stale_batches=0
    while tick<=last:
        while index<len(events) and events[index][0]<=tick:
            t,d,field=events[index];flags[d][field]=True;updated[d][field]=t;index+=1
        # Current diagnostics consume flags, candidate diagnostics copy caches.
        if tick>=diagnostic:
            if not strict:
                for d in DEVICES:flags[d]=[False]*5
            diagnostic+=1.
        ready=(all(all(flags[d]) for d in DEVICES) if strict else
               all(any(flags[d][f] for d in DEVICES) for f in range(5)))
        if ready:
            if tick-first>=2.:
                published.append(tick)
                if any(not all(flags[d]) for d in DEVICES):stale_batches+=1
                for d in DEVICES:
                    ages.extend(tick-v for v in updated[d] if v is not None)
            for d in DEVICES:flags[d]=[False]*5
        tick+=.05
    duration=max(0,last-first-2.)
    return {'phase_s':phase,'samples_after_two_second_warmup':len(published),
            'projected_hz':len(published)/duration if duration else None,
            'max_publication_gap_s':max((b-a for a,b in zip(published,published[1:])),default=None),
            'batches_with_unreceived_wheel_fields':stale_batches,
            'max_cached_field_age_s':max(ages,default=None)}


def replay_motion_fields(events, phase, age_bound_s=.2):
    """Model the candidate's field-age policy, including invalidation on bad RX."""
    if not events or age_bound_s <= 0:
        raise ValueError('nonempty events and positive age bound required')
    first,last=events[0][0],events[-1][0]
    updated={(d,f):None for d in DEVICES for f in range(5)}
    fresh={(d,f):False for d in DEVICES for f in (2,3)}
    index=0;tick=first+phase;published=[];ages=[]
    while tick<=last:
        while index<len(events) and events[index][0]<=tick:
            event=events[index];t,d,field=event[:3]
            valid=event[3] if len(event)>3 else True
            updated[d,field]=t if valid else None
            if (d,field) in fresh:fresh[d,field]=valid
            index+=1
        if (all(fresh.values()) and all(v is not None and 0<=tick-v<=age_bound_s
                                        for v in updated.values())):
            if tick-first>=2.:
                published.append(tick)
                ages.extend(tick-v for v in updated.values())
            fresh=dict.fromkeys(fresh,False)
        tick+=.05
    duration=max(0,last-first-2.)
    observation_start=first+2.
    boundaries=[observation_start,*published,last]
    return {'phase_s':phase,'age_bound_s':age_bound_s,
            'samples_after_two_second_warmup':len(published),
            'projected_hz':len(published)/duration if duration else None,
            'max_publication_gap_s':max((b-a for a,b in zip(published,published[1:])),default=None),
            'max_silence_including_edges_s':max((b-a for a,b in zip(boundaries,boundaries[1:])),default=None),
            'max_cached_field_age_s':max(ages,default=None)}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    output=args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/hardware'):p.error('save below artifacts/hardware')
    if output.exists():p.error('use a fresh output file')
    events=[];candidate_events=[]
    for line in args.capture.read_text().splitlines():
        update=receive_field_update(line)
        if update is not None:
            candidate_events.append(update)
            if update[3]:events.append(update[:3])
    events.sort();candidate_events.sort()
    if not events:raise ValueError('capture has no relevant RX fields')
    result={'capture_sha256':hashlib.sha256(args.capture.read_bytes()).hexdigest(),
            'relevant_receive_frames':len(events),'device_ids':DEVICES,
            'api_field_mapping':{hex(k):v for k,v in FIELDS.items()},
            'valid_payload_bytes_by_field':FIELD_BYTES,
            'invalid_relevant_receive_frames':len(candidate_events)-len(events),
            'field_receive_gaps':field_receive_gaps(events),
            'baseline':[replay(events,i*.005,False) for i in range(10)],
            'candidate':[replay(events,i*.005,True) for i in range(10)],
            'motion_fields_candidate':[replay_motion_fields(candidate_events,i*.005) for i in range(10)],
            'motion_fields_sensitivity':[
                {'experimental_age_bound_ms':bound_ms,
                 'phase_results':[replay_motion_fields(candidate_events,i*.005,bound_ms/1000)
                                  for i in range(10)]}
                for bound_ms in (150,175,200,225,250)],
            'limits':'20 Hz timer simulation using validated speed-mode payload widths; malformed relevant RX invalidates candidate cache. Diagnostics every 1 s; no ROS queue, live transport/control, or physical synchronization. Two-second warmup excluded. Not a deployed rate guarantee.'}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2)+'\n')
    print('relevant_rx', {'valid':len(events),'invalid':len(candidate_events)-len(events)})
    for label in ['baseline','candidate']:
        rows=result[label];print(label,{'rate_range_hz':[min(x['projected_hz'] for x in rows),max(x['projected_hz'] for x in rows)],
            'max_gap_s':max(x['max_publication_gap_s'] for x in rows),
            'max_age_s':max(x['max_cached_field_age_s'] for x in rows),
            'unreceived_batches_range':[min(x['batches_with_unreceived_wheel_fields'] for x in rows),max(x['batches_with_unreceived_wheel_fields'] for x in rows)]})
    rows=result['motion_fields_candidate']
    print('motion_fields_candidate',{'age_bound_s':.2,
        'rate_range_hz':[min(x['projected_hz'] for x in rows),max(x['projected_hz'] for x in rows)],
        'max_gap_s':max(x['max_publication_gap_s'] for x in rows),
        'max_silence_including_edges_s':max(x['max_silence_including_edges_s'] for x in rows),
        'max_age_s':max(x['max_cached_field_age_s'] for x in rows)})


if __name__=='__main__':main()
