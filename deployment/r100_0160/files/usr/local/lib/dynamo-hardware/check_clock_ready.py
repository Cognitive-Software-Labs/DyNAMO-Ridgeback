#!/usr/bin/env python3
"""Read-only bounded chrony convergence gate. Never steps/restarts the clock."""
import argparse
import csv
import json
import math
import subprocess
import time


def evaluate(text,max_correction,max_skew,max_reference_age,now):
    row=next(csv.reader([text.strip()]))
    if len(row)!=14:raise ValueError('unexpected chronyc tracking CSV fields')
    ref=int(row[0],16);stratum=int(row[2]);updated=float(row[3]);correction=float(row[4]);skew=float(row[9]);delay=float(row[10]);dispersion=float(row[11]);leap=row[13].strip()
    numbers=[updated,correction,skew,delay,dispersion,now]
    if not all(math.isfinite(v) for v in numbers):raise ValueError('nonfinite clock report')
    age=now-updated
    checks={'external_source':ref not in (0,0x7F7F0101) and 0<stratum<16,
            'normal_leap':leap=='Normal','correction_small':abs(correction)<=max_correction,
            'skew_bounded':0<=skew<=max_skew,'recent_reference':0<=age<=max_reference_age,
            'nonnegative_error_fields':delay>=0 and dispersion>=0}
    return {'ready':all(checks.values()),'checks':checks,'remaining_correction_s':correction,'skew_ppm':skew,'reference_age_s':age,
            'estimated_utc_error_bound_s':abs(correction)+dispersion+delay*.5,
            'limits':'Local chrony convergence only; not a measured Ridgeback–Thor offset or sensor timestamp guarantee.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout',type=float,default=30)
    parser.add_argument('--max-correction',type=float,default=.005)
    parser.add_argument('--max-skew',type=float,default=10)
    parser.add_argument('--max-reference-age',type=float,default=180)
    args=parser.parse_args()
    if any(not math.isfinite(v) or v<=0 for v in vars(args).values()) or args.timeout>900:parser.error('positive finite limits and timeout <=900 required')
    end=time.monotonic()+args.timeout;report={'ready':False}
    while True:
        try:
            result=subprocess.run(['/usr/bin/chronyc','-c','tracking'],stdin=subprocess.DEVNULL,capture_output=True,text=True,check=True,timeout=3)
            report=evaluate(result.stdout,args.max_correction,args.max_skew,args.max_reference_age,time.time())
        except (OSError,ValueError,subprocess.SubprocessError) as error:report={'ready':False,'error':type(error).__name__}
        if report['ready'] or time.monotonic()>=end:
            print(json.dumps(report,indent=2));return 0 if report['ready'] else 1
        time.sleep(min(1,max(0,end-time.monotonic())))

if __name__=='__main__':raise SystemExit(main())
