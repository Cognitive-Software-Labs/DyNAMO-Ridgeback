#!/usr/bin/env python3
"""Render standalone F2 comparison figures from retained result JSON."""
import argparse
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,required=True)
    args=parser.parse_args();root=args.directory
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    data=json.loads((root/'results.json').read_text());fusion=json.loads((root/'selected-fusion.json').read_text())
    fig,axes=plt.subplots(1,2,figsize=(11,5),layout='constrained')
    for ax,category in zip(axes,['camera','lidar']):
        for row in data['summaries']:
            if row['category'] != category:continue
            x=row['image_recall'] if category=='camera' else row['position_recall']
            y=row['image_precision'] if category=='camera' else row['position_precision']
            selected=row['candidate'] in ('owlv2','lfe-ppn')
            ax.scatter(x,y,s=100 if selected else 45,marker='*' if selected else 'o',color='#15803d' if selected else '#2563eb')
            offset=(-6,-15) if row['candidate']=='leg-detector' else (4,-12) if row['candidate']=='petra' else (4,7)
            ax.annotate(row['candidate'],(x,y),xytext=offset,textcoords='offset points',fontsize=8,
                        ha='right' if row['candidate']=='leg-detector' else 'left')
        ax.set(xlim=(-.05,1.13),ylim=(0,1.08),xlabel='Recall',ylabel='Precision',
               title='Camera: box IoU ≥ 0.5' if category=='camera' else 'Lidar: position gate ≤ 0.75 m')
        ax.grid(alpha=.2)
    fig.savefig(root/'candidates.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(12,5),layout='constrained');rows=fusion['results'];x=list(range(len(rows)))
    ax.bar([v-.18 for v in x],[r['leader_track_loss_events'] for r in rows],.36,label='Leader-track losses',color='#2563eb')
    ax.bar([v+.18 for v in x],[r['id_switches'] for r in rows],.36,label='Position ID switches',color='#d97706')
    labels=[r['backend']+' / '+r['scenario']+(' '+r['run'].split('crowd-')[1].split('-')[0] if r['scenario']=='crowd' else '') for r in rows]
    ax.set_xticks(x,labels,rotation=60,ha='right',fontsize=8)
    ax.set_ylabel('Events in one 20 s recording');ax.set_title('Selected OWLv2 + LFE-PPN + ByteTrack / Kalman profile')
    ax.legend();ax.grid(axis='y',alpha=.2);fig.savefig(root/'fusion-errors.png',dpi=160);plt.close(fig)


if __name__ == '__main__':main()
