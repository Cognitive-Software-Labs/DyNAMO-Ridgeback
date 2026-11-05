"""Bounding-box detector strata and official TrackEval HOTA/IDF1 integration."""
import numpy as np
from scipy.optimize import linear_sum_assignment


def box_iou(a,b):
    a,b=np.asarray(a,dtype=float).reshape(-1,4),np.asarray(b,dtype=float).reshape(-1,4)
    lo=np.maximum(a[:,None,:2],b[None,:,:2]);hi=np.minimum(a[:,None,2:],b[None,:,2:])
    overlap=np.maximum(hi-lo,0).prod(axis=2)
    areas=np.maximum(a[:,2:]-a[:,:2],0).prod(axis=1)
    other=np.maximum(b[:,2:]-b[:,:2],0).prod(axis=1)
    union=areas[:,None]+other[None,:]-overlap
    return np.divide(overlap,union,out=np.zeros_like(overlap),where=union > 0)


def tracking_metrics(frames):
    """frames: [(truth [(id,xyxy)], prediction [(id,xyxy)])]; missing frames stay."""
    from trackeval.metrics import HOTA,Identity
    gt_names=sorted({str(i) for truth,_ in frames for i,_ in truth})
    pred_names=sorted({str(i) for _,pred in frames for i,_ in pred})
    gi={v:i for i,v in enumerate(gt_names)};pi={v:i for i,v in enumerate(pred_names)}
    data={'num_timesteps':len(frames),'num_gt_ids':len(gi),'num_tracker_ids':len(pi),
          'num_gt_dets':sum(len(a) for a,b in frames),'num_tracker_dets':sum(len(b) for a,b in frames),
          'gt_ids':[],'tracker_ids':[],'similarity_scores':[]}
    for truth,pred in frames:
        data['gt_ids'].append(np.array([gi[str(i)] for i,box in truth],dtype=int))
        data['tracker_ids'].append(np.array([pi[str(i)] for i,box in pred],dtype=int))
        data['similarity_scores'].append(box_iou([b for i,b in truth],[b for i,b in pred]))
    hota=HOTA().eval_sequence(data)
    identity=Identity({'PRINT_CONFIG':False,'THRESHOLD':.5}).eval_sequence(data)
    return {'HOTA':float(np.mean(hota['HOTA'])),'DetA':float(np.mean(hota['DetA'])),
            'AssA':float(np.mean(hota['AssA'])),'IDF1':float(identity['IDF1']),
            'scope':'TrackEval bounding-box IoU HOTA over 0.05:0.05:0.95; IDF1 at IoU 0.5'}


def detection_metrics(annotations,predictions):
    """Global IoU association then recall by truth distance/visible fraction.

    In each stratum, ignore detections matched to other strata and retain all
    unmatched false positives. Stratum FP counts are shared, not additive.
    """
    bins={k:{'matched':0,'total':0} for k in ('0-2m','2-4m','4-8m','visibility-0-.25','visibility-.25-.5','visibility-.5-.75','visibility-.75-1')}
    tp=fp=fn=0
    for truth,pred in zip(annotations,predictions,strict=True):
        people=[p for p in truth['people'] if p['box'] is not None and p['visible_fraction'] > 0]
        boxes=pred.get('boxes',[])
        ious=box_iou([p['box'] for p in people],[b[:4] for b in boxes])
        matched=set()
        if len(people) and len(boxes):
            penalty=min(ious.shape)+1
            rows,cols=linear_sum_assignment(np.where(ious >= .5,1-ious,penalty))
            matched={int(i) for i,j in zip(rows,cols) if ious[i,j] >= .5}
        tp+=len(matched);fp+=len(boxes)-len(matched);fn+=len(people)-len(matched)
        for i,p in enumerate(people):
            d=p['distance_m'];v=p['visible_fraction']
            keys=[]
            if 0 <= d < 8: keys.append('0-2m' if d < 2 else '2-4m' if d < 4 else '4-8m')
            keys.append('visibility-0-.25' if v < .25 else 'visibility-.25-.5' if v < .5 else 'visibility-.5-.75' if v < .75 else 'visibility-.75-1')
            for key in keys:
                bins[key]['total']+=1;bins[key]['matched']+=i in matched
    for values in bins.values():
        values['false_positives']=fp
        values['recall']=values['matched']/values['total'] if values['total'] else None
        values['precision']=values['matched']/(values['matched']+fp) if values['total'] and values['matched']+fp else None
    return {'TP':tp,'FP':fp,'FN':fn,'precision':tp/(tp+fp) if tp+fp else 0.,'recall':tp/(tp+fn) if tp+fn else 0.,
            'strata':bins,'scope':'person IoU >= 0.5, projected in-image boxes; stratum precision ignores detections matched to other strata, retains every unmatched FP in each stratum'}
