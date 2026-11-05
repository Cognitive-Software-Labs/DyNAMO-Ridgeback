"""Lazy camera candidate adapters; all return xyxy, confidence in source pixels."""
from pathlib import Path
import os
import numpy as np

CAMERA_MODELS = {
    'yolo11': ('ultralytics','yolo11n.pt'),
    'yolo26': ('ultralytics','yolo26n.pt'),
    'rtdetrv2': ('transformers','PekingU/rtdetr_v2_r18vd'),
    'dfine': ('transformers','ustc-community/dfine-small-coco'),
    'owlv2': ('owlv2','google/owlv2-base-patch16-ensemble'),
    'rfdetr': ('rfdetr','RFDETRSmall'),
    'deim': ('deim','deim-small.pth'),
}


HF_REVISIONS = {
    'rtdetrv2':'5650961749fa93567c0d46fc7f43ea4f9e914107',
    'dfine':'f79e65b5fbb33ceb9d3ebba042955d7410c608f8',
    'owlv2':'cfd3195ba4ea9592eec887ded089f4c08eff231d',
}


class CameraDetector:
    def __init__(self, name, model_dir, device='cuda', threshold=None):
        os.environ.setdefault('HF_HOME',str(Path(model_dir).resolve()/'huggingface'))
        os.environ.setdefault('TORCH_HOME',str(Path(model_dir).resolve()/'torch'))
        import torch
        if name not in CAMERA_MODELS: raise ValueError(f'Unknown camera candidate {name}')
        self.name,self.device,self.threshold=name,device,(.1 if name == 'owlv2' else .25) if threshold is None else threshold
        family,identifier=CAMERA_MODELS[name]
        self.family=family
        self.identifier=identifier
        if family == 'ultralytics':
            from ultralytics import YOLO
            self.model=YOLO(str(Path(model_dir)/identifier))
        elif family in ('transformers','owlv2'):
            from transformers import AutoImageProcessor, AutoModelForObjectDetection, Owlv2Processor, Owlv2ForObjectDetection
            if family == 'owlv2':
                self.processor=Owlv2Processor.from_pretrained(identifier,revision=HF_REVISIONS[name])
                self.model=Owlv2ForObjectDetection.from_pretrained(identifier,revision=HF_REVISIONS[name]).to(device).eval()
            else:
                self.processor=AutoImageProcessor.from_pretrained(identifier,revision=HF_REVISIONS[name])
                self.model=AutoModelForObjectDetection.from_pretrained(identifier,revision=HF_REVISIONS[name]).to(device).eval()
                self.person_labels={int(k) for k,v in self.model.config.id2label.items() if v.lower() == 'person'}
                if not self.person_labels: raise ValueError('Model has no person label')
        elif family == 'deim':
            import sys
            vendor=Path(model_dir).resolve().parent/'f2-vendor/deim'
            sys.path.insert(0,str(vendor))
            from engine.core import YAMLConfig
            cfg=YAMLConfig(str(vendor/'configs/deim_dfine/deim_hgnetv2_s_coco.yml'))
            cfg.yaml_cfg['HGNetv2']['pretrained']=False
            checkpoint=torch.load(Path(model_dir)/identifier,map_location='cpu',weights_only=False)
            cfg.model.load_state_dict(checkpoint['ema']['module'] if 'ema' in checkpoint else checkpoint['model'])
            self.model=cfg.model.deploy().to(device).eval()
            self.postprocessor=cfg.postprocessor.deploy()
        else:
            from rfdetr import RFDETRSmall
            self.model=RFDETRSmall(device=device,pretrain_weights=str(Path(model_dir)/'rf-detr-small.pth'))
        self.torch=torch

    def detect(self, rgb):
        if self.family == 'ultralytics':
            # Ultralytics ndarray inputs are BGR.
            boxes=self.model.predict(rgb[:,:,::-1],classes=[0],conf=self.threshold,device=self.device,verbose=False)[0].boxes
            return np.column_stack((boxes.xyxy.cpu().numpy(),boxes.conf.cpu().numpy()))
        if self.family == 'rfdetr':
            result=self.model.predict(rgb,threshold=self.threshold)
            keep=result.class_id == 1
            return np.column_stack((result.xyxy[keep],result.confidence[keep]))
        from PIL import Image
        image=Image.fromarray(rgb)
        if self.family == 'deim':
            from torchvision.transforms.functional import to_tensor
            inputs=to_tensor(image.resize((640,640))).unsqueeze(0).to(self.device)
            with self.torch.inference_mode():
                labels,boxes,scores=self.postprocessor(self.model(inputs),self.torch.tensor([[rgb.shape[1],rgb.shape[0]]],device=self.device))
            labels,boxes,scores=labels[0].cpu().numpy(),boxes[0].cpu().numpy(),scores[0].cpu().numpy()
            keep=(labels == 0) & (scores >= self.threshold)
            return np.column_stack((boxes[keep],scores[keep]))
        if self.family == 'owlv2':
            inputs=self.processor(text=[['a person']],images=image,return_tensors='pt').to(self.device)
        else:
            inputs=self.processor(images=image,return_tensors='pt').to(self.device)
        with self.torch.inference_mode():
            outputs=self.model(**inputs)
        post=self.processor.image_processor if self.family == 'owlv2' else self.processor
        result=post.post_process_object_detection(outputs,threshold=self.threshold,
            target_sizes=self.torch.tensor([rgb.shape[:2]],device=self.device))[0]
        boxes,scores,labels=[result[k].cpu().numpy() for k in ('boxes','scores','labels')]
        keep=np.ones(len(labels),dtype=bool) if self.family == 'owlv2' else np.isin(labels,list(self.person_labels))
        return np.column_stack((boxes[keep],scores[keep]))
