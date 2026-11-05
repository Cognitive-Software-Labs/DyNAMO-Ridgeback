"""Lazy F3 model adapters. Checkpoints and preprocessing are revision pinned."""
from pathlib import Path

from .reid import AppearanceEncoder

DINOV2_ID = 'facebook/dinov2-small'
DINOV2_REVISION = 'ed25f3a31f01632728cabb09d1542f84ab7b0056'


def dinov2_encoder(model_dir, device='cuda', batch_size=16, local_files_only=False):
    """DINOv2-small CLS features; a generic visual baseline, not trained ReID."""
    if batch_size < 1:
        raise ValueError('batch_size must be positive')
    import torch
    from PIL import Image
    from transformers import AutoImageProcessor, AutoModel
    cache = Path(model_dir).resolve() / 'huggingface'
    common = dict(revision=DINOV2_REVISION, cache_dir=str(cache),
                  local_files_only=local_files_only)
    processor = AutoImageProcessor.from_pretrained(DINOV2_ID, **common)
    model = AutoModel.from_pretrained(DINOV2_ID, **common).to(device).eval()

    def infer(images):
        batches = []
        with torch.inference_mode():
            for start in range(0, len(images), batch_size):
                inputs = processor(images=[Image.fromarray(image) for image in
                                           images[start:start+batch_size]],
                                   return_tensors='pt').to(device)
                output = model(**inputs)
                batches.append(output.last_hidden_state[:, 0].float().cpu())
        return torch.cat(batches).numpy()

    encoder = AppearanceEncoder('dinov2', infer, model.config.hidden_size)
    encoder.provenance = {'model': DINOV2_ID, 'revision': DINOV2_REVISION,
                          'feature': 'last_hidden_state_cls', 'device': device,
                          'preprocessing': processor.to_dict()}
    return encoder


def official_encoder(name, model_dir, device='cuda', batch_size=16, vendor_dir=None):
    """Load one official candidate per process (upstreams share module names)."""
    if name == 'dinov2':
        return dinov2_encoder(model_dir, device, batch_size)
    import hashlib
    import importlib.util
    import sys
    import torch
    from PIL import Image
    from torchvision import transforms as T
    directory = Path(model_dir).resolve()
    vendor = Path(vendor_dir).resolve() if vendor_dir else directory.parent / 'f3-vendor'
    checkpoint = directory / (name + '.pth')
    mean, std, size = [.5]*3, [.5]*3, (256, 128)

    def module_file(label, path):
        spec = importlib.util.spec_from_file_location(label, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    if name == 'osnet-ain':
        files = list((directory / 'osnet').glob('osnet_ain_x1_0_msmt17*.pth'))
        if len(files) != 1:
            raise FileNotFoundError('Expected exactly one official OSNet-AIN MSMT17 checkpoint')
        checkpoint = files[0]
        architecture = module_file('f3_osnet_ain', vendor / 'osnet/torchreid/models/osnet_ain.py')
        model = architecture.osnet_ain_x1_0(num_classes=1, pretrained=False)
        state = torch.load(checkpoint, map_location='cpu', weights_only=False)
        state = state.get('state_dict', state)
        state = {k.removeprefix('module.'): v for k, v in state.items()}
        state = {k: v for k, v in state.items() if not k.startswith('classifier.')}
        missing, unexpected = model.load_state_dict(state, strict=False)
        if unexpected or any(not k.startswith('classifier.') for k in missing):
            raise ValueError(f'OSNet checkpoint mismatch: {missing}, {unexpected}')
        mean, std = [.485, .456, .406], [.229, .224, .225]
        dimension = 512
        forward = model
    elif name == 'clip-reid':
        architecture = module_file('f3_clip_visual', vendor / 'clip-reid/model/clip/model.py')
        model = architecture.VisionTransformer(16, 8, 16, 16, 768, 12, 12, 512)
        state = torch.load(checkpoint, map_location='cpu', weights_only=False)
        model.load_state_dict({k[len('image_encoder.'):]: v for k, v in state.items()
                               if k.startswith('image_encoder.')}, strict=True)
        def forward(inputs):
            _, visual, projected = model(inputs)
            return torch.cat((visual[:, 0], projected[:, 0]), dim=1)
        dimension = 1280
    elif name in ('transreid', 'solider'):
        if any(key in sys.modules for key in ('config', 'model', 'loss')):
            raise RuntimeError('Load each upstream ReID candidate in a fresh process')
        sys.path.insert(0, str(vendor / name))
        from config import cfg
        config = 'configs/Market/vit_base.yml' if name == 'transreid' else 'configs/market/swin_tiny.yml'
        cfg.merge_from_file(str(vendor / name / config))
        cfg.MODEL.PRETRAIN_CHOICE = ''
        cfg.MODEL.PRETRAIN_PATH = ''
        cfg.TEST.NECK_FEAT = 'before'
        from model import make_model
        if name == 'solider':
            model = make_model(cfg, num_class=751, camera_num=0, view_num=0,
                               semantic_weight=cfg.MODEL.SEMANTIC_WEIGHT)
            size, dimension = (384, 128), 768
        else:
            model = make_model(cfg, num_class=751, camera_num=0, view_num=0)
            dimension = 768
        state = torch.load(checkpoint, map_location='cpu', weights_only=False)
        model.load_state_dict({k.removeprefix('module.'): v for k, v in state.items()}, strict=True)
        def forward(inputs):
            result = model(inputs)
            return result[0] if isinstance(result, tuple) else result
    else:
        raise ValueError(f'Unknown appearance candidate {name}')
    if batch_size < 1:
        raise ValueError('batch_size must be positive')
    model.to(device).eval()
    transform = T.Compose([T.Resize(size, interpolation=T.InterpolationMode.BILINEAR),
                           T.ToTensor(), T.Normalize(mean, std)])
    def infer(images):
        batches = []
        with torch.inference_mode():
            for start in range(0, len(images), batch_size):
                inputs = torch.stack([transform(Image.fromarray(image))
                                      for image in images[start:start+batch_size]]).to(device)
                batches.append(forward(inputs).float().cpu())
        return torch.cat(batches).numpy()
    encoder = AppearanceEncoder(name, infer, dimension)
    import subprocess
    source_name = 'osnet' if name == 'osnet-ain' else name
    source_revision = subprocess.check_output(['git','-C',str(vendor/source_name),'rev-parse','HEAD'],text=True).strip()
    source_diff = subprocess.check_output(['git','-C',str(vendor/source_name),'diff','--binary'])
    encoder.provenance = {'source_revision':source_revision,
                          'source_patch_sha256':hashlib.sha256(source_diff).hexdigest(),
                          'model': name, 'checkpoint': str(checkpoint),
                          'sha256': hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
                          'size': size, 'mean': mean, 'std': std,
                          'feature': 'before_neck', 'device': device}
    return encoder
