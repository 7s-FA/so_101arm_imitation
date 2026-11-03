"""Identical fixed camera ROI masking for training and deployment.

Coordinates are original 640x480 pixels, x1/y1 inclusive, x2/y2 exclusive.
This is not object detection, jig alignment verification, or motion planning.
"""
import hashlib
import json
from pathlib import Path


def validate(config):
    if config.get('version') != 1 or config.get('size') != [640,480]:
        raise ValueError('ROI format must be version 1, size [640,480]')
    if config.get('wrist') not in ('black','full'):
        raise ValueError('ROI wrist mode must explicitly be black or full')
    boxes=config.get('overview_boxes')
    if not isinstance(boxes,list) or not 1<=len(boxes)<=16:
        raise ValueError('Keep between 1 and 16 overview rectangles')
    for box in boxes:
        if not isinstance(box,list) or len(box)!=4 or any(type(v) is not int for v in box):
            raise ValueError('ROI coordinates must be four integers')
        x1,y1,x2,y2=box
        if not (0<=x1<x2<=640 and 0<=y1<y2<=480):raise ValueError('ROI rectangle is outside the image')
    return config


def load(path):return validate(json.loads(Path(path).read_text()))


def save(path,config):
    validate(config)
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n');temporary.replace(path)


def fingerprint(config):
    return hashlib.sha256(json.dumps(validate(config),sort_keys=True,separators=(',',':')).encode()).hexdigest()


def mask_rgb(rgb,name,config):
    import numpy as np
    validate(config)
    if rgb.shape!=(480,640,3):raise ValueError('ROI requires a 640x480 RGB image')
    if name=='wrist' and config['wrist']=='full':return rgb
    result=np.zeros_like(rgb)
    if name=='overview':
        for x1,y1,x2,y2 in config['overview_boxes']:result[y1:y2,x1:x2]=rgb[y1:y2,x1:x2]
    elif name!='wrist':raise ValueError('Unknown camera')
    return result


def mask_tensor(tensor,name,config):
    """Accept CHW or TCHW, float [0,1] or uint8; no image rescaling."""
    validate(config)
    if tuple(tensor.shape[-3:])!=(3,480,640):raise ValueError('ROI tensor camera shape mismatch')
    if name=='wrist' and config['wrist']=='full':return tensor
    result=tensor.new_zeros(tensor.shape)
    if name=='overview':
        for x1,y1,x2,y2 in config['overview_boxes']:result[...,y1:y2,x1:x2]=tensor[...,y1:y2,x1:x2]
    elif name!='wrist':raise ValueError('Unknown camera')
    return result


class MaskedDataset:
    def __init__(self,dataset,config):
        self.dataset=dataset;self.roi=validate(config)

    def __len__(self):return len(self.dataset)

    def __getattr__(self,name):
        if name in ('dataset','roi'):raise AttributeError(name)
        return getattr(self.dataset,name)

    def __getitem__(self,index):
        item=dict(self.dataset[index])
        for name in ('wrist','overview'):
            key=f'observation.images.{name}'
            item[key]=mask_tensor(item[key],name,self.roi)
        return item

    def __getitems__(self,indices):
        # Do not delegate batched fetching to the unmasked underlying dataset.
        return [self[index] for index in indices]
