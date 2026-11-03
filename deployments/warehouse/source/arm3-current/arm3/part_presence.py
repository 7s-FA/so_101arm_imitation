"""Fixed-jig color/size occupancy check; no reference photos or pose templates.

Counts separated colored components contained in the destination jig. This is
an RGB presence check, not a depth/contact check or a general object detector.
"""
import cv2
import numpy as np


def rectangle(value):
    if (not isinstance(value, list) or len(value) != 4
            or any(type(v) is not int for v in value)):
        raise ValueError('Invalid presence rectangle')
    x1,y1,x2,y2=value
    if not (0 <= x1 < x2 <= 640 and 0 <= y1 < y2 <= 480):
        raise ValueError('Presence rectangle outside camera image')
    return value


class PartPresence:
    def __init__(self, config):
        self.jig=rectangle(config['jig_region'])
        self.clearance=rectangle(config['clearance_region'])
        x1,y1,x2,y2=self.clearance
        a,b,c,d=self.jig
        if not (x1 <= a < c <= x2 and y1 <= b < d <= y2):
            raise ValueError('Presence jig must be inside clearance region')
        self.rules=config['parts']
        if set(self.rules) != {'orange','blue'}:
            raise ValueError('Presence check requires orange and blue rules')
        for name,rule in self.rules.items():
            if rule['count'] != (2 if name == 'orange' else 1):
                raise ValueError('Expected two orange parts and one blue part')
            for key,limits in [('hsv_low',(179,255,255)),('hsv_high',(179,255,255))]:
                v=rule[key]
                if len(v)!=3 or any(type(n) is not int or not 0 <= n <= limit for n,limit in zip(v,limits)):
                    raise ValueError('Invalid presence HSV range')
            if any(a>b for a,b in zip(rule['hsv_low'],rule['hsv_high'])):
                raise ValueError('Reversed presence HSV range')
            for key in ('area','width','height','aspect'):
                lo,hi=rule[key]
                if not np.isfinite([lo,hi]).all() or not 0 < lo <= hi:
                    raise ValueError('Invalid presence size range')
            if not 0 < rule['min_fill'] <= 1:
                raise ValueError('Invalid presence fill threshold')
        self.inside=config['min_inside_fraction']
        self.max_extra_blue=config['max_extra_blue_pixels']
        if not .9 <= self.inside <= 1 or type(self.max_extra_blue) is not int or not 0 <= self.max_extra_blue <= 200:
            raise ValueError('Invalid presence clearance thresholds')

    def evaluate(self, rgb):
        if rgb.shape != (480,640,3) or rgb.dtype != np.uint8:
            raise ValueError('Completion requires overview RGB uint8 640x480')
        x1,y1,x2,y2=self.clearance
        a,b,c,d=self.jig
        hsv=cv2.cvtColor(rgb[y1:y2,x1:x2],cv2.COLOR_RGB2HSV)
        objects={};masks={};accepted_masks={}
        for name,rule in self.rules.items():
            mask=cv2.inRange(hsv,tuple(rule['hsv_low']),tuple(rule['hsv_high']))
            # Only remove isolated noise; never join two separate objects.
            mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
            _,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
            selected=[];accepted=np.zeros(mask.shape,dtype=bool)
            for label,(x,y,w,h,area) in enumerate(stats[1:],1):
                if not all(rule[key][0] <= val <= rule[key][1] for key,val in
                           [('area',area),('width',w),('height',h),('aspect',w/h)]):
                    continue
                if area/(w*h) < rule['min_fill']:continue
                pixels=labels==label
                inside=float(np.count_nonzero(pixels[b-y1:d-y1,a-x1:c-x1])/area)
                cx,cy=centers[label]+(x1,y1)
                if inside < self.inside or not (a <= cx < c and b <= cy < d):continue
                selected.append({'bbox':[int(x+x1),int(y+y1),int(x+x1+w),int(y+y1+h)],
                                 'area':int(area),'inside_fraction':inside})
                accepted|=pixels
            objects[name]=selected;masks[name]=mask;accepted_masks[name]=accepted
        # Exempt the detected cylinder itself, wherever it lies inside the jig.
        # Long/merged gripper shapes fail the size test and remain in this mask.
        extra_blue=(masks['blue']!=0)&~accepted_masks['blue']
        extra_count=int(np.count_nonzero(extra_blue))
        counts={name:len(items) for name,items in objects.items()}
        matches={name+'_parts':counts[name]==rule['count'] for name,rule in self.rules.items()}
        scores={name+'_parts':min(counts[name]/rule['count'],1.0) if counts[name]<=rule['count'] else 0.0
                for name,rule in self.rules.items()}
        clear=extra_count <= self.max_extra_blue
        return {'method':'color_presence','matches':matches,'presence_scores':scores,
                'object_counts':counts,'objects':objects,'extra_blue_pixels':extra_count,
                'gripper_clear':clear,'candidate':bool(all(matches.values()) and clear)}
