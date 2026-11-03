"""LeRobot 0.6.1 adapter: ROI before normalizers, checkpoint-bound configuration."""
import json
import os
from pathlib import Path
from roi_mask import MaskedDataset,validate,save


def install_hooks(trainer,config):
    config=validate(config)
    original_factory=trainer.make_train_eval_datasets
    original_save=trainer.save_checkpoint
    def datasets(cfg):
        if cfg.dataset.streaming:raise ValueError('ROI supports non-streaming datasets only')
        if cfg.dataset.image_transforms.enable:
            raise ValueError('Disable random image transforms for fixed-pixel ROI training')
        train,eval_set=original_factory(cfg)
        return MaskedDataset(train,config),None if eval_set is None else MaskedDataset(eval_set,config)
    def checkpoint(*args,**kwargs):
        path=Path(kwargs['checkpoint_dir'] if 'checkpoint_dir' in kwargs else args[0])
        original_save(*args,**kwargs)
        save(path/'pretrained_model/roi_mask.json',config)
    trainer.make_train_eval_datasets=datasets
    trainer.save_checkpoint=checkpoint


def main():
    from importlib.metadata import version
    if version('lerobot')!='0.6.1':raise RuntimeError('ROI adapter requires LeRobot 0.6.1')
    config=validate(json.loads(os.environ['ARM3_TRAIN_ROI_JSON']))
    from lerobot.scripts import lerobot_train
    install_hooks(lerobot_train,config)
    print('ROI training: fixed overview boxes',config['overview_boxes'],'wrist:',config['wrist'],flush=True)
    lerobot_train.main()


if __name__=='__main__':main()
