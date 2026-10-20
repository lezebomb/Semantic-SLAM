#!/usr/bin/env python3
"""Single-architecture stock Ultralytics GPU training with persistent checkpoints."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import time
import torch
import ultralytics
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent


def atomic_copy(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + '.tmp')
    shutil.copyfile(source, temporary)
    temporary.replace(destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--persist', type=Path, required=True)
    parser.add_argument('--resume', type=Path)
    parser.add_argument('--epochs', type=int, default=20)
    parser.add_argument('--batch', type=int, default=8)
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--gpu-hours', type=float, default=3.0)
    args = parser.parse_args()
    if not torch.cuda.is_available():
        raise SystemExit('No usable CUDA GPU: refusing formal CPU training.')
    assert ultralytics.__version__ == '8.3.221', ultralytics.__version__
    assert (ROOT / 'data/dataset/images_complete.json').exists(), 'Run prepare_data.py first'
    args.persist.mkdir(parents=True, exist_ok=True)
    previous_progress = args.persist / 'progress.json'
    previous = json.loads(previous_progress.read_text()) if args.resume and previous_progress.exists() else {}
    prior_seconds = previous.get('total_elapsed_seconds', previous.get('elapsed_seconds', 0.0))
    if prior_seconds >= args.gpu_hours * 3600:
        raise SystemExit('The saved training budget is exhausted; existing Drive checkpoints are preserved.')
    device = torch.cuda.get_device_properties(0)
    with torch.cuda.device(0):
        probe = torch.randn(16, 16, device='cuda')
        assert torch.isfinite(probe @ probe).all(), 'Actual CUDA kernel check failed'
    path = args.resume or ROOT / 'weights/initialized_untrained.pt'
    model = YOLO(str(path), task='segment', verbose=False)
    expected_state = {k: v.detach().cpu().clone() for k, v in model.model.state_dict().items()}
    started, epoch_times, previous_epoch_end = time.monotonic(), [], [None]
    initial_projection = [None]
    evidence = dict(gpu=device.name, gpu_memory_gib=device.total_memory / 2**30,
                    cuda_capability=list(torch.cuda.get_device_capability()), torch=torch.__version__,
                    torchvision=__import__('torchvision').__version__, ultralytics=ultralytics.__version__,
                    epochs_requested=args.epochs, batch=args.batch, imgsz=640,
                    optimizer='AdamW', lr0=0.001, amp=True, workers=args.workers, seed=20261015,
                    gpu_training_budget_hours=args.gpu_hours, resume_source=str(args.resume) if args.resume else None,
                    prior_training_seconds=prior_seconds,
                    formal_training_completed=False)
    evidence.update(python=sys.version,
        training_data_sizes=json.loads((ROOT / 'data/dataset/manifest.json').read_text())['sizes'],
        source_sha256={name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
            for name in ['train.py', 'yolov8n-mobilenetv3-small-seg.yaml', 'data/dataset/manifest.json']},
        initial_checkpoint_sha256=hashlib.sha256(path.read_bytes()).hexdigest())

    def ready(trainer):
        current = trainer.model.state_dict()
        assert all(torch.equal(v, current[k].detach().cpu()) for k, v in expected_state.items()), 'Initialization was overwritten'
        initial_projection[0] = trainer.model.model[2].conv.weight.detach().cpu().clone()
        expected_state.clear()
        evidence['initialization_preserved_in_actual_trainer'] = True
        previous_epoch_end[0] = time.monotonic()

    def epoch_end(trainer):
        now = time.monotonic()
        epoch_times.append(now - previous_epoch_end[0])
        previous_epoch_end[0] = now
        elapsed = prior_seconds + now - started
        # Keep a hard epoch cap, unlike time= which can dynamically increase epochs.
        if elapsed + max(epoch_times[-2:]) > args.gpu_hours * 3600:
            trainer.stop = True
            evidence['budget_stop_requested'] = True

    def save(trainer):
        if trainer.last.exists():
            atomic_copy(trainer.last, args.persist / 'weights/resume.pt')
        if trainer.best.exists():
            atomic_copy(trainer.best, args.persist / 'weights/best.pt')
        for name in ['results.csv', 'args.yaml']:
            source = trainer.save_dir / name
            if source.exists():
                atomic_copy(source, args.persist / 'training' / name)
        progress = dict(epoch=trainer.epoch + 1, optimizer_updates=trainer.ema.updates,
                        elapsed_seconds=time.monotonic() - started,
                        total_elapsed_seconds=prior_seconds + time.monotonic() - started,
                        current_metrics={k:float(v) for k, v in trainer.metrics.items()})
        (args.persist / 'progress.json').write_text(json.dumps(progress, indent=2))

    model.add_callback('on_pretrain_routine_end', ready)
    model.add_callback('on_train_epoch_end', epoch_end)
    model.add_callback('on_model_save', save)
    kwargs = dict(data=str(ROOT / 'data/coco_subset.yaml'), device=0, amp=True, workers=args.workers,
                  cache=False, project=str(ROOT / 'training'), name='mobilenet', exist_ok=False,
                  optimizer='AdamW', lr0=0.001, lrf=0.01, weight_decay=0.0005,
                  epochs=args.epochs, batch=args.batch, imgsz=640, patience=8,
                  close_mosaic=5, seed=20261015, deterministic=True, freeze=0,
                  save=True, save_period=-1, plots=True, val=True)
    if args.resume:
        kwargs['resume'] = True
    try:
        model.train(**kwargs)
        trainer = model.trainer
        update = float((trainer.model.model[2].conv.weight.detach().cpu() - initial_projection[0]).abs().max())
        assert trainer.ema.updates > 0 and update > 0, 'No gradient updates demonstrated'
        for source, name in [(trainer.best, 'best.pt'), (trainer.last, 'last.pt')]:
            atomic_copy(source, ROOT / 'weights' / name)
            atomic_copy(source, args.persist / 'weights' / name)
        evidence.update(formal_training_completed=True, epochs_actual=trainer.epoch + 1,
                        optimizer_updates=trainer.ema.updates, projection_max_weight_change=update,
                        wall_seconds=prior_seconds + time.monotonic() - started,
                        session_wall_seconds=time.monotonic() - started,
                        best_sha256=hashlib.sha256((ROOT / 'weights/best.pt').read_bytes()).hexdigest(),
                        training_save_dir=str(trainer.save_dir),
                        final_selection_metrics={k:float(v) for k, v in trainer.metrics.items()})
    except BaseException as error:
        saved_progress = args.persist / 'progress.json'
        saved = json.loads(saved_progress.read_text()) if saved_progress.exists() else {}
        evidence.update(error=f'{type(error).__name__}: {error}',
                        wall_seconds=prior_seconds + time.monotonic() - started,
                        epochs_saved=saved.get('epoch', 0), optimizer_updates=saved.get('optimizer_updates', 0))
        raise
    finally:
        text = json.dumps(evidence, indent=2) + '\n'
        (ROOT / 'results/training_summary.json').write_text(text)
        (args.persist / 'training_summary.json').write_text(text)
    print(json.dumps(evidence, indent=2))


if __name__ == '__main__':
    main()
