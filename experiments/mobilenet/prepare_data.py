#!/usr/bin/env python3
"""Real COCO polygon subset. No full COCO image archive or invented labels."""
import argparse
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor
import gc
import hashlib
import json
from pathlib import Path
import random
import shutil
import os
import threading
import urllib.request
import zipfile
import requests
import yaml
from ultralytics.data.converter import convert_coco

ROOT = Path(__file__).resolve().parent
DATA = ROOT / 'data/dataset'
BUCKET = 'https://s3.amazonaws.com/images.cocodataset.org'
HTTP = threading.local()


def fetch(url, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_file() and destination.stat().st_size:
        return
    temporary = destination.with_suffix(destination.suffix + f'.part.{os.getpid()}')
    for attempt in range(3):
        try:
            if not hasattr(HTTP, 'session'):
                HTTP.session = requests.Session()
                HTTP.session.headers['User-Agent'] = 'mobilenet-coco-subset/1.0'
            with HTTP.session.get(url, timeout=(15, 90), stream=True) as src, temporary.open('wb') as dst:
                src.raise_for_status()
                for chunk in src.iter_content(1024 * 1024):
                    dst.write(chunk)
            if temporary.stat().st_size == 0:
                raise RuntimeError('Empty download')
            temporary.replace(destination)
            return
        except Exception:
            temporary.unlink(missing_ok=True)
            if attempt == 2:
                raise


def select_ids(ids, categories, count, seed, minimum_per_class):
    if len(ids) < count:
        raise ValueError(f'Not enough annotated images for {count}')
    rng = random.Random(seed)
    selected = set()
    for category in sorted(categories):
        pool = sorted(ids.intersection(categories[category]))
        rng.shuffle(pool)
        selected.update(pool[:minimum_per_class])
    if len(selected) > count:
        raise RuntimeError('Coverage quota exceeds subset budget')
    remaining = sorted(ids - selected)
    rng.shuffle(remaining)
    selected.update(remaining[:count - len(selected)])
    return selected


def valid_polygon(annotation):
    segments = annotation.get('segmentation')
    return (not annotation.get('iscrowd', 0) and isinstance(segments, list) and segments
            and all(isinstance(p, list) and len(p) >= 6 and len(p) % 2 == 0 for p in segments)
            and annotation['bbox'][2] > 0 and annotation['bbox'][3] > 0)


def prepare_metadata(train_size=8000, val_size=1000, test_size=1000):
    manifest_path = DATA / 'manifest.json'
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        assert manifest['sizes'] == dict(train=train_size, val=val_size, test=test_size), 'Existing subset sizes differ'
        write_yaml(manifest['names'])
        return manifest
    source = ROOT / 'data/source'
    source.mkdir(parents=True, exist_ok=True)
    archive = source / 'annotations_trainval2017.zip'
    print('Fetching official annotations only (~241 MiB); no complete image archive.', flush=True)
    fetch(BUCKET + '/annotations/annotations_trainval2017.zip', archive)
    digest = hashlib.sha256()
    with archive.open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(chunk)
    selected_dir = source / 'selected'
    selected_dir.mkdir(exist_ok=True)
    records, distributions = {}, {}
    names, original_categories = None, None
    with zipfile.ZipFile(archive) as zipped:
        for official_split in ['train2017', 'val2017']:
            print('Reading', official_split, flush=True)
            with zipped.open(f'annotations/instances_{official_split}.json') as stream:
                coco = json.load(stream)
            category_list = sorted(coco['categories'], key=lambda c: c['id'])
            assert len(category_list) == 80
            names = {i: c['name'] for i, c in enumerate(category_list)}
            original_categories = {c['id']: i for i, c in enumerate(category_list)}
            eligible, by_class = set(), defaultdict(set)
            for ann in coco['annotations']:
                if valid_polygon(ann):
                    eligible.add(ann['image_id'])
                    by_class[ann['category_id']].add(ann['image_id'])
            if official_split == 'train2017':
                groups = {'train': select_ids(eligible, by_class, train_size, 20261015, 64)}
            else:
                validation = select_ids(eligible, by_class, val_size, 20261016, 8)
                testing = select_ids(eligible - validation, by_class, test_size, 20261017, 8)
                assert not validation.intersection(testing)
                groups = {'val': validation, 'test': testing}
            union = set().union(*groups.values())
            images = {im['id']: im for im in coco['images'] if im['id'] in union}
            chosen = [ann for ann in coco['annotations'] if ann['image_id'] in union and valid_polygon(ann)]
            annotations = defaultdict(list)
            for ann in chosen:
                annotations[ann['image_id']].append(ann)
            for split, image_ids in groups.items():
                records[split] = [dict(id=i, file=images[i]['file_name'], official_split=official_split,
                    url=f'{BUCKET}/{official_split}/{images[i]["file_name"]}',
                    category_ids=sorted({a['category_id'] for a in annotations[i]})) for i in sorted(image_ids)]
                count = Counter(a['category_id'] for i in image_ids for a in annotations[i])
                distributions[split] = {names[idx]: count[cat] for cat, idx in original_categories.items()}
            with (selected_dir / f'instances_{official_split}.json').open('w') as stream:
                json.dump(dict(images=list(images.values()), annotations=chosen,
                               categories=category_list), stream)
            del coco, images, annotations, chosen, eligible, by_class
            gc.collect()
    assert not {r['id'] for r in records['train']}.intersection({r['id'] for r in records['val'] + records['test']})
    converted = ROOT / 'data/converted'
    assert not converted.exists(), 'Remove only an incomplete new conversion before retrying.'
    convert_coco(labels_dir=str(selected_dir), save_dir=str(converted), use_segments=True, cls91to80=True)
    DATA.mkdir(parents=True, exist_ok=True)
    (DATA / 'labels').mkdir(exist_ok=True)
    for official_split in ['train2017', 'val2017']:
        shutil.move(str(converted / 'labels' / official_split), str(DATA / 'labels' / official_split))
    for split, rows in records.items():
        (DATA / f'{split}.txt').write_text(''.join(f'./images/{r["official_split"]}/{r["file"]}\n' for r in rows))
        for row in rows:
            label = DATA / 'labels' / row['official_split'] / Path(row['file']).with_suffix('.txt')
            assert label.exists() and label.stat().st_size
            for line in label.read_text().splitlines():
                parts = line.split()
                assert 0 <= int(parts[0]) < 80 and len(parts) >= 7 and len(parts) % 2 == 1
    manifest = dict(seed=20261015, sizes=dict(train=train_size, val=val_size, test=test_size),
                    source='official COCO 2017 instance annotations', annotation_sha256=digest.hexdigest(),
                    annotation_url=BUCKET + '/annotations/annotations_trainval2017.zip',
                    names=names, original_category_to_yolo=original_categories,
                    records=records, class_instance_counts=distributions,
                    conversion='Ultralytics 8.3.221 convert_coco(use_segments=True); noncrowd polygons only',
                    limitation='Class-covered subset, not full COCO distribution; multi-polygons joined by stock converter.')
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    write_yaml(names)
    print('Prepared disjoint real-mask subsets:', manifest['sizes'], flush=True)
    return manifest


def write_yaml(names):
    names = {int(k): value for k, value in names.items()}
    config = dict(path=str(DATA.resolve()), train='train.txt', val='val.txt', test='test.txt', names=names)
    (ROOT / 'data/coco_subset.yaml').write_text(yaml.safe_dump(config, sort_keys=False))


def download_images(manifest, smoke_images=0):
    rows = manifest['records']['train'][:smoke_images] if smoke_images else [
        row for split in ['train', 'val', 'test'] for row in manifest['records'][split]]
    def download(row):
        destination = DATA / 'images' / row['official_split'] / row['file']
        fetch(row['url'], destination)
        from PIL import Image
        with Image.open(destination) as image:
            image.verify()
        return destination
    with ThreadPoolExecutor(max_workers=32) as pool:
        for index, _ in enumerate(pool.map(download, rows), 1):
            if index % 100 == 0 or index == len(rows):
                print(f'Downloaded and verified {index}/{len(rows)} selected images', flush=True)
    if not smoke_images:
        (DATA / 'images_complete.json').write_text(json.dumps(dict(images=len(rows), verified=True)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metadata-only', action='store_true')
    parser.add_argument('--smoke-images', type=int, default=0)
    args = parser.parse_args()
    manifest = prepare_metadata()
    if not args.metadata_only or args.smoke_images:
        download_images(manifest, args.smoke_images)
