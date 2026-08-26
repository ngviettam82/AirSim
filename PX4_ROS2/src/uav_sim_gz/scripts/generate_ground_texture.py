#!/usr/bin/env python3
"""Generate ground textures at several difficulty levels; README 3b."""

import argparse
import os

import numpy as np
from PIL import Image

TILE_METRES = 2.0  # one texture tile covers this much ground


def fractal_noise(size, octaves, rng):
    """Sum octaves of upsampled white noise: detail at several scales."""
    field = np.zeros((size, size), dtype=np.float64)
    amplitude = 1.0
    total = 0.0
    for octave in range(octaves):
        cells = 2 ** (octave + 2)
        coarse = rng.random((cells, cells))
        upsampled = Image.fromarray((coarse * 255).astype(np.uint8)).resize(
            (size, size), Image.BICUBIC)
        # asarray without dtype: PIL's __array__ rejects numpy 2's copy keyword.
        layer = np.asarray(upsampled).astype(np.float64) / 255.0
        field += amplitude * layer
        total += amplitude
        amplitude *= 0.5
    return field / total


def normalise(field):
    span = field.max() - field.min()
    return (field - field.min()) / span if span > 1e-9 else np.zeros_like(field)


def gravel(size, rng, contrast):
    """Broadband texture: strong gradients at every scale, no dominant direction."""
    base = normalise(fractal_noise(size, octaves=6, rng=rng))
    grain = rng.random((size, size))
    field = normalise(0.75 * base + 0.25 * grain)

    mid = 0.5
    field = np.clip(mid + (field - mid) * contrast, 0.0, 1.0)

    rgb = np.stack([
        field * 0.62 + 0.16,
        field * 0.60 + 0.16,
        field * 0.56 + 0.15,
    ], axis=-1)
    return (rgb * 255).astype(np.uint8)


def scatter_landmarks(rgb, rng, count):
    """Distinct blobs so corner detectors have unambiguous features to track."""
    size = rgb.shape[0]
    yy, xx = np.mgrid[0:size, 0:size]
    for _ in range(count):
        cx, cy = rng.integers(0, size, size=2)
        radius = rng.integers(size // 60, size // 22)
        mask = (xx - cx) ** 2 + (yy - cy) ** 2 < radius ** 2
        shade = rng.integers(30, 90) if rng.random() < 0.5 else rng.integers(170, 235)
        rgb[mask] = shade
    return rgb


def build(name, size, contrast, landmarks, seed):
    rng = np.random.default_rng(seed)
    rgb = gravel(size, rng, contrast)
    if landmarks:
        rgb = scatter_landmarks(rgb, rng, landmarks)
    return Image.fromarray(rgb, mode='RGB')


# Three levels so flow degradation on poor texture is measurable.
VARIANTS = {
    'rich': dict(contrast=1.35, landmarks=45, seed=1),
    'sparse': dict(contrast=0.55, landmarks=8, seed=2),
    'flat': dict(contrast=0.04, landmarks=0, seed=3),
}


def write_ground_mesh(mesh_dir, name, extent):
    """Repeating-UV quad, not <plane>; see README section 3b."""
    tiles = extent / TILE_METRES
    half = extent / 2.0

    obj_path = os.path.join(mesh_dir, f'ground_{name}.obj')
    with open(obj_path, 'w', encoding='utf-8') as obj:
        obj.write(f'mtllib ground_{name}.mtl\n')
        obj.write(f'v {-half} {-half} 0\nv {half} {-half} 0\n')
        obj.write(f'v {half} {half} 0\nv {-half} {half} 0\n')
        obj.write(f'vt 0 0\nvt {tiles} 0\nvt {tiles} {tiles}\nvt 0 {tiles}\n')
        obj.write('vn 0 0 1\n')
        obj.write(f'usemtl ground_{name}\n')
        obj.write('f 1/1/1 2/2/1 3/3/1 4/4/1\n')

    mtl_path = os.path.join(mesh_dir, f'ground_{name}.mtl')
    with open(mtl_path, 'w', encoding='utf-8') as mtl:
        mtl.write(f'newmtl ground_{name}\n')
        mtl.write('Ka 1.000 1.000 1.000\nKd 1.000 1.000 1.000\nKs 0.000 0.000 0.000\n')
        mtl.write('illum 1\nd 1.0\n')
        mtl.write(f'map_Kd ../textures/ground_{name}.png\n')
    return obj_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, help='model materials directory')
    parser.add_argument('--size', type=int, default=1024, help='pixels per tile')
    parser.add_argument('--extent', type=float, default=100.0, help='ground side in metres')
    args = parser.parse_args()

    texture_dir = os.path.join(args.out, 'textures')
    mesh_dir = os.path.join(args.out, 'meshes')
    os.makedirs(texture_dir, exist_ok=True)
    os.makedirs(mesh_dir, exist_ok=True)

    for name, spec in VARIANTS.items():
        image = build(name, args.size, spec['contrast'], spec['landmarks'], spec['seed'])
        image.save(os.path.join(texture_dir, f'ground_{name}.png'), optimize=True)
        write_ground_mesh(mesh_dir, name, args.extent)

        pixels = np.asarray(image.convert('L')).astype(np.float64)
        gy, gx = np.gradient(pixels)
        strength = float(np.sqrt(gx ** 2 + gy ** 2).mean())
        print(f'{name:8s} {args.size}px/tile  '
              f'{TILE_METRES * 1000 / args.size:.2f} mm/pixel  '
              f'mean gradient {strength:6.2f}')


if __name__ == '__main__':
    main()
