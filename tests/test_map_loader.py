"""Test map loading, runtime generation, and coordinate conversion."""
from pathlib import Path
import random
import numpy as np
import pytest

try:
    from airmouse.tools import generate_maps
except ImportError:
    import generate_maps
from airmouse.core.mapping.map_io import load_map_file, cell_to_world, world_to_cell

SAMPLE_MAP = Path(__file__).resolve().parent.parent / "maps" / "house_00042.json"
GENERATED_DIR = Path(__file__).resolve().parent.parent / "generated_maps"
GENERATED_MAP = GENERATED_DIR / "house_00042.json"


@pytest.fixture(scope="module", autouse=True)
def ensure_runtime_generated_map():
    GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    rng = random.Random(42)
    grid = generate_maps.gen_house(28, 20, rng)
    start = generate_maps.pick_start(grid, (2, 2))
    generate_maps.add_obstacles(grid, rng, 5, avoid=start)
    people = generate_maps.place_people(grid, start, 6, rng)
    generate_maps.export_map(grid, start, people, 42, "house", 1.0, GENERATED_DIR, "house_00042")
    return GENERATED_MAP


def test_load_generated_map(ensure_runtime_generated_map):
    map_file = ensure_runtime_generated_map
    assert map_file.exists(), f"Generated map file does not exist: {map_file}"
    grid, meta = load_map_file(map_file)

    assert grid.shape == (20, 28)
    assert meta.rows == 20
    assert meta.cols == 28
    assert meta.cell_size_m == 1.0
    assert meta.start_cell == (2, 2)
    assert len(meta.true_survivors) == 6

    # Test coordinate bijection
    wx, wy = cell_to_world(meta.start_cell[0], meta.start_cell[1], meta.rows, meta.cell_size_m)
    cx, cy = world_to_cell(wx, wy, meta.rows, meta.cell_size_m)
    assert (cx, cy) == meta.start_cell


def test_load_sample_map():
    assert SAMPLE_MAP.exists(), f"Sample map file does not exist: {SAMPLE_MAP}"
    grid, meta = load_map_file(SAMPLE_MAP)
    assert grid.shape == (20, 28)
    assert meta.start_cell == (2, 2)


