"""Test map loading and coordinate conversion."""
from pathlib import Path
import numpy as np
import pytest
from airmouse.core.mapping.map_io import load_map_file, cell_to_world, world_to_cell

MAP_FILE = Path(__file__).resolve().parent.parent / "maps" / "house_00042.json"

def test_load_map():
    assert MAP_FILE.exists(), f"Map file does not exist: {MAP_FILE}"
    grid, meta = load_map_file(MAP_FILE)

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

