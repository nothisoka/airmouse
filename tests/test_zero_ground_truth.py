"""Unit test verifying that core/ has zero illegal imports.
Enforces:
1. No 'pygame' imports anywhere inside core/
2. No 'rclpy' or ROS imports anywhere inside core/
3. No imports of 'backends' or simulator ground-truth models inside core/
"""
import ast
from pathlib import Path
import pytest

CORE_DIR = Path(__file__).resolve().parent.parent / "core"

FORBIDDEN_MODULES = {
    "pygame",
    "rclpy",
    "ros_gz_bridge",
    "ros_gz_sim",
    "airmouse.backends",
    "backends",
}

def check_file_imports(py_file: Path):
    tree = ast.parse(py_file.read_text(), filename=str(py_file))
    violations = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                for forbidden in FORBIDDEN_MODULES:
                    if alias.name == forbidden or alias.name.startswith(forbidden + "."):
                        violations.append((node.lineno, alias.name))
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                for forbidden in FORBIDDEN_MODULES:
                    if node.module == forbidden or node.module.startswith(forbidden + "."):
                        violations.append((node.lineno, node.module))
    return violations


def test_core_isolation():
    assert CORE_DIR.exists(), f"Core directory {CORE_DIR} does not exist"
    py_files = list(CORE_DIR.rglob("*.py"))
    assert len(py_files) > 0, "No python files found in core/"

    all_violations = {}
    for py_file in py_files:
        violations = check_file_imports(py_file)
        if violations:
            all_violations[str(py_file.relative_to(CORE_DIR.parent))] = violations

    assert not all_violations, f"Forbidden imports detected in core/:\n{all_violations}"

