"""基础场景与可复现森林。"""
import importlib.util
from pathlib import Path
def world_source(root, name, density="medium", seed=17):
    root = Path(root)
    if name == "benchmark":
        spec = importlib.util.spec_from_file_location("forest_generator", root / "scripts/generate/prepare_benchmark_world.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.prepare(density, seed)
    if name not in ("test", "forest", "forest_long"):
        raise ValueError("world must be test, forest, forest_long or benchmark")
    return root / "simulation/worlds" / (name + ".sdf")
