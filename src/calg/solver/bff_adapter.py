from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shutil
import subprocess
import tempfile

from calg.modeling.mesh import TriangleMesh


@dataclass
class BFFResult:
    available: bool
    success: bool
    command: list[str]
    stdout: str
    stderr: str
    output_path: Path | None
    message: str


class BFFAdapter:
    """Optional adapter for the official BFF command-line executable.

    The CALG core solver never imports or depends on BFF.  This adapter is only
    used for optional local patch conditioning or reference parameterization
    experiments.  It is deliberately skip-safe: if the executable is unavailable,
    callers receive a structured result rather than an exception.
    """

    def __init__(self, executable: str | None = None):
        self.executable = executable or self._find_default_executable()

    @staticmethod
    def _find_default_executable() -> str | None:
        candidates = [
            "bff-command-line",
            "bff-command-line.exe",
            "third_party/bff_official/binaries/windows-v1.6/bff-command-line.exe",
            "third_party/bff_official/binaries/osx-v1.6/bff.app/Contents/MacOS/bff-command-line",
        ]
        for c in candidates:
            p = shutil.which(c) or (str(Path(c)) if Path(c).exists() else None)
            if p:
                return p
        return None

    def available(self) -> bool:
        return self.executable is not None and Path(self.executable).exists()

    def flatten_obj(self, input_obj: str | Path, output_obj: str | Path | None = None, timeout: float = 60.0) -> BFFResult:
        if not self.available():
            return BFFResult(False, False, [], "", "", None, "BFF executable not available; optional step skipped")
        input_obj = Path(input_obj)
        if output_obj is None:
            output_obj = input_obj.with_name(input_obj.stem + "_bff.obj")
        output_obj = Path(output_obj)
        # The official command-line interface writes output according to its own options.
        # Different BFF releases use slightly different flags, so we first try the common
        # minimal form.  The result is captured for reproducibility.
        cmd = [str(self.executable), str(input_obj), str(output_obj)]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
            return BFFResult(True, proc.returncode == 0, cmd, proc.stdout, proc.stderr, output_obj if output_obj.exists() else None, "completed" if proc.returncode == 0 else "BFF returned nonzero status")
        except Exception as e:  # pragma: no cover - executable-dependent
            return BFFResult(True, False, cmd, "", str(e), None, f"BFF execution failed: {e}")

    def flatten_mesh(self, mesh: TriangleMesh, work_dir: str | Path | None = None) -> BFFResult:
        if work_dir is None:
            with tempfile.TemporaryDirectory() as td:
                return self._flatten_mesh_in_dir(mesh, Path(td))
        return self._flatten_mesh_in_dir(mesh, Path(work_dir))

    def _flatten_mesh_in_dir(self, mesh: TriangleMesh, work_dir: Path) -> BFFResult:
        work_dir.mkdir(parents=True, exist_ok=True)
        input_obj = work_dir / f"{mesh.name}_input.obj"
        output_obj = work_dir / f"{mesh.name}_bff.obj"
        mesh.write_obj(input_obj)
        return self.flatten_obj(input_obj, output_obj)
