"""Verify the bundled pinned PX4 wire definitions before building."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1]
manifest=json.loads((root/'src/px4_msgs/source_manifest.json').read_text())
lock=json.loads((root/'dependencies/lock.json').read_text())
assert manifest['profile']=='real' and manifest['commit']==lock['px4_real_msgs']['commit']
for name,expected in manifest['messages'].items():
 actual=hashlib.sha256((root/'src/px4_msgs/msg'/f'{name}.msg').read_bytes()).hexdigest()
 if actual!=expected:raise SystemExit('Modified PX4 message: '+name)
print('Pinned PX4 message snapshot verified.')
