"""Visibility-only test copy; no production statement or math is replaced."""
from pathlib import Path
import sys,hashlib,json
source=Path(sys.argv[1]);dest=Path(sys.argv[2]);s=source.read_text()
assert s.count('private:')==1
s=s.replace('private:','public: // test visibility only',1)
s=s.replace('#include "IMU_Processing.hpp"', '// IMU header already included with audit fixture visibility')
dest.write_text(s)
dest.with_suffix('.source.json').write_text(json.dumps({'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'change':'access label private->public and omit duplicate unguarded header include; all methods unchanged'},indent=2)+'\n')
