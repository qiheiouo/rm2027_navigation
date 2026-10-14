"""Insert only the read-only observer; preserve all original SDF bytes.

SDFormat/DART consumes the literal ignition:expressed_in extension name.
Generic XML namespace reserialization changes its executable meaning.
"""
import hashlib,html,json,pathlib
def insert(path,library,output):
    path=pathlib.Path(path);original=path.read_text();start=original.index('<model name="rm_sentry_2027">');end=original.index('</model>',start)
    plugin=f'<plugin filename="{html.escape(str(library),quote=True)}" name="BoundaryObserver"><output>{html.escape(str(output))}</output></plugin>'
    augmented=original[:end]+plugin+original[end:];assert augmented.replace(plugin,'',1)==original
    assert augmented.count('ignition:expressed_in="base_link"')==original.count('ignition:expressed_in="base_link"')==4
    assert 'ns0:expressed_in' not in augmented;path.write_text(augmented)
    return {'original_sha256':hashlib.sha256(original.encode()).hexdigest(),'augmented_sha256':hashlib.sha256(augmented.encode()).hexdigest(),'original_bytes_preserved':True,'friction_extension_count':4}
