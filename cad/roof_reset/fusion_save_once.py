"""One authorized new-file cloud save after a verified local F3D roundtrip.

Run in Fusion with the existing presentation namespace named ``gtroofreset``.
The persistent result file prevents repeated saveAs attempts.
"""
import json
import traceback
from pathlib import Path

import adsk.core


ns = gtroofreset
result_path = ns['OUT'] / 'fusion_cloud_save_result.json'
if result_path.exists():
    raise RuntimeError('Cloud-save result already exists; no repeated attempt is permitted.')
state = {'phase': 'preflight', 'save_attempted': False,
         'target_name': ns['NAME'], 'source_id': ns['SOURCE_ID']}
app = adsk.core.Application.get()
document = None
baseline = json.loads(ns['SOURCE_RECORD'].read_text(encoding='utf-8'))


def publish():
    result_path.write_text(json.dumps(state, indent=2), encoding='utf-8')


try:
    record = json.loads(ns['RESULT'].read_text(encoding='utf-8'))
    if record['phase'] != 'local_archive_verified':
        raise RuntimeError('A verified local archive is required before cloud save.')
    state['run_id'] = record['run_id']
    state['local_archive'] = record['local_archive']
    if ns['_sha256'](Path(record['local_archive'])) != record['archive_sha256']:
        raise RuntimeError('Verified local archive changed.')
    if ns['_sha256'](ns['MANIFEST']) != record['manifest_sha256']:
        raise RuntimeError('Concept manifest changed.')
    if ns['_sha256'](ns['REFERENCE_STEP']) != record['reference_step_sha256']:
        raise RuntimeError('Neutral reference STEP changed.')
    for part in record['parts']:
        if ns['_sha256'](Path(part['step'])) != part['sha256']:
            raise RuntimeError('Concept STEP changed: ' + part['name'])
    source = ns['_source_guard'](app, baseline)
    document = ns['_marked_document'](app, record)
    if document.isSaved:
        raise RuntimeError('Study is already saved; no new-file save attempted.')
    try:
        existing_data = document.dataFile
    except RuntimeError:
        existing_data = None
    if existing_data:
        raise RuntimeError('Study already has a DataFile; no new-file save attempted.')
    ns['_verify_study'](ns['_design'](document), record)
    folder = source.dataFile.parentFolder
    files = folder.dataFiles
    names = [files.item(i).name for i in range(files.count)]
    if ns['NAME'].casefold() in [name.casefold() for name in names]:
        raise RuntimeError('Target name already exists in source folder; refusing overwrite.')
    state['folder_name'] = folder.name
    state['folder_id'] = folder.id
    state['existing_name_collision'] = False
    state['save_attempted'] = True
    state['phase'] = 'single_saveAs_attempt_started'
    publish()
    # This is the only saveAs call. A failed call is never retried.
    state['saveAs_returned'] = document.saveAs(
        ns['NAME'], folder,
        'Drive-front roof exterior study with independent neutral STEP chassis reference. '
        'Original Mower V3 unchanged. Local F3D roundtrip verified.', '')
    state['phase'] = ('cloud_save_submitted' if state['saveAs_returned']
                      else 'cloud_save_failed_once')
    try:
        saved = document.dataFile
        if saved:
            state['data_file'] = {'id': saved.id, 'name': saved.name,
                                  'version': saved.versionNumber}
            if saved.id == ns['SOURCE_ID'] or saved.name != ns['NAME']:
                raise RuntimeError('Unexpected saved document identity.')
            state['phase'] = 'cloud_save_verified'
    except RuntimeError as exc:
        state['data_file_lookup_error'] = str(exc)
    ns['_verify_study'](ns['_design'](document), record)
except Exception:
    state['phase'] = ('cloud_save_failed_once' if state['save_attempted']
                      else 'cloud_save_preflight_failed')
    state['error'] = traceback.format_exc()
finally:
    try:
        ns['_source_guard'](app, baseline)
        state['source_v3_modified'] = False
        state['source_geometry_unchanged'] = True
    except Exception:
        state['source_guard_error'] = traceback.format_exc()
    if document:
        try:
            document.activate()
            state['active_document'] = document.name
            image_path = ns['OUT'] / 'fusion_viewport_final.png'
            state['viewport_saved'] = app.activeViewport.saveAsImageFile(
                str(image_path), 1400, 1000)
            state['viewport_image'] = str(image_path)
        except Exception:
            state['viewport_error'] = traceback.format_exc()
    publish()
