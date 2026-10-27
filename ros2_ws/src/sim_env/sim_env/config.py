"""YAML configuration loading.

A config file may name a parent with ``extends: <path>``; the child is deep-merged on top of
it (mappings merge, everything else is replaced). Relative paths inside a file are resolved
against that file's directory when a key ends in ``_file`` or ``path``.
"""
import copy
import os

import yaml


def deep_merge(base, override):
    out = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def _resolve_paths(node, base_dir):
    if isinstance(node, dict):
        for key, value in node.items():
            if isinstance(value, str) and (key.endswith('_file') or key == 'path'):
                node[key] = resolve(value, base_dir)
            else:
                _resolve_paths(value, base_dir)
    elif isinstance(node, list):
        for item in node:
            _resolve_paths(item, base_dir)


def resolve(path, base_dir):
    """Expand ``~``/env vars and ``package://<pkg>/...``; make relative paths absolute."""
    path = os.path.expandvars(os.path.expanduser(path))
    if path.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        pkg, _, rest = path[len('package://'):].partition('/')
        return os.path.join(get_package_share_directory(pkg), rest)
    return path if os.path.isabs(path) else os.path.normpath(os.path.join(base_dir, path))


def load(path, _seen=None):
    path = os.path.abspath(path)
    seen = _seen or set()
    if path in seen:
        raise ValueError(f'config extends cycle at {path}')
    seen.add(path)
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    base_dir = os.path.dirname(path)
    parent = data.pop('extends', None)
    _resolve_paths(data, base_dir)
    if parent:
        data = deep_merge(load(resolve(parent, base_dir), seen), data)
    return data


def get(cfg, dotted, default=None):
    node = cfg
    for part in dotted.split('.'):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def require(cfg, dotted):
    value = get(cfg, dotted)
    if value is None:
        raise KeyError(f'missing required config key: {dotted}')
    return value


def cache_dir(*parts):
    """Single per-user directory for generated artifacts (worlds, heightmaps, logs)."""
    root = os.environ.get('SIM_ENV_CACHE') or os.path.join(
        os.environ.get('XDG_CACHE_HOME', os.path.expanduser('~/.cache')), 'sim_env')
    path = os.path.join(root, *parts)
    os.makedirs(path, exist_ok=True)
    return path
