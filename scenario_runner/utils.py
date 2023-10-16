import glob
import os
import sys

from typing import Optional

PACKAGE_EXT = 'egg'

def add_carla_path(path: Optional[str] = None) -> None:
    if path and path in sys.path:
        return

    bin_version = 'win-amd64' if os.name == 'nt' else 'linux-x86_64'
    uri = f'{path}/carla-*{sys.version_info.major}.{sys.version_info.minor}-{bin_version}.{PACKAGE_EXT}'

    if path is None:
        uri = [p for p in sys.path if 'PythonAPI\\carla\\dist' in p][0]
            
    matched_carla_api = glob.glob(uri)

    if (len(matched_carla_api) == 0):
        carla_apis = glob.glob(f'{path}/carla-*-{bin_version}.{PACKAGE_EXT}')
        carla_apis = '\n  '.join(carla_apis)
        raise RuntimeError(f'you are running the script in Python {sys.version_info.major}.{sys.version_info.minor}, but these CARLA APIs support different Python versions:\n    {carla_apis}')
    
    sys.path.append(matched_carla_api[0])
