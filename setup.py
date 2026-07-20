import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'sim'


def data_files_for_dir(src_dir, dest_prefix):
    """Recursively map every file under src_dir to share/<package>/<dest_prefix>/<relpath>,
    preserving subdirectory structure (needed for STL meshes in nested folders)."""
    entries = []
    for root, _dirs, files in os.walk(src_dir):
        if not files:
            continue
        rel = os.path.relpath(root, src_dir)
        dest = os.path.join('share', package_name, dest_prefix, rel) if rel != '.' \
            else os.path.join('share', package_name, dest_prefix)
        entries.append((dest, [os.path.join(root, f) for f in files]))
    return entries


data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
]
data_files += data_files_for_dir('urdf', 'urdf')
data_files += data_files_for_dir('worlds', 'worlds')
data_files += data_files_for_dir('meshes', 'meshes')
data_files += data_files_for_dir('world', 'world')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Launches gz sim with the ARCC_Field_2026 world and spawns the sentry robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wasd_teleop = sim.wasd_teleop:main',
            'auto_explore = sim.auto_explore:main',
            'head_sweep = sim.head_sweep:main',
        ],
    },
)
