import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sar_drone_sim'


def collect_model_data_files():
    """Collect model files preserving models/<model_name>/ subdirectory layout."""
    data_files = []
    models_root = 'models'
    if os.path.isdir(models_root):
        for dirpath, _, filenames in os.walk(models_root):
            if filenames:
                files = [os.path.join(dirpath, f) for f in filenames]
                install_dir = os.path.join('share', package_name, dirpath)
                data_files.append((install_dir, files))
    return data_files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ] + collect_model_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gomezluis2907',
    maintainer_email='gomezluis2907@local.dev',
    description='Search-and-rescue drone simulation in an earthquake disaster world with thermal CV.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'thermal_detector = sar_drone_sim.thermal_detector:main',
            'keyboard_teleop = sar_drone_sim.keyboard_teleop:main',
        ],
    },
)
