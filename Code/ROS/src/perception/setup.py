from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception'


def collect_worlds():
    """Recursively install everything under worlds/ (SDF + model meshes)."""
    result = []
    for root, dirs, files in os.walk('worlds'):
        install_dir = os.path.join('share', package_name, root)
        result.append((install_dir, [os.path.join(root, f) for f in files]))
    return result


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ] + collect_worlds(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='rh08461@st.habib.edu.pk',
    description='Point cloud filtering, elevation mapping, and APF planning pipeline',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'ceiling_filter  = perception.ceiling_filter_node:main',
            'ceiling_flipper = perception.ceiling_flipper_node:main',
            'map_flipper     = perception.map_flipper_node:main',
            'map2apf         = perception.map2apf_node:main',
        ],
    },
)
