import os
from glob import glob
from setuptools import setup

package_name = 'fyp_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),

        # --- ROBOT MODELS INSTALLATION ---
        # Install fyp_burger main files (sdf, config)
        (os.path.join('share', package_name, 'models/fyp_burger'), 
            glob('models/fyp_burger/*.sdf') + glob('models/fyp_burger/*.config')),
        
        # Install fyp_burger URDF subfolder
        (os.path.join('share', package_name, 'models/fyp_burger/urdf'), 
            glob('models/fyp_burger/urdf/*.urdf')),

        # Install custom environment models (e.g., L-shaped corridor)
        (os.path.join('share', package_name, 'models/L-shaped_corridor'), 
            glob('models/L-shaped_corridor/*.sdf') + glob('models/L-shaped_corridor/*.config')),
        
        # If you have meshes for your corridor, include them here:
        # (os.path.join('share', package_name, 'models/L-shaped_corridor/meshes'), 
        #    glob('models/L-shaped_corridor/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ser',
    maintainer_email='ser@todo.todo',
    description='FYP Robot Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_pacer = fyp_bot.human_pacer:main',
            'truth_extractor = fyp_bot.truth_extractor:main', # Added for Step 1.2
        ],
    },
)