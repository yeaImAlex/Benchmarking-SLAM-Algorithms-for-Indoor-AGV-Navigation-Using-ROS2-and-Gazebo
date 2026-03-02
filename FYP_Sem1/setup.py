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
        
        # --- CONFIG & LUA INSTALLATION ---
        # This puts the .lua file in share/fyp_bot/config/cartographer/
        (os.path.join('share', package_name, 'config', 'cartographer'), 
            glob('config/cartographer/*.lua')),
        
        # Also include any .yaml files in the main config folder (like nav2_params.yaml)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # --- LAUNCH FILES ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # --- WORLDS & MODELS ---
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        
        (os.path.join('share', package_name, 'models/fyp_burger'), 
            glob('models/fyp_burger/*.sdf') + glob('models/fyp_burger/*.config')),
        
        (os.path.join('share', package_name, 'models/fyp_burger/urdf'), 
            glob('models/fyp_burger/urdf/*.urdf')),

        (os.path.join('share', package_name, 'models/L-shaped_corridor'), 
            glob('models/L-shaped_corridor/*.sdf') + glob('models/L-shaped_corridor/*.config')),
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
            'truth_extractor = fyp_bot.truth_extractor:main',
            # Adding your recording scripts here allows you to run them as ROS nodes
            'auto_record_AMCL = fyp_bot.auto_record_AMCL:main',
            'auto_record_GMapping = fyp_bot.auto_record_GMapping:main',
            'auto_record_Cartographer = fyp_bot.auto_record_Cartographer:main',
        ],
    },
)