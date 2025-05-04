from setuptools import find_packages, setup

package_name = 'fractal_drawer_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nox',
    maintainer_email='nox@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fractal_drawer = fractal_drawer_turtlesim.fractal_drawer:main',
            'triangle_fractal_drawer = fractal_drawer_turtlesim.triangle_fractal_drawer:main',
            'koch_snowflake_drawer = fractal_drawer_turtlesim.koch_snowflake_drawer:main',
        ],
    },
)
