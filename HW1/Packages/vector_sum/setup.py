from setuptools import setup

package_name = 'vector_sum'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gs',
    maintainer_email='gsrinivasan@wpi.edu',
    description='Python Vector Sum',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = vector_sum.service_member_function:main',
            'client = vector_sum.client_member_function:main'
        ],
    },
)
